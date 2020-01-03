#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  int lane = 1;
  //double d_car_prev = 2+4*lane;
  int desired_lane = 0;                                                     // This is the desired lane to be in
  double ref_vel = 0;                                                       // The desired speed setpoint between 0 and max speed
  int time_since_shift = 0;                                                 // variable to prevent hunting lane changes 
  double target_d = 0;                                                      // variable to terminate lane change maneuver saved between messages
  enum states                                                               // Finite State Machine states
  {   KL = 0,
      LCL = 1, 
      LCR = 2, 
      HOLD = 3 
    } state = HOLD;
  vector<double> nearest_ahead = {max_s, max_s, max_s};                     // Create distance measurement of each lanes free path forward
  vector<double> nearest_behind = {max_s, max_s, max_s};                    // Create distance measurement of open gap behind in each lane

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  h.onMessage([&nearest_ahead,&nearest_behind, &desired_lane, &lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &max_s, &time_since_shift, &state, &target_d]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      double max_vel = 49.5;
     
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();									//size of previous path from simulator
          
          json msgJson;

          if(prev_size >0){
            car_s = end_path_s;
          }
          
          // Setup vectors for closest vehicle in each lane
          nearest_ahead = {max_s, max_s-500.0, max_s-900.0};
          nearest_behind = {-max_s, -max_s, -max_s};
          
          bool too_close = false;
          
          // Set the vehicle's measured lane
          if (car_d<4){
            lane = 0;
          } else if (car_d>8){
            lane = 2;
          } else {
            lane =1;
          }
          
          // Iterate through the data available from sensor fusion
          for (int i = 0; i < sensor_fusion.size(); i++){
            // Pull each vehicle's data and calculate distance
            float d = sensor_fusion[i][6];
            double check_car_s = sensor_fusion[i][5];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx+vy*vy);
            check_car_s+= ((double)prev_size*.02*check_speed);
            double check_car_dist = check_car_s-car_s;
            
            // Check nearest car ahead in each lane and save in nearest ahead
            if(d<=4 && check_car_dist>=0 && check_car_dist<nearest_ahead[0]){
              nearest_ahead[0] = check_car_dist;
            } else if (d>=8 && check_car_dist>=0 && check_car_dist<nearest_ahead[2]){
              nearest_ahead[2] = check_car_dist;
            } else if (d>4 && d<8 && check_car_dist>=0 && check_car_dist<nearest_ahead[1]){
              nearest_ahead[1] = check_car_dist;
            }
            
            // Check nearest car behind in each lane and save in nearest behind
            if(d<=4 && check_car_dist<0 && check_car_dist>nearest_behind[0]){
              nearest_behind[0] = check_car_dist;
            } else if (d>=8 && check_car_dist<0 && check_car_dist>nearest_behind[2]){
              nearest_behind[2] = check_car_dist;
            } else if (d>4 && d<8 && check_car_dist<0 && check_car_dist>nearest_behind[1]){
              nearest_behind[1] = check_car_dist;
            }
            
            // Check if my vehicle's position is impeding on the vehicle in my lane directly in front of me
            if(d < (2+4*lane+2) && d> (2+4*lane-2)){
              if((check_car_s>car_s)&&((check_car_dist)<40)){
                too_close = true;
              }
            }
          }
          
          // Select the desired to be that which has the longest clear path
          desired_lane = std::max_element(nearest_ahead.begin(),nearest_ahead.end()) - nearest_ahead.begin();
          
          // Debugging printouts
          /*
          std::cout<<std::endl<<"Occupied: "<<lane<<" currently calculated: "<<std::endl;
          std::cout<<"Desired: "<<desired_lane<< "    " << nearest_ahead[0]<< "    " << nearest_ahead[1]<< "    " << nearest_ahead[2]<<std::endl;
          std::cout<<"Current: "<<desired_lane<< "    " << nearest_behind[0]<< "    " << nearest_behind[1]<< "    " << nearest_behind[2]<<std::endl;
          */

          // Finite State machine (could explore using switch/case statements)
          if (state == HOLD){
            time_since_shift++;                       // debug print out std::cout<<time_since_shift;
            if(time_since_shift>50)                   // hold lane for 5 seconds then transition to Keep Lane state
              state = KL;
          } else if(state == KL){
            // check car in other lanes gap range of s, then check right and left change if exists
            //eval lane changes
            // std::cout<<"Test KEEP_Lane"<<std::endl<<std::endl<<std::endl;
            double optimal = 0; // should really track vehicle velocity behind in cost function to prevent lane changees in front of faster vehicles
            
            // Setup cost function to enable opportune lane changes only when moving at speed (avoid collisions at low speed 
            // Cost function normalizes to 1, combines vehicle speed ratio and lane occupancy for 100m
            if (car_speed>0.5*max_vel){
              optimal = std::min(1.0,car_speed/max_vel*(std::min(std::max(std::max(nearest_ahead[0],nearest_ahead[1]),nearest_ahead[2]),100.0))/100);
            }
            // Setup lane change functionality if optimizer indicates to do so or if vehicle is too close (slower vehicle encountered)
            if(optimal>0.5 || too_close){
              int lane_error = desired_lane - lane;  // Setup lane error target and properly select motion direction state machine
              // Debug std::cout<<" Optimizer, fix lane error of: "<<lane_error<<std::endl;
              if (lane_error>0){
                state = LCR;
              } else if (lane_error<0){
                state = LCL;
              }
            }
            
          } else if(state == LCR){
            // Stay in LCR - Lane Change Right until executed 
            // This state looks for gap in traffic to execute safe move
            // This should have vehicle speed of adjacent vehicles and my vehicle being fed into a lookup table to determine safe gap size 
            if(nearest_ahead[lane+1]>30 && ((nearest_behind[lane+1]<-10 && car_speed>0.85*max_vel) 
                                            || (nearest_behind[lane+1]<-20 && car_speed>0.5*max_vel)
                                            || (nearest_behind[lane+1]<-60))){
              lane++;
              target_d = (2+lane*4-1.8);   // Hold Lane change until we've crossed threshold (debounce across lane boundary)
              //Debugging std::cout<<">>>>>>>SHIFT RIGHT>>>>>>>"<<std::endl;
            }
            if(car_d>target_d){     // if lane boundary zone is passed changes state to Hold to prevent hunting lane changes (should be in target lane)
              state=HOLD;
              time_since_shift = 0; // reset timer for Hold / anti-hunting function
            }
          } else if (state == LCL){
            if(nearest_ahead[lane-1]>30 && ((nearest_behind[lane-1]<-10 && car_speed>0.85*max_vel) 
                                            || (nearest_behind[lane-1]<-20 && car_speed>0.5*max_vel)
                                            || (nearest_behind[lane-1]<-60))){// need to implement fix for including vehicle speed 
              lane--;
              target_d = (2+lane*4+1.8);   // Hold Lane change until we've crossed threshold (debounce across lane boundary)
              // Debugging std::cout<<"<<<<<<<SHIFT LEFT<<<<<<<"<<std::endl;
            }
            if(car_d<target_d){     // if lane boundary zone is passed changes state to Hold to prevent hunting lane changes (should be in target lane)
              state=HOLD;
              time_since_shift = 0; // reset timer for Hold / anti-hunting function
            }
          }
          // Debugging std::cout<<"                                                    "<<lane<<" The state machine is in: "<<state ;
          
          // Change target speed based on proximity to vehicle in front and to ensure accel and jerk aren't violated
          if(too_close){
            ref_vel-=0.40;
          } else if (ref_vel <max_vel) {
            ref_vel+=0.5;
          }

          
          // Setup transformation point vectors
          vector<double> ptsx;
          vector<double> ptsy;
          
          //Pick out the last two points from the path planner to determine tangent path for next path planner iteration (continuity)
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_size<2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
         
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // Setup spline vector points to project swept path along a spline oriented
          // from the previous keep lane path points to include the new spline points
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s, map_waypoints_x, map_waypoints_y);
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          // Transform the coordinates to a unique frame (unique y for each x)
          for (int i = 0; i<ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }
          
          // Initialize and build spline
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          // Intialize and propagate next points to be passed to simulator
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }          
        
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          double x_add_on = 0;
          
          for (int i = 1; i<= 50 - previous_path_x.size(); i++){
            double N = (target_dist/(.02*ref_vel/2.24)); // convert mph to m/s
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            //rotate back
            
            x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


          
          
          
          // Provided Code
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}