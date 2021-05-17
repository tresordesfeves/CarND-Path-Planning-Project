
//   mkdir build && cd build to create and enter the build directory
//   cmake .. && make to compile your project
//  ./path_planning to run your code



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

int lane =1; //ego car's lane : waypoints will be added ahead on this lane

double ref_vel_mph = 49.5;  /*
                          the ego vehicle velocity : 
                          the car moves from one waypoint to the nextone at fixed time intervals (0.02 s) therefore
                          the spacing of the way points to generate is conditioned by this velocity
                          */

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s, // passing the extrinsic parameters (map)
               &map_waypoints_dx,&map_waypoints_dy,&lane, &ref_vel_mph]// passing the ego vehicle intrinsic parameters( velocity ,lane)
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

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
          

          int remaining_path_ahead_size =previous_path_x.size(); /* remaining waypoints ahead since generating the last batch
                                                        ie  waypoints not driven on yet */

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

           /*
           Discrete waypoints will be generated using a Spline interpolation based on these 5 points
           */
          vector<double> base_WP_x; 
          vector<double> base_WP_y;


          double ref_x,ref_y,ref_yaw /* origin and orientation of a coordinate space centered:
                                      - either on the last previous point ( when the tarjectory is not starting)
                                      - or on the car itself ( trajectory starting, no previous points)
                                      and oriented :
                                      - either like  the last 2 points 
                                      - or like the car
                                      */

        // picking the two first base points for the Spline:

          if remaining_path_ahead_size<2 
            { // only one or no waypoint remaining from the previous path :
              base_WP_x.push_back(car_x-cos(deg2rad(car_yaw)));// creating a new base point behind the car and tangent to the car angle
              base_WP_y.push_back(car_y-cos(deg2rad(car_yaw)));// creating a new base point behind the car and tangent to the car angle
              base_WP_x.push_back(car_x);//pick the car for second base point 
              base_WP_y.push_back(car_y);//pick the car for second base point 

              // for later use to switch to space referential centered on ego car 
              ref_x= car_x; 
              ref_y=car_y;
              ref_yaw= car_yaw;
              
            }
          else 
            { // use the last two remaining waypoints from previous_path
              base_WP_x.push_back(previous_path_x[remaining_path_ahead_size-2]);
              base_WP_y.push_back(previous_path_y[remaining_path_ahead_size-2]);
              base_WP_x.push_back(previous_path_x[remaining_path_ahead_size-1]);
              base_WP_y.push_back(previous_path_y[remaining_path_ahead_size-1]);


              // for later use to switch to space referential centered on last previius point, and last 2 points direction
              ref_x= previous_path_x[remaining_path_ahead_size-1];
              ref_y= previous_path_y[remaining_path_ahead_size-1];
              
              ref_yaw= atan2(previous_path_y[remaining_path_ahead_size-2]-ref_y,previous_path_x[remaining_path_ahead_size-2]-ref_x);

            }

        // done picking the two first base points for the Spline:

          // create  3 way points ahead 30 meters apart to anchor a Spline base discretization 
          
          // ref_x, ref_y, ref_yaw????

              double spacer = 30; //space between each spline base points in Frenet coordinates
              int sparsed_base_points=3;  // number of sparsed base waypoints
              
              for(int i=1; i<=sparsed_base_points;i++)
                {
                  base_WP_x.push_back (getXY(car_s+(i*spacer),2+(4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y)[0]);
                  base_WP_y.push_back (getXY(car_s+(i*spacer),2+(4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y)[1]);
                }

        // transform all 5 basepoints from map space coordinates to car space coordinates (or the last point of the previous path )

              double shift_ref_x,shift_ref_y;     

              for(int i=0; i<base_WP_x.size();i++)
                {
                  // translation to reference  origin (car or last previous point )
                  shift_ref_x= base_WP_x[i]- ref_x;
                  shift_ref_y= base_WP_x[i]- ref_y;   

                  // rotation of the axis to match the car or last 2 points direction 
                  base_WP_x= (shift_ref_x* cos(ref_yaw))+ (shift_ref_y*sin(ref_yaw));
                  base_WP_x= (-shift_ref_x* sin(ref_yaw))+ (shift_ref_y*cos(ref_yaw));  
                }

// to do the spline : on video youtube: https://youtu.be/7sI3VHFPP0w?t=1763



//**********
          //* TODO(1): define a path made up of (x,y) points that the car will visit
          double next_s, next_d;
          double dist_inc = 0.49;
          vector<double>car_XY;


          for (int i = 0; i < 50; ++i) 
            {
              next_s= car_s+((i+1)*dist_inc); // longitudinal distance along the s axis 
              next_d= 6;// car is centered on lane 1 (3 lanes, from left to right: L0,L1,l2, lane width = 4 )
              car_XY=getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
              next_x_vals.push_back(car_XY[0]);
              next_y_vals.push_back(car_XY[1]);
            }
           //* END(1)                              

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