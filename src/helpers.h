#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;


/**
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
//-----------------------------------------------


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
  }

int safe_lane_change_check(vector <vector <double>> sensor_fusion,int lane, int intended_lane, int remaining_path_ahead_size, double car_s)
  { 
    double VHCLE_i_speed;// speed of an other vehicle i
    double VHCLE_i_s; // Frenet "s" longitudinal s coordinate of vehicle i    
    double dist2_VHCLE_i;// longitudinal Frenet distance (approximation) from ego to vehicle i 
    float d_i; // vehicle i transversal Frenet coordinate
    bool unsafe_change=false;// Are we tailgating ??

    // check if the intended is not off-road

    if (intended_lane>2 or intended_lane<0)
      {
        return(lane);
      }

    else 
      {

      for(int i=0;i<sensor_fusion.size();i++)// checkall the other vehicles sensed 

        {
          d_i=sensor_fusion[i][6];  

          if  (fabs((intended_lane*4)+2-d_i)<2) // check if other vehicle is in Ego car's lane
            {
              VHCLE_i_speed=dist(0,0,sensor_fusion[i][3], sensor_fusion[i][4]); // other vehicle i speed    

              VHCLE_i_s = sensor_fusion[i][5] ; //other vehicle s longitudinal coordinate as given by sensor
              
            //  ego car s position was antcipated at the end of the previous path ( see : car_s=end_path_s)
            //  other vehicles s position have to be antipated similarly 
            
              VHCLE_i_s+=VHCLE_i_speed * 0.02 * remaining_path_ahead_size;// anticipation given that the ego car drives in 0.02 between each points   

              dist2_VHCLE_i=VHCLE_i_s - car_s; // distance vehicle i to ego vehicle   

              if (fabs(dist2_VHCLE_i)<30) // wheter the side vehicle is ahead or behind too risky
                {
                  return(lane);// do not change lane
                }
             }            
          }

          return(intended_lane);
        }
  }



bool side_gap_clearance(vector <vector <double>> sensor_fusion,int intended_lane, int remaining_path_ahead_size, double car_s)
  { 
    double VHCLE_i_speed;// speed of an other vehicle i
    double VHCLE_i_s; // Frenet "s" longitudinal s coordinate of vehicle i    
    double dist2_VHCLE_i;// longitudinal Frenet distance (approximation) from ego to vehicle i 
    float d_i; // vehicle i transversal Frenet coordinate
    bool unsafe_change=false;// Are we tailgating ??

      for(int i=0;i<sensor_fusion.size();i++)// checkall the other vehicles sensed 
        {
          d_i=sensor_fusion[i][6];  

          if  (fabs((intended_lane*4)+2-d_i)<2) // check if other vehicle is in Ego car's lane
            {
              VHCLE_i_speed=dist(0,0,sensor_fusion[i][3], sensor_fusion[i][4]); // other vehicle i speed    

              VHCLE_i_s = sensor_fusion[i][5] ; //other vehicle s longitudinal coordinate as given by sensor
              
            //  ego car s position was antcipated at the end of the previous path ( see : car_s=end_path_s)
            //  other vehicles s position have to be antipated similarly 
            
              VHCLE_i_s+=VHCLE_i_speed * 0.02 * remaining_path_ahead_size;// anticipation given that the ego car drives in 0.02 between each points   

              dist2_VHCLE_i=VHCLE_i_s - car_s; // distance vehicle i to ego vehicle   

              if (fabs(dist2_VHCLE_i)<30) // wheter the side vehicle is ahead or behind too risky
               {
                  unsafe_change=true;// reduce speed when vehicle ahead is too close
              }
                
             }            
          }

          return(unsafe_change);
  }

bool front_clearance(vector <vector <double>> sensor_fusion,int lane, int remaining_path_ahead_size, double car_s, float warning_distance, double &invader_d)
  { 
    double VHCLE_i_speed;// speed of an other vehicle i
    double VHCLE_i_s; // Frenet "s" longitudinal s coordinate of vehicle i    
    double dist2_VHCLE_i;// longitudinal Frenet distance (approximation) from ego to vehicle i 
    float d_i; // vehicle i transversal Frenet coordinate
    bool tail_gating=false;// Are we tailgating ??

      for(int i=0;i<sensor_fusion.size();i++)// checkall the other vehicles sensed 
        {
          d_i=sensor_fusion[i][6];  

          if  (fabs((lane*4)+2-d_i)<2) // check if other vehicle is in Ego car's lane
            {
              VHCLE_i_speed=dist(0,0,sensor_fusion[i][3], sensor_fusion[i][4]); // other vehicle i speed    

              VHCLE_i_s = sensor_fusion[i][5] ; //other vehicle s longitudinal coordinate as given by sensor
              
            //  ego car s position was antcipated at the end of the previous path ( see : car_s=end_path_s)
            //  other vehicles s position have to be antipated similarly 
            
              VHCLE_i_s+=VHCLE_i_speed * 0.02 * remaining_path_ahead_size;// anticipation given that the ego car drives in 0.02 between each points   

              dist2_VHCLE_i=VHCLE_i_s - car_s; // distance vehicle i to ego vehicle   

              if ((dist2_VHCLE_i>=0)  &&  (dist2_VHCLE_i<warning_distance)) // only check if the vehicles ahead of the ego car are at less than 30 meters 
                {
                  tail_gating=true;// reduce speed when vehicle ahead is too close

                  invader_d=sensor_fusion[i][6];
                } 
             }            
          }

          return(tail_gating);
  }




#endif  // HELPERS_H