#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <limits>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2) {
	return hypot(x2-x1,y2-y1);
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) {

	double closestLen = std::numeric_limits<double>::infinity(); //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++) {
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2(map_y-y,map_x-x);

	double angle = fabs(theta-heading);
  angle = min(2.0*M_PI - angle, angle);

  if(angle > M_PI/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0) {
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

	if(centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++) {
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
	
  int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) )) {
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-M_PI/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


std::vector<int> sort_indices(const std::vector<double> &v) {

  // initialize original index locations
  std::vector<int> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](int i1, int i2) {return v[i1] > v[i2];});

  return idx;
}

double get_distance_ahead_on_track(const double s0, const double s1, const double track_length) {

  assert(s0>=0.0);
  assert(s0<=track_length);
  assert(s1>=0.0);
  assert(s1<=track_length);
  
  double s1p = s1;
  if(s1p<s0) {
    s1p+=track_length;
  }
  
  assert(s1p>=s0);

  return s1p-s0;

}

std::pair<std::vector<double>, std::vector<double> > analyze_traffic_in_each_lane(const std::vector<std::vector<double> >& tracks, 
        const double car_s, const int num_lanes, const double lane_width, const double track_length, 
        const double look_ahead_distance, const double max_vel) {

    // initialize output
    std::vector<double> exit_speed_by_lane(num_lanes,max_vel);
    std::vector<double> minimum_distance_to_blockage(num_lanes,std::numeric_limits<double>::infinity());

    const double road_width = num_lanes*lane_width;

    // 0 car's unique ID, 
    // 1 car's x position in map coordinates, 
    // 2 car's y position in map coordinates, 
    // 3 car's x velocity in m/s, 
    // 4 car's y velocity in m/s, 
    // 5 car's s position in frenet coordinates, 
    // 6 car's d position in frenet coordinates;
    
    for(size_t ii=0;ii<tracks.size();++ii) {
      
      const double s = tracks[ii][5];

      const double distance_ahead = get_distance_ahead_on_track(car_s,s,track_length);
      if(distance_ahead<look_ahead_distance) {     // only look at cars that are within the next 1000 meters in front of us
        const double d = tracks[ii][6];
        if(d>=0.0 and d<=road_width) {              // is this car even on the road?
          const int threat_lane = int(d/lane_width);

          assert(threat_lane>=0);
          assert(threat_lane<num_lanes);

          if(minimum_distance_to_blockage[threat_lane] > distance_ahead) {
            minimum_distance_to_blockage[threat_lane] = distance_ahead;
            const double vx = tracks[ii][3];
            const double vy = tracks[ii][4];
            const double speed = hypot(vx,vy);
            exit_speed_by_lane[threat_lane] = speed;
          }
        } 
      }       
    }

    return std::make_pair(minimum_distance_to_blockage,exit_speed_by_lane);

}

std::vector<int> prioritize_lanes(const int car_current_lane, const std::vector<double>& exit_speed_by_lane, const std::vector<double>& minimum_distance_to_blockage) {

  const int num_lanes = exit_speed_by_lane.size();
  const double speed_verse_changing_lane_preference = 0.01;

  std::vector<double> modified_exit_speed_by_lane = exit_speed_by_lane;
  modified_exit_speed_by_lane[car_current_lane] += speed_verse_changing_lane_preference;                   //favor staying in current lane
  if(car_current_lane>0) modified_exit_speed_by_lane[car_current_lane-1] += 0.5*speed_verse_changing_lane_preference;  //favor changing only one lane
  if(car_current_lane<num_lanes-1) modified_exit_speed_by_lane[car_current_lane+1] += 0.25*speed_verse_changing_lane_preference; // favor changing only one lane

  std::vector<int> prioritized_lanes = sort_indices(modified_exit_speed_by_lane);

  return prioritized_lanes;

}


int main() {

  ///////////////////////////////////////////////////
  // PARAMETERS

  // track parameters
  const double track_length = 6945.554;
  const double lane_width = 4.0;
  const int num_lanes = 3;

  // simulator parameters
  const double time_step = 0.02;

  // vehicle control parameters
  const double maximum_speed = 49.5;
  const double look_ahead_distance = 500.0;

  /////////////////////////////////////////////////////
  // MAP 

  string map_file_ = "../data/highway_map.csv";  // Waypoint map to read from
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x,y;
  	float s,d_x,d_y;
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

  //////////////////////////////////////////////////////////
  // SOCKET CALLBACK

  uWS::Hub h;
  h.onMessage([&look_ahead_distance,&lane_width,&track_length,&maximum_speed,&time_step,
    &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
   uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

          ///////////////////////////////////////////////////
          // READ TELEMETRY DATA

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
          const int prev_size = previous_path_x.size(); 

          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          std::vector<std::vector<double> > sensor_fusion = j[1]["sensor_fusion"];

          ////////////////////////////////////////////////////////////////////////////
          const int car_current_lane = int(car_d/lane_width);

          double future_car_s = car_s;
          if(prev_size > 0) {
          future_car_s = end_path_s;
          }     

          // Analyze current traffic conditions
          std::vector<double> minimum_distance_to_blockage;
          std::vector<double> exit_speed_by_lane;
          std::tie(minimum_distance_to_blockage,exit_speed_by_lane) = analyze_traffic_in_each_lane(sensor_fusion, 
          car_s, num_lanes, lane_width, track_length, 
          look_ahead_distance, maximum_speed);

          //order the lanes by optimality: fastest to slowest,  if a tie for fastest, use current lane.
          std::vector<int> prioritized_lanes = prioritize_lanes(car_current_lane,exit_speed_by_lane,minimum_distance_to_blockage);
          
          std::cout << "Lane " << prioritized_lanes[0] << " is fastest" << std::endl;

          int car_desired_lane = car_current_lane;
          for(int kk=0;kk<num_lanes;++kk) {
            const int candidate_lane = prioritized_lanes[kk];

            if(abs(candidate_lane-car_current_lane)<2) {
              if(minimum_distance_to_blockage[candidate_lane]>20.0) {
                car_desired_lane = candidate_lane; 
                break;
              }
              else {
                std::cout << "can't move to lane " << candidate_lane << std::endl;
                std::cout << "minimum_distance_to_blockage : " << minimum_distance_to_blockage[car_desired_lane] << std::endl;
              }
            }
          }


        // const double acceleration_required = 

         const double ref_vel = 49.5;

         std::vector<double> ptsx;
         std::vector<double> ptsy;

         double ref_x = car_x;
         double ref_y = car_y;
         double ref_yaw = deg2rad(car_yaw);

         if(prev_size<2) {

          const double prev_car_x = car_x - cos(car_yaw);
          const double prev_car_y = car_y - sin(car_yaw);

          ptsx.push_back(prev_car_x);
          ptsx.push_back(car_x);

          ptsy.push_back(prev_car_y);
          ptsy.push_back(car_y);


        } else {

          ref_x = previous_path_x[prev_size-1];
          ref_y = previous_path_y[prev_size-1];

          double ref_x_prev = previous_path_x[prev_size-2];
          double ref_y_prev = previous_path_y[prev_size-2];

          ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

          ptsx.push_back(ref_x_prev);
          ptsx.push_back(ref_x);

          ptsy.push_back(ref_y_prev);
          ptsy.push_back(ref_y);

         }

         std::vector<double> next_wp0 = getXY(future_car_s + 30.0,2+4*car_desired_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
         std::vector<double> next_wp1 = getXY(future_car_s + 60.0,2+4*car_desired_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
         std::vector<double> next_wp2 = getXY(future_car_s + 90.0,2+4*car_desired_lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);

         ptsx.push_back(next_wp0[0]);
         ptsx.push_back(next_wp1[0]);
         ptsx.push_back(next_wp2[0]);

         ptsy.push_back(next_wp0[1]);
         ptsy.push_back(next_wp1[1]);
         ptsy.push_back(next_wp2[1]);


         for(int i=0;i<ptsx.size(); ++i) {

          double shift_x = ptsx[i]-ref_x;
          double shift_y = ptsy[i]-ref_y;

          ptsx[i] = (shift_x * cos(0.0-ref_yaw)-shift_y*sin(0.0-ref_yaw));
          ptsy[i] = (shift_x * sin(0.0-ref_yaw)+shift_y*cos(0.0-ref_yaw));

         }

         tk::spline s;
         s.set_points(ptsx,ptsy);    // currently it is required that X is already sorted

         vector<double> next_x_vals;
         vector<double> next_y_vals;

         for(int i = 0; i < previous_path_x.size(); ++i) {
          next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
         }

         const double target_x = 30.0;
         const double target_y = s(target_x);
         const double target_dist = hypot(target_x,target_y);
         double x_add_on = 0.0;

         for(int i=1;i<=50-previous_path_x.size();++i) {

          const double N = target_dist / (time_step*ref_vel/2.24);
          double x_point = x_add_on + target_x/N;
          double y_point = s(x_point);

          x_add_on = x_point;

          const double x_ref = x_point;
          const double y_ref = y_point;

          x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
          y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);

          x_point += ref_x;
          y_point += ref_y;

          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);

         }





          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
   size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
