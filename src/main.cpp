#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <algorithm>
#include <thread>
#include <limits>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2) {
	return hypot(x2-x1,y2-y1);
}

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {

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

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2(map_y-y,map_x-x);

	double angle = fabs(theta-heading);
  angle = std::min(2.0*M_PI - angle, angle);

  if(angle > M_PI/4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
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
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, 
  const std::vector<double> &maps_y) {
	
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

// argsort a vector 
std::vector<int> sort_indices(const std::vector<double> &v) {
  // initialize original index locations
  std::vector<int> idx(v.size());
  iota(idx.begin(), idx.end(), 0);
  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](int i1, int i2) {return v[i1] > v[i2];});
  return idx;
}

// ego car at s0, and threat at s1.  How far ahead is the threat vehicle? 
// answer is always positive 
double get_distance_ahead_on_track(const double s0, const double s1, const double track_length) {
  double diff = s1 - s0;
  while(diff<0.0) {
    diff += track_length;
  }
  while(diff>track_length) {
    diff -= track_length;
  }
  return diff;
}

// ego car at s0, threat at s1.  WHat is relative distance between vehicles?
// answer is from -0.5*track_length to 0.5*track_length
double get_relative_s(const double s0, const double s1, const double track_length) {
  double diff = s1 - s0;
  while(diff<-0.5*track_length) {
    diff += track_length;
  }
  while(diff>0.5*track_length) {
    diff -= track_length;
  }
  return diff;

}

// given the current traffic pattern, 
// for each lane, determine distance to blockage, and the speed of that blockage.
std::pair<std::vector<double>, std::vector<double> > analyze_traffic_in_each_lane(const std::vector<std::vector<double> >& tracks, 
        const double car_s, const int num_lanes, const double lane_width, const double track_length, 
        const double look_ahead_distance, const double max_vel) {

    // initialize output
    std::vector<double> exit_speed_by_lane(num_lanes,max_vel);
    std::vector<double> minimum_distance_to_blockage(num_lanes,std::numeric_limits<double>::infinity());

    // 0 car's unique ID, 
    // 1 car's x position in map coordinates, 
    // 2 car's y position in map coordinates, 
    // 3 car's x velocity in m/s, 
    // 4 car's y velocity in m/s, 
    // 5 car's s position in frenet coordinates, 
    // 6 car's d position in frenet coordinates;
    
    const double road_width = num_lanes * lane_width;

    // look through each track
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

// prioritize lanes based on exit velocity
std::vector<int> prioritize_lanes(const int car_current_lane, const std::vector<double>& exit_speed_by_lane, const std::vector<double>& minimum_distance_to_blockage) {

  const int num_lanes = exit_speed_by_lane.size();
  const double speed_verse_changing_lane_preference = 0.01;

  std::vector<double> modified_exit_speed_by_lane = exit_speed_by_lane;
  modified_exit_speed_by_lane[car_current_lane] += speed_verse_changing_lane_preference;                   //favor staying in current lane
  if(car_current_lane>0) modified_exit_speed_by_lane[car_current_lane-1] += 0.5*speed_verse_changing_lane_preference;  //favor changing to faster lane
  if(car_current_lane<num_lanes-1) modified_exit_speed_by_lane[car_current_lane+1] += 0.25*speed_verse_changing_lane_preference; // favor changing only one lane

  std::vector<int> prioritized_lanes = sort_indices(modified_exit_speed_by_lane);

  return prioritized_lanes;

}

// build a trajectory to a desired target lane
std::pair<std::vector<double>,std::vector<double> > continue_to_lane(const int target_lane, 
            const std::vector<double>& previous_path_x,
            const std::vector<double>& previous_path_y,
            const double future_car_x,
            const double future_car_y,
            const double future_car_yaw,
            const double future_car_s,
            const double future_car_d,
            const double future_car_speed,
            const int proposed_path_length,
            const double time_step,
            const double maximum_speed_control,
            const double maximum_acceleration_control,
            const double maximum_jerk_control,
            const double safe_following_distance,
            const std::vector<std::vector<double>>& sensor_fusion,
            const double lane_width,
            const double track_length,
            const std::vector<double>& map_waypoints_s,
            const std::vector<double>& map_waypoints_x,
            const std::vector<double>& map_waypoints_y) {

  // std::cout << "continue_to_lane ..." << std::endl;

  std::vector<double> target_path_x;
  std::vector<double> target_path_y;

    // generate the points that will go into the spline
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  const int prev_size = previous_path_x.size();
  double ref_x = future_car_x;
  double ref_y = future_car_y;
  double ref_yaw = future_car_yaw;
  double ref_speed = future_car_speed;

  // prime the pump on the spline with the last previously requested points
  if(prev_size<2) {  // we have less than two points in our history
    const double prev_car_x = future_car_x - cos(future_car_yaw);
    const double prev_car_y = future_car_y - sin(future_car_yaw);
    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);
    ptsx.push_back(future_car_x);
    ptsy.push_back(future_car_y);

  } 
  else {  // add the last two previously requested points to get us started
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ptsx.push_back(ref_x_prev);
    ptsy.push_back(ref_y_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y);
  }

   // now we get to add new points in the XY frame
  std::vector<double> next_wp0 = getXY(future_car_s + 35.0,lane_width*(0.5+target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
  std::vector<double> next_wp1 = getXY(future_car_s + 65.0,lane_width*(0.5+target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
  std::vector<double> next_wp2 = getXY(future_car_s + 95.0,lane_width*(0.5+target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsy.push_back(next_wp0[1]);
  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);

  // shift points to local xy frame of the car
  for(int i=0;i<ptsx.size(); ++i) {
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
    ptsx[i] =  shift_x * cos(ref_yaw) + shift_y*sin(ref_yaw);
    ptsy[i] = -shift_x * sin(ref_yaw) + shift_y*cos(ref_yaw);
  }

  //create the spline
  tk::spline xyspline;
  xyspline.set_points(ptsx,ptsy);    

  const double target_x = 30.0;
  const double target_y = xyspline(target_x);
  const double target_distance = hypot(target_x,target_y);

  // start off at the end of the previous path
  double current_x_global = ref_x;
  double current_y_global = ref_y;

  double current_x_local = 0.0;
  double current_y_local = 0.0;

  for(int i=1;i<=proposed_path_length-prev_size;++i) {  //for each point into the future

    // Determine acceleration

    // find the next obstacle in our lane
    double min_distance_ahead = std::numeric_limits<double>::infinity();
    double speed_of_next_threat = std::numeric_limits<double>::infinity();
    for(int tt=0;tt<sensor_fusion.size();++tt) {
      const double threat_d = sensor_fusion[tt][6];
      const int threat_lane = int(threat_d/lane_width);
      if(threat_lane==target_lane) {
        std::vector<double> ego_sd = getFrenet(current_x_global, current_y_global, ref_yaw, map_waypoints_x, map_waypoints_y);
        const double ego_s = ego_sd[0];
        const double ego_d = ego_sd[1];
        const double threat_s0 = sensor_fusion[tt][5];
        const double threat_vx = sensor_fusion[tt][3];
        const double threat_vy = sensor_fusion[tt][4];
        const double threat_speed = hypot(threat_vx,threat_vy);
        const double threat_s = threat_s0 + i*time_step*threat_speed;
        const double distance_ahead = get_distance_ahead_on_track(ego_s,threat_s,track_length);
        if(distance_ahead<min_distance_ahead) {
          min_distance_ahead = distance_ahead;
          speed_of_next_threat = threat_speed;
        }
      }
    }

    // adjust our speed accordingly
    double safe_stopping_distance = safe_following_distance + (speed_of_next_threat-ref_speed)*(speed_of_next_threat-ref_speed)/(maximum_acceleration_control);

    if(safe_stopping_distance>min_distance_ahead) {   //we need to slow down
      if(ref_speed>speed_of_next_threat) {
        ref_speed -= time_step * 0.5*maximum_acceleration_control;
      }
      else {
        ref_speed += time_step * 0.5*maximum_acceleration_control;
      }
    }
    else {
      ref_speed += time_step * 0.5*maximum_acceleration_control;
    }
    ref_speed = std::max(ref_speed,0.0);
    ref_speed = std::min(ref_speed,maximum_speed_control);  // always ensure that we obey speed limit

    const double N = target_distance / (time_step*ref_speed);
    current_x_local += target_x/N;
    current_y_local = xyspline(current_x_local);

    //shift back to global XY frame
    current_x_global = ref_x + current_x_local*cos(ref_yaw)-current_y_local*sin(ref_yaw);
    current_y_global = ref_y + current_x_local*sin(ref_yaw)+current_y_local*cos(ref_yaw);

    target_path_x.push_back(current_x_global);
    target_path_y.push_back(current_y_global);

  }
  // std::cout << "continue_to_lane ... OK" << std::endl;
  return std::make_pair(target_path_x,target_path_y);

}

// determine if it is safe to change lanes
double look_if_faster_lane_is_safe(const int future_car_lane, const double future_car_s, const double future_car_speed, 
  const double future_delta_t, const std::vector<int>& prioritized_lanes, std::vector<std::vector<double>> sensor_fusion, const double track_length, const double lane_width) {

  int next_lane_selection = future_car_lane;  //default lane selection

  for(int ll=0;ll<prioritized_lanes.size();++ll) {        // look through list of prioritized lanes
    int candidate_lane = prioritized_lanes[ll];     // this is our candidate lane

    if(candidate_lane==future_car_lane) {  // we are already in a good lane
      break;
    }

    if(abs(candidate_lane-future_car_lane)==2) {  // If we are trying to change two lanes at once, just move to the center lane
      candidate_lane = 1;
    } 

    if(abs(candidate_lane-future_car_lane)==1) {          // only look at adjacent lanes
      bool lane_is_safe = true;
      for(int tt=0;tt<sensor_fusion.size();++tt) {        // look at each threat
        
        const double threat_d = sensor_fusion[tt][6]; 
        const int threat_lane = int(threat_d/lane_width);
                             // is this threat in the candidate lane
        if(threat_lane==candidate_lane) {  

          const double threat_s = sensor_fusion[tt][5];
          const double threat_vx = sensor_fusion[tt][3];
          const double threat_vy = sensor_fusion[tt][4];
          const double threat_speed = hypot(threat_vx,threat_vy);
          const double threat_future_s = threat_s + future_delta_t * threat_speed;

          const double relative_s = get_relative_s(future_car_s,threat_future_s,track_length);
          if(relative_s>-14 and relative_s<24) {    // is there enough space to change lanes?
            lane_is_safe = false;    // Not safe to move to this lane
            break;
          }
        }
      }
      if(lane_is_safe) {  // we found a safe lane
        next_lane_selection = candidate_lane;
        break;
      }
    }
  }
  return next_lane_selection;
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
  const int proposed_path_length = 50;
  const double maximum_speed_control = 49.6 * 1609.34 / 3600.0;
  const double maximum_acceleration_control = 9.0;
  const double maximum_jerk_control = 9.0;
  const double safe_following_distance = 15.0;
  const double look_ahead_distance = 200.0;

  // Initial state for FSM
  int current_target_lane = 1;

  /////////////////////////////////////////////////////
  // MAP 

  std::string map_file_ = "../data/highway_map.csv";  // Waypoint map to read from
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  std::string line;
  while (getline(in_map_, line)) {
  	std::istringstream iss(line);
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
  h.onMessage([&current_target_lane,&safe_following_distance,&maximum_jerk_control,&maximum_acceleration_control,&look_ahead_distance,&lane_width,&track_length,&maximum_speed_control,&time_step,
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
        
        std::string event = j[0].get<std::string>();
        
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

          ///////////////////////////////////////////////////////////////////////////
          // Vectors for points to return to simulator
          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          // Add points from the previous path 
          for(int i = 0; i < prev_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          ////////////////////////////////////////////////////////////////////////////
          // Analyze the current car state
          double future_car_x = car_x;
          double future_car_y = car_y;
          double future_car_yaw = car_yaw;
          double future_car_s = car_s;
          double future_car_d = car_d;
          double future_car_speed = car_speed;
          if(prev_size > 0) {
            future_car_x = previous_path_x[prev_size-1];
            future_car_y = previous_path_y[prev_size-1];
            future_car_s = end_path_s;
            future_car_d = end_path_d;
          }     
          if(prev_size > 1) {
            const double x1 = previous_path_x[prev_size-1];
            const double x0 = previous_path_x[prev_size-2];
            const double y1 = previous_path_y[prev_size-1];
            const double y0 = previous_path_y[prev_size-2];
            future_car_yaw = atan2(y1-y0,x1-x0);
            future_car_speed = hypot(x1-x0,y1-y0) / time_step;
          }

          const int future_car_lane = int(future_car_d/lane_width);
          const double future_delta_t = time_step * prev_size;

          /////////////////////////////////////////////////////////////////////////////
          // Analyze current traffic conditions
          std::vector<double> minimum_distance_to_blockage;
          std::vector<double> exit_speed_by_lane;
          std::tie(minimum_distance_to_blockage,exit_speed_by_lane) = analyze_traffic_in_each_lane(sensor_fusion, 
              future_car_s, num_lanes, lane_width, track_length, look_ahead_distance, maximum_speed_control);



          //////////////////////////////////////////////////////////////////////////////////
          // Order the lanes by optimality: fastest to slowest
          std::vector<int> prioritized_lanes = prioritize_lanes(future_car_lane,exit_speed_by_lane,minimum_distance_to_blockage);
          std::cout << "Lane " << prioritized_lanes[0] << " is fastest" << std::endl;

          /////////////////////////////////////////////////////////////////////////////////
          // Decide which lane to target
          if(future_car_lane==current_target_lane) {
            const double goal_error = fabs(end_path_d - (current_target_lane+0.5)*lane_width);
            if(goal_error<0.5) {     // we have arrived in the lane. Do we want another lane?
              current_target_lane = look_if_faster_lane_is_safe(future_car_lane, future_car_s, future_car_speed, future_delta_t, prioritized_lanes, sensor_fusion, track_length, lane_width);
              if(current_target_lane!=future_car_lane) std::cout << "Switching from lane " << future_car_lane << " to " << current_target_lane << std::endl;
            }
          }

          ////////////////////////////////////////////////////////////////////////////////
          // Move towards the targeted lane
          std::vector<double> candidate_path_x;
          std::vector<double> candidate_path_y;
          std::tie(candidate_path_x,candidate_path_y) = continue_to_lane(current_target_lane, previous_path_x, previous_path_y,
              future_car_x,
              future_car_y,
              future_car_yaw,
              future_car_s,
              future_car_d,
              future_car_speed,
              proposed_path_length,
              time_step,
              maximum_speed_control,
              maximum_acceleration_control,
              maximum_jerk_control,
              safe_following_distance,
              sensor_fusion,
              lane_width,
              track_length,
              map_waypoints_s,
              map_waypoints_x,
              map_waypoints_y);

          //////////////////////////////////////////////////////////////////////////////////
          // Add the proposed path onto the list of next values
          next_x_vals.insert(next_x_vals.end(),candidate_path_x.begin(), candidate_path_x.end());
          next_y_vals.insert(next_y_vals.end(),candidate_path_y.begin(), candidate_path_y.end());

          //////////////////////////////////////////////////////////////////////////////////
          // Populate the json message
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
