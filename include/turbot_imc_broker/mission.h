// The MIT License (MIT)

// Copyright (c) 2017 Universitat de les Illes Balears

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

#ifndef INCLUDE_TURBOT_IMC_BROKER_MISSION_H_
#define INCLUDE_TURBOT_IMC_BROKER_MISSION_H_

#include <ros/ros.h>
#include <math.h>

#include <turbot_imc_broker/ned.h>

// Base IMC template
#include <ros_imc_broker/ImcTypes.hpp>
#include <IMC/Base/Packet.hpp>
#include <IMC/Spec/AllMessages.hpp>
#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/Goto.hpp>
#include <IMC/Spec/StationKeeping.hpp>
#include <IMC/Spec/FollowPath.hpp>
#include <IMC/Spec/Loiter.hpp>

class NEPoint {
 public:
  NEPoint() : north(0), east(0) {
    // empty
  }

  NEPoint(const double& n, const double& e) : north(n), east(e) {
    // empty
  }

  double DistanceTo(const NEPoint& other) const {
    return sqrt(pow(north - other.north, 2.0) + pow(east - other.east, 2.0));
  }

  double north;
  double east;
};

/**
 * @brief      Class for mission point.
 */
class MissionPoint : public NEPoint {
 public:
  MissionPoint() : NEPoint(), z(0), speed(0), duration(-1),
                   radius(0), is_altitude(false) {
    // empty
  }

  double z;
  double yaw;
  double speed;
  double duration;
  double radius;
  bool is_altitude;  //! if true, depth is altitude
};

enum MissionState {
  MISSION_EMPTY,
  MISSION_LOADED,
  MISSION_RUNNING,
  MISSION_STATION_KEEPING, // mission non stoped by neptus but in a station keeping mode
  MISSION_STOPPED,
  MISSION_FAILED,
  MISSION_ABORTED
};

/**
 * @brief      Class for mission.
 */
class Mission {
 public:
  Mission() : mission_length_(0), points_idx_(0) {
    ros::NodeHandle nh("~");

    std::string param_ned_lat;
    std::string param_ned_lon;
    nh.param<std::string>("param_ned_lat", param_ned_lat, std::string("/navigator/ned_origin_lat"));
    nh.param<std::string>("param_ned_lon", param_ned_lon, std::string("/navigator/ned_origin_lon"));
    state_ = MISSION_EMPTY;

    // Get NED origin
    double ned_lat = 0.0, ned_lon = 0.0;
    initial_distance_ = 0.0;
    if (nh.hasParam(param_ned_lat) && nh.hasParam(param_ned_lon)) {
      nh.getParamCached(param_ned_lat, ned_lat);
      nh.getParamCached(param_ned_lon, ned_lon);
    } else {
      ROS_WARN("NED Origin NOT FOUND!");
    }
    ned_ = new Ned(ned_lat, ned_lon, 0.0);
  }

  /**
   * @brief      Pushes a IMC::FollowPath message.
   *
   * @param[in]  msg   The message
   */
  void push_back(const IMC::FollowPath& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: IMC::FollowPath starting point ("
      << msg.lat << ", " << msg.lon << ")");

    // Get NED reference
    
    double north, east, depth;
    ned_->geodetic2Ned(msg.lat*180/M_PI, msg.lon*180/M_PI, 0.0,
                       north, east, depth);

    MissionPoint point;
    IMC::MessageList<IMC::PathPoint>::const_iterator it_fp;
    for (it_fp = msg.points.begin();
         it_fp != msg.points.end(); it_fp++) {
      // for each PathPoint in the list offset with respect the starting point
      double n = (*it_fp)->x + north;
      double e = (*it_fp)->y + east;
      double d = (*it_fp)->z + depth; // distance included in the point
      ROS_INFO_STREAM("[turbot_imc_broker]: Adding waypoint at "
        << n << ", " << e << ", " << d << ".");
      point.north = n;
      point.east = e;
      point.z = d + msg.z; // add the reference
      point.is_altitude  = (msg.z_units == 2); // 1 for depth 2 for altitude
      point.speed = msg.speed; // retrieve the follow path message speed.
      ROS_INFO_STREAM("Saving point " << n << ", " << e << ", " << d << "SPEED: " << point.speed);
      points_.push_back(point);
    }
  }

  /**
   * @brief      Pushes a IMC::Goto message.
   *
   * @param[in]  msg   The message
   */
  void push_back(const IMC::Goto& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: IMC::Goto message ("
      << msg.lat << ", " << msg.lon << ")");
    double north, east, depth;
    ned_->geodetic2Ned(msg.lat*180/M_PI, msg.lon*180/M_PI, 0.0,
                       north, east, depth);
    ROS_INFO_STREAM("[turbot_imc_broker]: Adding waypoint at "
      << north << ", " << east << ", " << depth << ".");
    MissionPoint point;
    point.north = north;
    point.east = east;
    point.z = msg.z;
    point.is_altitude  = (msg.z_units == 2);
    point.yaw = msg.yaw;
    point.speed = msg.speed;
    points_.push_back(point);
  }

  /**
   * @brief      Pushes a IMC::StationKeeping message.
   *
   * @param[in]  msg   The message
   */
  void push_back(const IMC::StationKeeping& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: IMC::StationKeeping message: ("
      << msg.lat << ", " << msg.lon << ") " << msg.duration << " s");

    double north, east, depth;
    ned_->geodetic2Ned(msg.lat*180/M_PI, msg.lon*180/M_PI, 0.0,
      north, east, depth);
    ROS_INFO_STREAM("[turbot_imc_broker]: Adding waypoint at "
      << north << ", " << east << ", " << depth << ".");
    MissionPoint point;
    point.north = north;
    point.east = east;
    point.z = msg.z;
    point.yaw = -1;
    point.speed = msg.speed;
    point.duration = msg.duration;
    point.radius = msg.radius;
    points_.push_back(point);
  }

  /**
   * @brief      Pushes a IMC::Loiter message.
   *
   * @param[in]  msg   The message
   */
  void push_back(const IMC::Loiter &msg){
    ROS_INFO_STREAM("[turbot_imc_broker]: IMC::Loiter mission center ("
                    << msg.lat << ", " << msg.lon << ")");

    // Get NED reference

    double north, east, depth;
    ned_->geodetic2Ned(msg.lat * 180 / M_PI, msg.lon * 180 / M_PI, 0.0,
                       north, east, depth);

    double r = msg.radius; 

    // Get WPs
    double wp_dist = 5; //Distance betweeen consecutive WPs, TODO parameterize
    double perimeter = 2 * M_PI * msg.radius;
    int wp_num = (int) perimeter / wp_dist;
    double delta = 2 * M_PI / wp_num;
    double alpha = 0;

    for (size_t i = 0; i < wp_num; i++)
    {
      double n = north + r * sin(alpha);
      double e = east + r * cos(alpha);
      double d = msg.z;
      ROS_INFO_STREAM("[turbot_imc_broker]: Adding waypoint at "
                      << n << ", " << e << ", " << d << ".");
      MissionPoint point;
      point.north = n;
      point.east = e;
      point.z = d;
      point.speed = msg.speed;
      alpha = alpha + delta;
      ROS_INFO_STREAM("Saving point " << n << ", " << e << ", " << d << "SPEED: " << point.speed);
      points_.push_back(point);
    }
  }

  void clear() {
    points_.clear();
    points_idx_ = 0;
    mission_length_ = 0;
  }

  /**
   * @brief      Parses a IMC::PlanSpecification
   * north and east : current vehicle position
   * @param[in]  msg   The message
   */
  void parse(const IMC::PlanSpecification& msg, double& north, double& east) {
    // Delete previous mission
    clear();
    raw_msg_ = msg;

    ROS_INFO_STREAM("Parsing IMC::PlanSpecification...");

    // PlanSpecification has a MessageList of PlanManeuver called maneuvers,
    // and the MessageList defines the a const iterator
    IMC::MessageList<IMC::PlanManeuver>::const_iterator it;
    for (it = msg.maneuvers.begin();
         it != msg.maneuvers.end(); it++) {
      // Each msg.maneuvers[i] is a PlanManeuver
      IMC::Maneuver* maneuver_msg = (*it)->data.get();
      uint16_t maneuver_id = maneuver_msg->getId();
      std::string maneuver_name = maneuver_msg->getName();
      if (maneuver_id == IMC::Goto::getIdStatic()) {
        ROS_WARN_STREAM("Maneuver : " << maneuver_id << " Goto");
        IMC::Goto* goto_msg = IMC::Goto::cast((*it)->data.get());
        push_back(*goto_msg);
      } else if (maneuver_id == IMC::FollowPath::getIdStatic()) {
        ROS_WARN_STREAM("Maneuver : " << maneuver_id << " FollowPath");
        IMC::FollowPath* fp_msg = IMC::FollowPath::cast((*it)->data.get());
        push_back(*fp_msg);
      } else if (maneuver_id == IMC::StationKeeping::getIdStatic()) {
        ROS_WARN_STREAM("Maneuver : " << maneuver_id << " Station Keeping");
        IMC::StationKeeping* sk_msg = IMC::StationKeeping::cast((*it)->data.get());
        push_back(*sk_msg);
      } else if (maneuver_id == IMC::Loiter::getIdStatic()) {
        ROS_WARN_STREAM("Maneuver : " << maneuver_id << " Station Keeping");
        IMC::Loiter *lo_msg = IMC::Loiter::cast((*it)->data.get());
        push_back(*lo_msg);
      } else {
        ROS_WARN_STREAM("Maneuver " << maneuver_name << " (" << maneuver_id
                        << ") not implemented!");
      }
    }
    ROS_INFO_STREAM("Parsing successful!");
    SetStartingPosition(north, east); // set the starting position as the vehicle current pose when the mission is received
    GetTotalLength();
    mean_velocity = 0.0;
    accumulated_velocity = 0.0;
    mean_velocity_counter=1;
    mean_route_time = 0.0;
    accumulated_route_time = 0.0;
  }

  IMC::PlanSpecification GetRaw() {
    return raw_msg_;
  }

  double GetTotalLength() {
    if (mission_length_ == 0) {
      distances_.resize(points_.size());
      for (size_t i = 0; i < points_.size(); i++) {
        double d;
        if (i == 0) {
          d = starting_point_.DistanceTo(points_[0]);
          ROS_INFO_STREAM("[GetTotalLength]: Mission - first goal point " << "(" << points_.at(0).north <<  " , " << points_.at(0).east << ")" );
          ROS_INFO_STREAM("[GetTotalLength]: Mission - starting_point_ " << "(" << starting_point_.north <<  " , " << starting_point_.east << ")" );
        } else {
          d = points_[i-1].DistanceTo(points_[i]);
        }
        distances_[i] = d;
        mission_length_ += d;
        ROS_INFO_STREAM("Distance to " << i << ": " << d);
      }
      ROS_INFO_STREAM("[turbot_imc_broker]: Mission - Plan Total Distance " << mission_length_);
    }
    return mission_length_;
  }

  MissionPoint GetNextPoint() {
    MissionPoint p = points_[points_idx_];
    points_idx_++; // returns the current goal point
    return p; 
  }


  int GetCurrentIdx() const {
    if (points_idx_ - 1 > 0)
      return points_idx_ - 1; // decrement since before the index was incrementes in GetNextPoint
    else
      return 0;
  }


  float GetProgress() {
    if ((duration < 0) || (duration == 0)){ // if GOTO or unlimited Station keeping, progress can reach 100% 
      return GetAcomplishedLength() / GetTotalLength() * 100.0;
    }
    if (duration > 0){ // for station keeping of limited duration add the duration of the sk to the total time of mission execution
      double now = ros::Time::now().toSec();
      double t1 = now - sk_ti; // the time passed between the start of the mission and now. GetAcomplishedLength()/module_velocity;
      if (state_ == MISSION_RUNNING){ // update the t2 variable when vehicle is running towards the points where it has to do the S. Keeping
        accumulated_velocity = accumulated_velocity + module_velocity_;
        mean_velocity = (accumulated_velocity)/mean_velocity_counter; // mean of all velocity samples along the path towards the starting and the goal point
        route_time_ = GetTotalLength()/mean_velocity;
        accumulated_route_time = accumulated_route_time + route_time_;
        mean_route_time = accumulated_route_time/mean_velocity_counter;
        mean_velocity_counter++; //use the velocity mean and the mean of the route time to smooth the progress variations
        // if the instant velocity or the instant route time are used, the global process can exceed the 100%.  
      } // t2 will be fixed during the SK maneuver. Total time = time to get the point + duration of SK
     // ROS_INFO_STREAM("[turbot_imc_broker]: route_time: " << mean_route_time << "  t1: " << t1 << " mean_velocity_counter: " << mean_velocity_counter << " mean_velocity: " << mean_velocity << "module_velocity_" << module_velocity_);
      return ( t1 / (duration + mean_route_time) ) * 100.0;
    }
  }

  double GetAcomplishedLength() {
    double acomplished_length = 0; 
    if (points_.size() == 1){// just a GOTO maneuver
      acomplished_length = current_point_.DistanceTo(starting_point_);
    }else{ // follow path
      for (size_t i = 0; i < GetCurrentIdx(); i++) {
        acomplished_length += distances_[i];
      }
      if (GetCurrentIdx() == 0) {
        acomplished_length += starting_point_.DistanceTo(current_point_);
      } else {
        acomplished_length += points_[GetCurrentIdx() - 1].DistanceTo(current_point_);
      }
    }
    return acomplished_length;
  }

  MissionState GetState() {
    return state_;
  }

  void SetStartingPosition(const double& north, const double& east) {
    starting_point_ = NEPoint(north, east);
    ROS_INFO_STREAM("[turbot_imc_broker]: Mission - Starting Point " << "(" << starting_point_.north <<  " , " << starting_point_.east << ")" );
  }

  void SetCurrentPosition(const double& north, const double& east) {
    current_point_ = NEPoint(north, east);
  }

  size_t size() const {
    return points_.size();
  }

  std::vector<MissionPoint> points_;
  double initial_distance_; // initial distance between the vehicle and the first goal point at the start of the mission
  double mission_length_;
  double module_velocity_;
  double route_time_;
  double duration; // it has sense only for station keeping maneuvers 
  double sk_ti; // mission start time
  int points_idx_;
  double mean_velocity =0.0;
  double accumulated_velocity; 
  int mean_velocity_counter;
  double mean_route_time;
  double accumulated_route_time;
  NEPoint starting_point_;
  NEPoint current_point_;
  std::vector<double> distances_; // distances between points of the path
  MissionState state_;

 private:
  IMC::PlanSpecification raw_msg_;
  Ned* ned_;


};

#endif // INCLUDE_TURBOT_IMC_BROKER_MISSION_H_
