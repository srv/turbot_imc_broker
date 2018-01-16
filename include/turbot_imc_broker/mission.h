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

#ifndef MISSION_H
#define MISSION_H

#include <ros/ros.h>

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

class MissionPoint {
 public:
  MissionPoint() : north(0), east(0), z(0), speed(0), duration(0),
                   radius(0), is_altitude(false) {
    // empty
  }
  double north;
  double east;
  double z; //TODO: he canviat depth per z ja que al poder triar si es depth o altitude amb el bolea de sota crec que és més quarent
  double yaw;
  double speed;
  double duration;
  double radius;
  bool is_altitude;  //! if true, depth is altitude
};

class Mission {
 public:
  Mission() {
    ros::NodeHandle nh("~");

    std::string param_ned_lat;
    std::string param_ned_lon;
    nh.param<std::string>("param_ned_lat", param_ned_lat, std::string("/navigator/ned_origin_lat"));
    nh.param<std::string>("param_ned_lon", param_ned_lon, std::string("/navigator/ned_origin_lon"));

    // Get NED origin
    double ned_lat = 0.0, ned_lon = 0.0;
    if (nh.hasParam(param_ned_lat) && nh.hasParam(param_ned_lon)) {
      nh.getParamCached(param_ned_lat, ned_lat);
      nh.getParamCached(param_ned_lon, ned_lon);
    } else {
      ROS_WARN("NED Origin NOT FOUND!");
    }
    ned_ = new Ned(ned_lat, ned_lon, 0.0);
  }

  void push_back(const IMC::FollowPath& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: Store Follow Path starting point (" 
      << msg.lat << ", " << msg.lon << ")"); // lat and lon of stating point
    double north, east, depth;
    MissionPoint point;
    // NED starting position 
    ned_->geodetic2Ned(msg.lat*180/M_PI, msg.lon*180/M_PI, 0.0, north, east, depth);
    // msg contains a list of Path Points. Extract each point and push it into
    //the vector 'points'
    // iterator for the message list of PathPoints
  //  std::cout << "[turbot_imc_broker]: Follow Path. Points " << *msg.points.end() ;
    IMC::MessageList<IMC::PathPoint>::const_iterator it_fp;
    for (it_fp = msg.points.begin();
         it_fp != msg.points.end(); it_fp++) { // for each PathPoint in the list 
            float n = (*it_fp)->x + north;
            float e = (*it_fp)->y + east;
            float d = (*it_fp)->z + depth; // ofsets with respect the starting point
            ROS_INFO_STREAM("[turbot_imc_broker]: Follow Path. Add path waypoint at " << n << ", " << e << ", " << d << ".");
            point.north = n;
            point.east = e;
            point.z = d;
            point.duration = -1; // just in order to distinguish between goto and station keeping 
            points_.push_back(point);
    }
  }

  void push_back(const IMC::Goto& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: Goto ("
                    << msg.lat << ", " << msg.lon << ")");
    double north, east, depth;
    ned_->geodetic2Ned(msg.lat*180/M_PI, msg.lon*180/M_PI, 0.0, north, east, depth);
    ROS_INFO_STREAM("[turbot_imc_broker]: NED (" << north << ", " << east << ")");

    MissionPoint point;
    point.north = north;
    point.east = east;
    point.z = msg.z;
    point.yaw = msg.yaw;
    point.duration = -1; // just in order to distinguish between goto and station keeping 
    point.speed = msg.speed;
    points_.push_back(point);
  }

  void push_back(const IMC::StationKeeping& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: StationKeeping ("
                    << msg.lat << ", " << msg.lon << ")");

    double north, east, depth;
    ned_->geodetic2Ned(msg.lat*180/M_PI, msg.lon*180/M_PI, 0.0, north, east, depth);

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

  void parse(const IMC::PlanSpecification& msg) {

    // Delete previous mission
    points_.clear();

    raw_msg_ = msg;

    // define it as a variable type const_iterator defined in the MessaList class,
    IMC::MessageList<IMC::PlanManeuver>::const_iterator it;
    // the PlanSpecification has a MessageList of PlanManeuver called maneuvers,
    // and the MessageList defines the cons_iterator
    for (it = msg.maneuvers.begin();
         it != msg.maneuvers.end(); it++) {
      // Each msg.maneuvers[i] is a PlanManeuver
      // the data part of the PlanManeuver is a Maneuver which inherits from InlineMessage,
      IMC::Maneuver* maneuver_msg = (*it)->data.get();
      uint16_t maneuver_id = maneuver_msg->getId();
      std::string maneuver_name = maneuver_msg->getName();
      //which has the get() method that returns the pointer to the message (maneuver).
      // message.hpp has a method called getName which returns the maneuver type
      if (maneuver_id == IMC::Goto::getIdStatic()) {
        IMC::Goto* goto_msg = IMC::Goto::cast((*it)->data.get());
        push_back(*goto_msg);         // store the Goto msg in the vector of points
      } else if (maneuver_id == IMC::FollowPath::getIdStatic()) {
        IMC::FollowPath* fp_msg = IMC::FollowPath::cast((*it)->data.get());
        push_back(*fp_msg);
      } else if (maneuver_id == IMC::StationKeeping::getIdStatic()) {
        IMC::StationKeeping* sk_msg = IMC::StationKeeping::cast((*it)->data.get());
        // store the StationKeeping msg in the vector of points
        push_back(*sk_msg);
      } else {
        ROS_WARN_STREAM("Maneuver " << maneuver_name << " (" << maneuver_id << ") not implemented!");
      }
    }
  }

  IMC::PlanSpecification getRaw() {
    return raw_msg_;
  }

  void run() {

  }

  size_t size() const {
    return points_.size();
  }

  std::vector<MissionPoint> points_;
 private:
  IMC::PlanSpecification raw_msg_;
  Ned* ned_;
};

#endif // MISSION_H
