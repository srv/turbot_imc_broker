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
                   radius(0), altitude(false) {
    // empty
  }
  double north;
  double east;
  double z; //TODO: he canviat depth per z ja que al poder triar si es depth o altitude amb el bolea de sota crec que és més quarent
  double yaw;
  double speed;
  double duration;
  double radius;
  bool altitude;  //! if true, depth is altitude
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

  }

  void push_back(const IMC::Goto& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: Goto ("
                    << msg.lat << ", " << msg.lon << ")");
    double north, east, depth;
    ned_->geodetic2Ned(msg.lat, msg.lon, 0.0, north, east, depth);

    MissionPoint point;
    point.north = north;
    point.east = east;
    point.z = msg.z;
    point.yaw = msg.yaw;
    point.speed = msg.speed;
    points.push_back(point);
  }

  void push_back(const IMC::StationKeeping& msg) {
    ROS_INFO_STREAM("[turbot_imc_broker]: StationKeeping ("
                    << msg.lat << ", " << msg.lon << ")");

    double north, east, depth;
    ned_->geodetic2Ned(msg.lat, msg.lon, 0.0, north, east, depth);

    MissionPoint point;
    point.north = north;
    point.east = east;
    point.z = msg.z;
    point.yaw = -1;
    point.speed = msg.speed;
    point.duration = msg.duration;
    point.radius = msg.radius;
    points.push_back(point);
  }

  void parse(const IMC::PlanSpecification& msg) {
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
      //which has the get() method that returns the pointer to the message (maneuver).
      // message.hpp has a method called getName which returns the maneuver type
      if (maneuver_msg->getName() == "Goto") {
        IMC::Goto* goto_msg = IMC::Goto::cast((*it)->data.get());
        push_back(*goto_msg);
      } else if (maneuver_msg->getName() == "FollowPath") {
        IMC::FollowPath* fp_msg = IMC::FollowPath::cast((*it)->data.get());
        // store the Goto msg in the vector of points
        push_back(*fp_msg);
      } else if (maneuver_msg->getName() == "StationKeeping") {
        IMC::StationKeeping* sk_msg = IMC::StationKeeping::cast((*it)->data.get());
        // store the StationKeeping msg in the vector of points
        push_back(*sk_msg);
      }
    }
  }

  IMC::PlanSpecification getRaw() {
    return raw_msg_;
  }

  void run() {

  }

  size_t size() const {
    return points.size();
  }

  std::vector<MissionPoint> points;
 private:
  IMC::PlanSpecification raw_msg_;
  Ned* ned_;
};

#endif // MISSION_H
