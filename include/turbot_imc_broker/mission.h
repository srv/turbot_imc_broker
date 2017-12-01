#ifndef MISSION_H
#define MISSION_H

#include <ros/ros.h>

#ifdef UDG
#include <cola2_navigation/ned.h>
#endif
#ifdef UIB
#include <utils/ned.h>
#endif

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
  MissionPoint() : north(0), east(0), depth(0), speed(0), duration(0),
                   radius(0), altitude(false) {
    // empty
  }
  double north;
  double east;
  double depth;
  double speed;
  double duration;
  double radius;
  bool altitude;  //! if true, depth is altitude
};

class Mission {
 public:
  Mission(const double& ned_lat, const double& ned_lon) {
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
    point.depth = msg.z;
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
    point.depth = msg.z;
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

  std::vector<MissionPoint> points;
 private:
  IMC::PlanSpecification raw_msg_;
  Ned* ned_;
};

#endif // MISSION_H
