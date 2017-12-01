#ifndef MISSION_H
#define MISSION_H
//#include <utils/ned.h>



class MissionPoint {
 public:
  double x;
  double y;
  double z;
  double speed;
  double duration;
  double radius;
  bool z_unit;
};

class Mission {
 public:
  std::vector<MissionPoint> points;

  void push_back(const IMC::FollowPath& msg) {

  }

  void push_back(const IMC::Goto& msg) {

        ROS_INFO_STREAM("[ turbot_imc_broker: mission data Goto]: " << msg.lat);
        double ned_origin_lat=0, ned_origin_lon=0;
        // if (!getNedOrigin(ned_origin_lat, ned_origin_lon))
        //   {
        //     ROS_ERROR_STREAM("[" << node_name_ << "]: Impossible to get the ned origin from the parameter server.");
        //     return false;
        //   }
        double north, east, depth;
       // ned_ = new Ned(ned_origin_lat, ned_origin_lon, 0.0);
       // ned_->geodetic2Ned(msg.lat, msg.lon, 0.0, north, east, depth);

        MissionPoint point; 
        point.x= msg.lat; // transform lat/long into NED coordinates. 
        point.y= msg.lon;
        point.z= msg.z;
        point.speed=msg.speed;
        point.z_unit=1; //depth
        points.push_back(point);

  }

  void push_back(const IMC::StationKeeping& msg) {

        ROS_INFO_STREAM("[ turbot_imc_broker: mission data StationKeeping]: " << msg.lat);
        MissionPoint point; 
        point.x=msg.lat; // transform lat/long into NED coordinates. 
        point.y=msg.lon;
        point.z=msg.z;
        point.speed=msg.speed;
        point.duration=msg.duration;
        point.radius=msg.radius;
        point.z_unit=1;//depth
        points.push_back(point);

  }

  void parse(const IMC::PlanSpecification& msg) {
    raw_msg_ = msg;


    IMC::MessageList<IMC::PlanManeuver>::const_iterator it; // define it as a variable type const_iterator defined in the MessaList class, 
    for (it = msg.maneuvers.begin(); // the PlanSpecification has a MessageList of PlanManeuver called maneuvers, and the MessageList defines the cons_iterator
         it != msg.maneuvers.end(); it++) {
      // Each msg.maneuvers[i] is a PlanManeuver
      IMC::Maneuver* maneuver_msg = (*it)->data.get(); // the data part of the PlanManeuver is a Maneuver which inherits from InlineMessage, 
    //which has the get() method that returns the pointer to the message (maneuver).  
      if (maneuver_msg->getName() == "Goto") {  // message.hpp has a method called getName which returns the maneuver type
        IMC::Goto* goto_msg = IMC::Goto::cast((*it)->data.get());
        push_back(*goto_msg); // store the Goto msg in the vector of points
        // Handle Goto maneuver

      } else if (maneuver_msg->getName() == "FollowPath") {
        IMC::FollowPath* fp_msg = IMC::FollowPath::cast((*it)->data.get());
        // Handle FollowPath maneuver
      } else if (maneuver_msg->getName() == "StationKeeping") {
        IMC::StationKeeping* sk_msg = IMC::StationKeeping::cast((*it)->data.get());
        push_back(*sk_msg); // store the StationKeeping msg in the vector of points

        // Handle StationKeeping maneuver
      }
    }
  }

  IMC::PlanSpecification getRaw() {
    return raw_msg_;
  }

  void run() {

  }

 private:
  IMC::PlanSpecification raw_msg_;
};

#endif // MISSION_H
