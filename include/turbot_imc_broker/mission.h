#ifndef MISSION_H
#define MISSION_H

class MissionPoint {
 public:
  double x;
  double y;
  double z;
  double speed;
  double duration;
  double radius;
  bool z_unit;
}

class Mission {
 public:
  std::vector<MissionPoint> points;

  void push_back(const IMC::FollowPath& msg) {

  }

  void push_back(const IMC::Goto& msg) {

  }

  void push_back(const IMC::StationKeeping& msg) {

  }

  void parse(const PlanSpecification& msg) {
    raw_msg_ = msg;

    IMC::MessageList<IMC::PlanManeuver>::const_iterator it;
    for (it = msg.maneuvers.begin();
         it != msg.maneuvers.end(); it++) {
      // Each msg.maneuvers[i] is a PlanManeuver
      IMC::Maneuver* maneuver_msg = (*it)->data.get();
      if (maneuver_msg->getName() == "Goto") {
        IMC::Goto* goto_msg = IMC::Goto::cast((*it)->data.get());
        // Handle Goto maneuver
      } else if (maneuver_msg->getName() == "FollowPath") {
        IMC::FollowPath* fp_msg = IMC::FollowPath::cast((*it)->data.get());
        // Handle FollowPath maneuver
      } else if (maneuver_msg->getName() == "StationKeeping") {
        IMC::StationKeeping* sk_msg = IMC::StationKeeping::cast((*it)->data.get());
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
}

#endif // MISSION_H
