#ifndef AUV_H
#define AUV_H

#include <ros/ros.h>

// Base IMC template
#include <ros_imc_broker/ImcTypes.hpp>
#include <IMC/Base/Packet.hpp>
#include <IMC/Spec/AllMessages.hpp>
#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/Goto.hpp>
#include <IMC/Spec/StationKeeping.hpp>
#include <IMC/Spec/FollowPath.hpp>
#include <IMC/Spec/PlanControlState.hpp>
#define TIME_PER_MISSION_STEP   100


class AuvBase { // parent class
public:
  AuvBase() {}
  virtual IMC::PlanControlState GetPlanControlState() = 0;
  virtual bool Abort() =0;
 // virtual bool Goto(const MissionPoint& p);

};



#endif // AUV_H


