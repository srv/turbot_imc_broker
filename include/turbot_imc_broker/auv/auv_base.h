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
  AuvBase() : nh_("~") {}

  void Init(const int& auv_id, const int& entity_id) {
    // Plan control state default values
    plan_control_state_.setSource(auv_id);
    plan_control_state_.setSourceEntity(entity_id);
    plan_control_state_.plan_eta = 0;
    plan_control_state_.plan_progress = 0;
    plan_control_state_.man_id = "";
    // plan_control_state_.man_id ??;
    plan_control_state_.man_eta = -1;

    // Vehicle state default values
    vehicle_state_.setSource(auv_id);
    vehicle_state_.setSourceEntity(entity_id);
    // the vehicle is in ready to service request
    vehicle_state_.op_mode = IMC::VehicleState::VS_SERVICE;
    vehicle_state_.maneuver_eta = 65535; // value when no maneuver
    // still to define how to capture an error
    vehicle_state_.error_count = 0;
    vehicle_state_.error_ents = "no error";
    vehicle_state_.maneuver_type = 0;
    vehicle_state_.maneuver_stime = 0;
    vehicle_state_.control_loops = 0x00000000;  // no use
    vehicle_state_.flags = 0x00;  //0x01 when the maneuver is done, 0 elsewhere
    vehicle_state_.last_error = "no error";
    vehicle_state_.last_error_time = 0;
  }

  IMC::PlanControlState GetPlanControlState() {
    plan_control_state_.setTimeStamp(ros::Time::now().toSec());
    return plan_control_state_;
  }

  IMC::VehicleState GetVehicleState() {
    vehicle_state_.setTimeStamp(ros::Time::now().toSec());
    return vehicle_state_;
  }
  virtual bool Abort() = 0;
  virtual bool Goto(const MissionPoint& p) = 0;

 protected:
  ros::NodeHandle nh_;

  // DO NOT ADD IMC PUBLISHERS HERE
  ros::Subscriber nav_sts_sub_;
  ros::Subscriber plan_status_sub_;

  IMC::PlanControlState plan_control_state_;
  IMC::VehicleState vehicle_state_;
};

#endif // AUV_H


