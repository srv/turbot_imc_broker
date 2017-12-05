#ifndef AUV_H
#define AUV_H

#include <ros/ros.h>

#ifdef UDG
//#include Recovery Actions de UdG
#endif
#ifdef UIB
#include <safety/RecoveryAction.h>
#include <safety/MissionStatus.h>
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
#include <IMC/Spec/PlanControlState.hpp>
#define TIME_PER_MISSION_STEP   100


class Auv { // parent class 
public:
  Auv() {}
  virtual IMC::PlanControlState GetPlanControlState(const safety::MissionStatus& msg, int auv_id, int entity_id) = 0;
  virtual bool Abort() =0;
 // virtual bool Goto(const MissionPoint& p);

};

//! turbot_auv.h

//#include <turbot_imc_broker/auv.h>

class TurbotAUV : public Auv {
  
  public: 
  
    bool is_plan_loaded_=true;
    int m_eta=0;
    TurbotAUV(void): Auv() { // constructor
    ros::NodeHandle nh_("~");
    ROS_INFO_STREAM_THROTTLE(1, "[AUV:] Waiting for safety service...");
    recovery_actions_ = nh_.serviceClient<safety::RecoveryAction>("/safety/recovery_action");
    bool is_available = recovery_actions_.waitForExistence(ros::Duration(10));
    if (!is_available) {
        ROS_ERROR_STREAM("[ turbot_imc_broker ]: RecoveryAction service is not available.");
      }
  }
  
  
  IMC::PlanControlState GetPlanControlState(const safety::MissionStatus& msg,  int auv_id, int entity_id) {
      ros::NodeHandle nh_("~");
      plan_control_state_pub_ = nh_.advertise<IMC::PlanControlState>("/IMC/Out/PlanControlState", 100);
      IMC::PlanControlState plan_control_state;
      plan_control_state.setSource(auv_id);
      plan_control_state.setSourceEntity(entity_id);
      plan_control_state.setTimeStamp(ros::Time::now().toSec());

      // Default values
      plan_control_state.plan_eta = 0;
      plan_control_state.plan_progress = 0;
      plan_control_state.man_id = "";
      // plan_control_state.man_id ??;
      plan_control_state.man_eta = -1;

      if (msg.current_wp > 0) { // A plan is under execution
        plan_control_state.state = IMC::PlanControlState::PCS_EXECUTING;
        plan_control_state.plan_eta = msg.total_wp * TIME_PER_MISSION_STEP;
        plan_control_state.plan_progress = (float(msg.current_wp)/float(msg.total_wp))*100;
        plan_control_state.man_id = std::to_string(msg.current_wp);
        // plan_control_state.man_id ??;
        plan_control_state.man_eta = -1;
      }
      else { // No plan under execution ...
        if (is_plan_loaded_) { // ... and a plan is loaded.
          plan_control_state.state = IMC::PlanControlState::PCS_READY;
        }
        else { // ... and no plan is loaded.
          plan_control_state.state = IMC::PlanControlState::PCS_BLOCKED;
        }
      }

      if (is_plan_loaded_) {
        plan_control_state.plan_id = "last_plan SRV Group Turbot ";
      }
      else {
        plan_control_state.plan_id = "";
      }
      m_eta= plan_control_state.plan_eta;
      plan_control_state.last_outcome = plan_control_state.state;
      plan_control_state_pub_.publish(plan_control_state);


  }
  
  bool Abort() {
    //launch the emergency surface service ...
      safety::RecoveryAction srv;
      srv.request.error_level = srv.request.EMERGENCY_SURFACE;
      recovery_actions_.call(srv);
  }


  // bool Goto(const MissionPoint& p) {

  // }

  private:
  ros::ServiceClient recovery_actions_;
  ros::Publisher plan_control_state_pub_;

};

//! sparus_auv.h class sparus with the code of UdG 
// class SparusAUV : public AUV {
//   IMC::EstimatedState GetEstimatedState(){

//   }
//   IMC::PlanControlState GetPlanControlState(){

//   }
//   bool Abort(){

//   }
//   bool Goto(const MissionPoint& p){

//   }
// };

#endif // AUV_H


