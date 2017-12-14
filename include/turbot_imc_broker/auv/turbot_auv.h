#ifndef TURBOT_AUV_H
#define TURBOT_AUV_H

//! turbot_auv.h

#include <turbot_imc_broker/auv/auv_base.h>

#include <safety/RecoveryAction.h>
#include <safety/MissionStatus.h>

class TurbotAUV : public AuvBase {
 public:
  TurbotAUV() : AuvBase(), is_plan_loaded_(false) {
    plan_status_sub_ = nh_.subscribe("/control/mission_status", 1 ,
                                    &TurbotAUV::MissionStatusCallback, this);

    ROS_INFO_STREAM_THROTTLE(1, "[AUV:] Waiting for safety service...");
    recovery_actions_ = nh_.serviceClient<safety::RecoveryAction>("/safety/recovery_action");
    bool is_available = recovery_actions_.waitForExistence(ros::Duration(10));
    if (!is_available) {
      ROS_ERROR_STREAM("[ turbot_imc_broker ]: RecoveryAction service is not available.");
    }
  }

  /**
   * @brief      Launch the emergency surface service
   *
   * @return     true if successful to launch
   */
  bool Abort() {
    safety::RecoveryAction srv;
    srv.request.error_level = srv.request.EMERGENCY_SURFACE;
    recovery_actions_.call(srv);
  }


  /**
   * @brief      Go to a requested mission point
   *
   * @param[in]  p     The mission point
   *
   * @return     true if successful
   */
  bool Goto(const MissionPoint& p) {
    // TODO
  }

 private:
  void MissionStatusCallback(const safety::MissionStatus& msg) {
    if (msg.current_wp > 0) { // A plan is under execution
      plan_control_state_.state = IMC::PlanControlState::PCS_EXECUTING;
      plan_control_state_.plan_eta = msg.total_wp * TIME_PER_MISSION_STEP;
      plan_control_state_.plan_progress = (float(msg.current_wp)/float(msg.total_wp))*100;
      plan_control_state_.man_id = std::to_string(msg.current_wp);
      // plan_control_state_.man_id ??;
      plan_control_state_.man_eta = -1;
    } else { // No plan under execution ...
      if (is_plan_loaded_) { // ... and a plan is loaded.
        plan_control_state_.state = IMC::PlanControlState::PCS_READY;
      } else { // ... and no plan is loaded.
        plan_control_state_.state = IMC::PlanControlState::PCS_BLOCKED;
      }
    }

    if (is_plan_loaded_) {
      plan_control_state_.plan_id = "last_plan SRV Group Turbot ";
    } else {
      plan_control_state_.plan_id = "";
    }
    plan_control_state_.last_outcome = plan_control_state_.state;
  }

  bool is_plan_loaded_;
  ros::ServiceClient recovery_actions_;
};


#endif // TURBOT_AUV_H