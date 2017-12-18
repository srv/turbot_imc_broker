#ifndef SPARUS_AUV_H
#define SPARUS_AUV_H

#include <turbot_imc_broker/auv/auv_base.h>
#include <cola2_msgs/CaptainStatus.h>
#include <cola2_msgs/RecoveryAction.h>
#include <cola2_msgs/Goto.h>

class SparusAUV : public AuvBase {

public:
    SparusAUV() : AuvBase(), is_plan_loaded_(false) {
      plan_status_sub_ = nh_.subscribe("/cola2_control/captain_status", 1,
                                       &SparusAUV::CaptainStatusCallback, this);


      ROS_INFO_STREAM_THROTTLE(1, "[AUV:] Waiting for safety service...");
      recovery_actions_ = nh_.serviceClient<cola2_msgs::RecoveryAction>("/cola2_safety/recovery_action");
      bool is_available = recovery_actions_.waitForExistence(ros::Duration(10));
      if (!is_available) {
        ROS_ERROR_STREAM("[ turbot_imc_broker ]: RecoveryAction service is not available.");
      }

      ROS_INFO_STREAM_THROTTLE(1, "[AUV:] Waiting for goto service...");
      goto_srv_ = nh_.serviceClient<cola2_msgs::Goto>("/cola2_control/enable_goto");
      is_available = goto_srv_.waitForExistence(ros::Duration(10));
      if (!is_available) {
        ROS_ERROR_STREAM("[ turbot_imc_broker ]: Goto service is not available.");
      }
    }

    /**
     * @brief      Launch the emergency surface service
     *
     * @return     true if successful to launch
     */
    bool Abort() {
      cola2_msgs::RecoveryAction srv;
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
    bool Goto(const MissionPoint &p) {
      cola2_msgs::Goto srv;
      srv.request.position.x = p.north;
      srv.request.position.y = p.east;
      srv.request.position.z = p.z;
      srv.request.altitude = p.z;
      srv.request.altitude_mode = p.is_altitude;
      srv.request.disable_axis.x = false;
      srv.request.disable_axis.y = true;
      srv.request.disable_axis.z = false;
      srv.request.disable_axis.roll = true;
      srv.request.disable_axis.pitch = true;
      srv.request.disable_axis.yaw = false;
      srv.request.position_tolerance.x = 2.0;
      srv.request.position_tolerance.y = 2.0;
      srv.request.position_tolerance.z = 1.0;
      srv.request.orientation_tolerance.roll = 0.3;
      srv.request.orientation_tolerance.pitch = 0.3;
      srv.request.orientation_tolerance.yaw = 0.3;
      srv.request.linear_velocity.x = p.speed;
      srv.request.blocking = false;
      srv.request.keep_position = false;
      srv.request.reference = cola2_msgs::GotoRequest::REFERENCE_GLOBAL;
      srv.request.priority = 10;
      if (p.duration > 0) {
        srv.request.keep_position = true;
        srv.request.timeout = p.duration;
      }
      goto_srv_.call(srv);
    }

private:
    void CaptainStatusCallback(const cola2_msgs::CaptainStatus& msg) {
      // Default values
      plan_control_state_.plan_eta = 0;
      plan_control_state_.plan_progress = 0;
      plan_control_state_.man_id = "";
      // plan_control_state.man_id ??;
      plan_control_state_.man_eta = -1;

      if (msg.mission_active) { // A plan is under execution
        plan_control_state_.state = IMC::PlanControlState::PCS_EXECUTING;
        plan_control_state_.plan_eta = msg.total_steps * TIME_PER_MISSION_STEP;
        plan_control_state_.plan_progress = int((float(msg.current_step)/float(msg.total_steps))*100);
        plan_control_state_.man_id = std::to_string(msg.current_step);
        // plan_control_state.man_id ??;
        plan_control_state_.man_eta = -1;
      }
      else { // No plan under execution ...
        if (msg.active_controller == 0) { // ... and no actions are being executed ...
          if (is_plan_loaded_) { // ... and a plan is loaded.
            plan_control_state_.state = IMC::PlanControlState::PCS_READY;
          }
          else { // ... and no plan is loaded.
            plan_control_state_.state = IMC::PlanControlState::PCS_BLOCKED;
          }
        }
        else { // ... but an action (goto, keep position, ...) is under execution
          plan_control_state_.state = IMC::PlanControlState::PCS_BLOCKED;
        }
      }
      if (is_plan_loaded_) {
        plan_control_state_.plan_id = "last_plan";
      }
      else {
        plan_control_state_.plan_id = "";
      }
      plan_control_state_.last_outcome = plan_control_state_.state;
    }

    bool is_plan_loaded_;
    ros::ServiceClient recovery_actions_;
    ros::ServiceClient goto_srv_;
};

#endif // SPARUS_AUV_H
