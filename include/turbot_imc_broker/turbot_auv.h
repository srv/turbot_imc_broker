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

#ifndef TURBOT_AUV_H
#define TURBOT_AUV_H

#include <turbot_imc_broker/auv_base.h>
#include <std_srvs/Empty.h> // include the empty services for the enable keep position
#include <control/Goto.h>
#include <safety/RecoveryAction.h>
#include <safety/MissionStatus.h>


class TurbotAUV : public AuvBase {
 public:
  TurbotAUV() : AuvBase(), is_plan_loaded_(false) {
    plan_status_sub_ = nh.subscribe("/control/mission_status", 1 ,
                                    &TurbotAUV::MissionStatusCallback, this);
    bool oneshot;
    ROS_INFO_STREAM_THROTTLE(1, "[AUV:] Waiting for safety service...");
    recovery_actions_ = nh.serviceClient<safety::RecoveryAction>("/safety/recovery_action");
    client_goto_ = nh.serviceClient<control::Goto>("/control/goto_local_block");
    client_enable_keep_position_ = nh.serviceClient<std_srvs::Empty>("/control/enable_keep_position"); 
    client_disable_keep_position_ = nh.serviceClient<std_srvs::Empty>("/control/disable_keep_position"); 
    // create one shot timer, it fires only ones until is rescheduled again with a start()
    timer_keep_pos_ = nh.createTimer(ros::Duration(t_station_keeping), &TurbotAUV::Timer_keep_pos, this, oneshot = true);
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
    safety::RecoveryAction srv_emergency;
    srv_emergency.request.error_level = srv_emergency.request.EMERGENCY_SURFACE;
    recovery_actions_.call(srv_emergency);
  }

  bool StopMission() {
    safety::RecoveryAction srv_stopmission;
    srv_stopmission.request.error_level = srv_stopmission.request.ABORT_MISSION;
    recovery_actions_.call(srv_stopmission);
    // TODO
  }

// timer interruption service routine for keep position disabling: fires after 
  // the programmed duration 
void Timer_keep_pos(const ros::TimerEvent&) {
  srv_running_ = false; // indicates that the station keeping is inactive.
  std_srvs::Empty disable_keep_position; 
  client_disable_keep_position_.call(disable_keep_position); // dissable the station keeping
}

  /**
   * @brief      Go to a requested mission point
   *
   * @param[in]  p     The mission point
   * convert the data in the MissionPoint ( double north; double east; double z; double speed; double duration; double radius;
    bool altitude; ) into a goal point for the vehicle
   * @return     true if successful
   */
  bool Goto(const MissionPoint& p) { // implement different versions for Udg and UIB
    // distinguish between goto and station keeping
    if (p.duration > (-1)) // for station keeping , first goto to the desired location and then call keep position service.
      // ALERT !! duration in seconds = 0 for unlimited
    {

      control::Goto srv;
      srv.request.north_lat = p.north;
      srv.request.east_lon = p.east;
      srv.request.z = p.z;
      //srv.request.yaw = yaw; //take the yaw fro the Goto Maneuver
      srv.request.tolerance = params.goto_tolerance;
      if (srv_running_== false) { 
          if (client_goto_.call(srv)) {  // bloking go to --> waits until the threat is terminated. the return 
            // indicates the result of the Go to: goal reached = true, a problem = false. 
            ROS_INFO_STREAM("Go to service called and motion to the goal point terminated !");
            srv_running_ = true; // indicates that the station keeping is active.
            std_srvs::Empty keep_position;
            client_enable_keep_position_.call(keep_position); // enable keep position. 
            if (p.duration > 0) // keep position of limited duration
              {
              t_station_keeping=p.duration; //set the duration of the station keeping time
              timer_keep_pos_.start(); // start oneshot couter for keep position duration. 
              }
            //if duration = 0 --> unlimited time until a stop mission is requested  
            return true;
          }else{ // the go to service has had a problem
            ROS_ERROR_STREAM("Failed to call service Go to " ); //
            return false;
          }

        // TODO
        } 
        /* control if the vehicle is in the desired position is missing. Do the station keeping, only if the desired goal has been reached */
      
    }else{ // for simply goto. Do the goto only if the desired goal has been reached 

      control::Goto srv;
      srv.request.north_lat = p.north;
      srv.request.east_lon = p.east;
      srv.request.z = p.z;
      srv.request.tolerance = params.goto_tolerance;
      //srv.request.yaw = yaw; //take the yaw fro the Goto Maneuver
      //srv.request.tolerance = go_to_tolerance_;
      if (srv_running_== false) {
          if (!client_goto_.call(srv)) { // bloking go to --> waits until the threat is terminated. the return 
            // indicates the result of the Go to: goal reached = true, a problem = false. 
            ROS_ERROR_STREAM("Failed to call service Go to with Yaw " ); //
            return false;
          }
        ROS_INFO_STREAM("Go to service called!");
        return true;
        // TODO
        }
    }
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
  ros::ServiceClient client_goto_;
  ros::ServiceClient client_enable_keep_position_;
  ros::ServiceClient client_disable_keep_position_;

  bool srv_running_= false;
};


#endif // TURBOT_AUV_H