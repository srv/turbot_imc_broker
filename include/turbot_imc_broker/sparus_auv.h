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

#ifndef SPARUS_AUV_H
#define SPARUS_AUV_H

#include <turbot_imc_broker/auv_base.h>
#include <cola2_msgs/CaptainStatus.h>
#include <cola2_msgs/RecoveryAction.h>
#include <cola2_msgs/Goto.h>
#include <boost/thread.hpp>
#include <std_srvs/Empty.h>

class SparusAUV : public AuvBase
{
public:
  SparusAUV() : AuvBase(), is_mission_aborted_(false), current_step_(0)
  {
    plan_status_sub_ = nh.subscribe("/cola2_control/captain_status", 1, &SparusAUV::CaptainStatusCallback, this);

    ROS_INFO_STREAM_THROTTLE(1, "[ turbot_imc_broker ] Waiting for safety service...");
    recovery_actions_ = nh.serviceClient<cola2_msgs::RecoveryAction>("/cola2_safety/recovery_action");
    bool is_available = recovery_actions_.waitForExistence(ros::Duration(10));
    if (!is_available)
    {
      ROS_ERROR_STREAM("[ turbot_imc_broker ]: RecoveryAction service is not available.");
    }

    ROS_INFO_STREAM_THROTTLE(1, "[ turbot_imc_broker ] Waiting for goto service...");
    goto_srv_ = nh.serviceClient<cola2_msgs::Goto>("/cola2_control/enable_goto");
    is_available = goto_srv_.waitForExistence(ros::Duration(10));
    if (!is_available)
    {
      ROS_ERROR_STREAM("[ turbot_imc_broker ]: Goto service is not available.");
    }

    ROS_INFO_STREAM_THROTTLE(1, "[ turbot_imc_broker ] Waiting for captain services...");
    enable_external_mission_ = nh.serviceClient<std_srvs::Empty>("/captain/enable_external_mission");
    is_available = enable_external_mission_.waitForExistence(ros::Duration(10));
    if (!is_available)
    {
      ROS_ERROR_STREAM("[ turbot_imc_broker ]: Captain services are not available.");
    }
    disable_external_mission_ = nh.serviceClient<std_srvs::Empty>("/captain/disable_external_mission");
    is_available = disable_external_mission_.waitForExistence(ros::Duration(10));
    if (!is_available)
    {
      ROS_ERROR_STREAM("[ turbot_imc_broker ]: Captain services are not available.");
    }
  }

  /**
   * @brief      Launch the emergency surface service
   *
   * @return     true if successful to launch
   */
  bool Abort()
  {
    cola2_msgs::RecoveryAction srv;
    srv.request.error_level = srv.request.EMERGENCY_SURFACE;
    recovery_actions_.call(srv);
  }

  /**
   * @brief      Stop the curent mission if any
   *
   * @return     true if successful to launch
   */
  bool StopMission()
  {
    cola2_msgs::RecoveryAction srv;
    srv.request.error_level = srv.request.ABORT_MISSION;
    recovery_actions_.call(srv);
    is_mission_aborted_ = true;
  }

  /**
   * @brief      Go to a requested mission point
   *
   * @param[in]  p     The mission point
   *
   * @return     true if successful
   */
  bool Goto(const MissionPoint& p)
  {
    cola2_msgs::Goto srv;
    srv.request.reference = cola2_msgs::GotoRequest::REFERENCE_NED;
    srv.request.priority = 10;
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
    srv.request.blocking = true;
    srv.request.keep_position = false;
    if (p.duration > 0)
    {
      srv.request.keep_position = true;
      srv.request.timeout = p.duration;
    }
    goto_srv_.call(srv);
  }

  /**
   * @brief      Play current mission if a mission is defined
   *
   * @return     true if successful
  */
  bool PlayMission()
  {
    if (mission.size() == 0)
    {
      ROS_INFO_STREAM("[ turbot_imc_broker ] Mission is not loaded!");
      return false;
    }
    boost::thread* t;
    t = new boost::thread(&SparusAUV::PlayMissionThread, this);
  }

private:
  void PlayMissionThread()
  {
    std_srvs::Empty srv;
    enable_external_mission_.call(srv);
    for (size_t i = 0; i < mission.size(); i++)
    {
      current_step_++;
      if (not is_mission_aborted_)
      {
        Goto(mission.points[i]);
      }
    }
    current_step_ = 0;
    is_mission_aborted_ = false;
    disable_external_mission_.call(srv);
  }

  void CaptainStatusCallback(const cola2_msgs::CaptainStatus& msg)
  {
    // Default values
    plan_control_state_.plan_eta = 0;
    plan_control_state_.plan_progress = 0;
    plan_control_state_.man_id = "";
    // plan_control_state.man_id ??;
    plan_control_state_.man_eta = -1;

    if (msg.mission_active)
    {  // A plan is under execution
      plan_control_state_.state = IMC::PlanControlState::PCS_EXECUTING;

      // Check if is a normal cola2 plan or an external mission (Neptus one)
      int mission_steps = msg.total_steps;
      int current = msg.current_step;
      if (mission_steps < 0)
      {
        mission_steps = mission.size();
        current = current_step_;
      }

      plan_control_state_.plan_progress = int((float(current) / float(mission_steps)) * 100);
      plan_control_state_.plan_eta = mission_steps * TIME_PER_MISSION_STEP;
      plan_control_state_.man_id = std::to_string(current);
      // plan_control_state.man_id ??;
      plan_control_state_.man_eta = -1;
    }
    else
    {  // No plan under execution ...
      if (msg.active_controller == cola2_msgs::CaptainStatus::CONTROLLER_NONE)
      {  // ... and no actions are being executed ...
        if (mission.size() > 0)
        {  // ... and a plan is loaded.
          plan_control_state_.state = IMC::PlanControlState::PCS_READY;
        }
        else
        {  // ... and no plan is loaded.
          plan_control_state_.state = IMC::PlanControlState::PCS_BLOCKED;
        }
      }
      else
      {  // ... but an action (goto, keep position, ...) is under execution
        plan_control_state_.state = IMC::PlanControlState::PCS_BLOCKED;
      }
    }
    if (mission.size() > 0)
    {
      plan_control_state_.plan_id = "last_plan";
    }
    else
    {
      plan_control_state_.plan_id = "";
    }
    plan_control_state_.last_outcome = plan_control_state_.state;
  }

  bool is_mission_aborted_;
  unsigned int current_step_;
  ros::ServiceClient recovery_actions_;
  ros::ServiceClient goto_srv_;
  ros::ServiceClient enable_external_mission_;
  ros::ServiceClient disable_external_mission_;
};

#endif  // SPARUS_AUV_H
