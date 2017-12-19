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

#ifndef AUV_H
#define AUV_H

#include <ros/ros.h>
#include <auv_msgs/NavSts.h>
#include <cyclops_rhodamine_ros/Rhodamine.h>

#include <turbot_imc_broker/mission.h>

// Base IMC template
#include <ros_imc_broker/ImcTypes.hpp>
#include <IMC/Base/Packet.hpp>
#include <IMC/Spec/AllMessages.hpp>

#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/Goto.hpp>
#include <IMC/Spec/StationKeeping.hpp>
#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/FollowPath.hpp>
#include <IMC/Spec/PlanControlState.hpp>

// Subscribers
#include <IMC/Spec/PlanControl.hpp>
#include <IMC/Spec/PlanDB.hpp>
#include <IMC/Spec/Abort.hpp>

class AuvBase { // parent class
 public:
  struct Params {
    std::string outdir;        //!> Output directory
    std::string filename;      //!> CSV filename
    std::string system_name;   //!> AUV name
    int auv_id;                //!> AUV identifier
    int entity_id;             //!> Entity identifier
    // Default settings
    Params () {
      outdir            = "/tmp";
      filename          = "rhodamine.csv";
      system_name       = "turbot";
      auv_id            = 0x2000; // 8192 turbot identifier
      entity_id         = 0xFF; // 255 turbot identifier
      }
  };

  /**
   * @brief      Constructor
   */
  AuvBase();

  /**
   * @brief      Loads a mission.
   *
   * @param[in]  msg   The message
   */
  void LoadMission(const IMC::PlanSpecification& msg);

  /**
   * @brief      Starts a mission.
   */
  void StartMission();

  /**
   * @brief      Clears a mission
   */
  void ClearMission();

  //@note: Getters in alphabetical order

  /**
   * @brief      Gets the plan control state.
   *
   * @return     The plan control state.
   */
  IMC::PlanControlState GetPlanControlState() {
    plan_control_state_.setTimeStamp(ros::Time::now().toSec());
    return plan_control_state_;
  }


  /**
   * @brief      Gets the vehicle state.
   *
   * @return     The vehicle state.
   */
  IMC::VehicleState GetVehicleState() {
    vehicle_state_.setTimeStamp(ros::Time::now().toSec());
    return vehicle_state_;
  }

  virtual bool Abort() = 0;
  virtual bool Goto(const MissionPoint& p) = 0;

  ros::NodeHandle nh;
  Mission mission;  //!> Stores PlanDB missions.
  Params params; //!> Stores parameters.

 protected:
  //@note: Callbacks in alphabetical order

  /**
   * @brief      Stop mission and disable thrusters
   *
   * @param[in]  msg   The message
   */
  void AbortCallback(const IMC::Abort& msg);


  /**
   * @brief      Callback for Navigation status
   *
   * @param[in]  msg   The message
   */
  void NavStsCallback(const auv_msgs::NavStsConstPtr& msg);

  /**
   * @brief      Callback for IMC PlanDB
   *
   * @param[in]  msg   The message
   */
  void PlanDBCallback(const IMC::PlanDB& msg);

  /**
   * @brief      Callback for IMC PlanControl
   *
   * @param[in]  msg   The message
   */
  void PlanControlCallback(const IMC::PlanControl& msg);

  /**
   * @brief      Callback for ROS Rhodamine reading
   *
   * @param[in]  msg   The message
   */
  void RhodamineCallback(const cyclops_rhodamine_ros::RhodamineConstPtr& msg);

  /**
   * @brief      Periodic function callback
   *
   * @param[in]  <unnamed>  unused
   */
  void Timer(const ros::TimerEvent&);

  // Publishers
  ros::Publisher estimated_state_pub_;
  ros::Publisher vehicle_state_pub_;
  ros::Publisher rhodamine_pub_;
  ros::Publisher plan_control_state_pub_;
  ros::Publisher plan_db_pub_;
  ros::Publisher plan_control_pub_;

  // Subscribers
  ros::Subscriber rhodamine_sub_;
  ros::Subscriber plan_db_sub_;
  ros::Subscriber plan_control_sub_;
  ros::Subscriber abort_sub_;
  ros::Subscriber nav_sts_sub_;

  // To be implemented by child classes
  ros::Subscriber plan_status_sub_;

  // Timers
  ros::Timer timer_;

  // ROS messages
  auv_msgs::NavSts nav_sts_;

  bool nav_sts_received_;
  bool is_plan_loaded_;

  // IMC messages
  IMC::PlanControlState plan_control_state_;
  IMC::VehicleState vehicle_state_;
};

#endif // AUV_H


