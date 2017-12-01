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

#include <ros/ros.h>
#include <auv_msgs/NavSts.h>
#include <cyclops_rhodamine_ros/Rhodamine.h>

// Base IMC template
#include <ros_imc_broker/ImcTypes.hpp>
#include <IMC/Base/Packet.hpp>
#include <IMC/Spec/AllMessages.hpp>
// Subscribers
#include <IMC/Spec/PlanControl.hpp>
#include <IMC/Spec/PlanDB.hpp>
#include <IMC/Spec/Abort.hpp>

#include <string>
#include <fstream>

// #define UDG
// #define UIB

#ifdef UDG
#include <cola2_msgs/CaptainStatus.h>
#endif
#ifdef UIB
#include <safety/MissionStatus.h>
#endif

#define TIME_PER_MISSION_STEP   100

using namespace std;


class TurbotIMCBroker {
 public:

  struct Params
  {
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

  inline void setParams(const Params& params){params_ = params;}


  TurbotIMCBroker();


 protected:
  // Callbacks
  void NavStsCallback(const auv_msgs::NavStsConstPtr& msg);
  void Timer(const ros::TimerEvent&);
  void RhodamineCallback(const cyclops_rhodamine_ros::RhodamineConstPtr& msg);
  void PlanDBCallback(const IMC::PlanDB& msg);
  void PlanControlCallback(const IMC::PlanControl& msg);
  void AbortCallback(const IMC::Abort& msg);
#ifdef UDG
  void CaptainStatusCallback(const cola2_msgs::CaptainStatus& msg);
#endif
#ifdef UIB
    void MissionStatusCallback(const safety::MissionStatus& msg);
#endif
 private:

  Params params_; //!> Stores parameters.
  // Publishers
  ros::Publisher estimated_state_pub_;
  ros::Publisher announce_pub_;
  ros::Publisher heartbeat_pub_;
  ros::Publisher vehicle_state_pub_;
  ros::Publisher plan_control_state_pub_;
  ros::Publisher rhodamine_pub_;
  // Subscribers
  ros::Subscriber nav_sts_sub_;
  ros::Subscriber rhodamine_sub_;
  ros::Subscriber plan_status_sub_;
  ros::Subscriber plan_db_sub_;
  ros::Subscriber plan_control_sub_;
  ros::Subscriber abort_sub_;
  // Timers
  ros::Timer timer_;

  auv_msgs::NavSts nav_sts_;
  bool nav_sts_received_;
  bool is_plan_loaded_;
  int m_eta;
};
