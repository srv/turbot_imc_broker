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

#include <turbot_imc_broker/turbot_imc_broker.h>

#include <tf/tf.h>

// IMC messages to use: classes will be IMC::MessageType
#include <IMC/Spec/EstimatedState.hpp>
#include <IMC/Spec/Announce.hpp>
#include <IMC/Spec/Heartbeat.hpp>
#include <IMC/Spec/RhodamineDye.hpp>
#include <IMC/Spec/VehicleState.hpp>
#include <IMC/Spec/PlanControlState.hpp>
#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/Goto.hpp>
#include <IMC/Spec/StationKeeping.hpp>
#include <IMC/Spec/FollowPath.hpp>


TurbotIMCBroker::TurbotIMCBroker() :
        nav_sts_received_(false),m_eta(0),
        is_plan_loaded_(true) {
  ros::NodeHandle nh("~");

  nh.param("auv_id", params_.auv_id, 0x2000); // 8192
  nh.param("entity_id", params_.entity_id, 0xFF);  // LSTS said 255
  nh.param<std::string>("system_name", params_.system_name, std::string("turbot"));
  nh.param<std::string>("outdir", params_.outdir, std::string("/tmp"));
  nh.param<std::string>("filename", params_.filename, std::string("rhodamine.csv"));

  // Advertise ROS or IMC/Out messages
  estimated_state_pub_ = nh.advertise<IMC::EstimatedState>("/IMC/Out/EstimatedState", 100);
  heartbeat_pub_ = nh.advertise<IMC::Heartbeat>("/IMC/Out/Heartbeat", 100);
  announce_pub_ = nh.advertise<IMC::Announce>("/IMC/Out/Announce", 100);
  rhodamine_pub_ = nh.advertise<IMC::RhodamineDye>("/IMC/Out/RhodamineDye", 100);
  vehicle_state_pub_ = nh.advertise<IMC::VehicleState>("/IMC/Out/VehicleState", 100);
  plan_control_state_pub_ = nh.advertise<IMC::PlanControlState>("/IMC/Out/PlanControlState", 100);

  // TODO IMC messages:
  //  - PlanControl: Neptus sent, used to start and stop a plan, and send quick
  //    plans to a vehicle (goto or Station keeping)
  //  - Abort: Neptus sent, used in case of emergency to stop all activity on
  //    the vehicle
  //  - PlanDB: query by Neptus, used to interact with the vehicle, there are
  //    internal IMC messages used that  are described on the definition of the
  //    PlanDB message
  //  - PlanSpecification: Mission plans. The following parameters can be
  //    ignored: Namespace, Plan Variables, Start Actions, End Actions. A plan
  //    has maneuvers (PlanManeuver, again the Start Actions and End Actions can
  //    be ignored). The most basic maneuvers should be supported:
  //    - Goto
  //    - Follow Path
  //    - Station keeping

  // Subscribe to ROS or IMC/In messages
  abort_sub_ = nh.subscribe("/IMC/In/Abort", 1, &TurbotIMCBroker::AbortCallback, this);
  plan_db_sub_ = nh.subscribe("/IMC/In/PlanDB", 1, &TurbotIMCBroker::PlanDBCallback, this);
  plan_control_sub_ = nh.subscribe("/IMC/In/PlanControl", 1, &TurbotIMCBroker::PlanControlCallback, this);
  rhodamine_sub_ = nh.subscribe("/sensors/rhodamine", 1, &TurbotIMCBroker::RhodamineCallback, this);
#ifdef UIB
  nav_sts_sub_ = nh.subscribe("/navigation/nav_sts", 1, &TurbotIMCBroker::NavStsCallback, this);
  plan_status_sub_ = nh.subscribe("/control/mission_status", 1 , &TurbotIMCBroker::MissionStatusCallback, this);
#endif
#ifdef UDG
  nav_sts_sub_ = nh.subscribe("/cola2_navigation/nav_sts", 1, &TurbotIMCBroker::NavStsCallback, this);
  plan_status_sub_ = nh.subscribe("/cola2_control/captain_status", 1 , &TurbotIMCBroker::CaptainStatusCallback, this);
#endif

  // Create timers
  timer_ = nh.createTimer(ros::Duration(1), &TurbotIMCBroker::Timer, this);

  // Get NED origin
  double ned_lat = 0.0, ned_lon = 0.0;
  const string param_ned_lat = "/navigator/ned_origin_lat";
  const string param_ned_lon = "/navigator/ned_origin_lon";
  if (nh.hasParam(param_ned_lat) && nh.hasParam(param_ned_lon)) {
    nh.getParamCached(param_ned_lat, ned_lat);
    nh.getParamCached(param_ned_lon, ned_lon);
  } else {
    ROS_WARN("NED Origin NOT FOUND!");
  }
  mission_ = new Mission(ned_lat, ned_lon);
}

void TurbotIMCBroker::Timer(const ros::TimerEvent&) {
  if (!nav_sts_received_) return;
  IMC::Announce announce_msg;
  announce_msg.setSource(params_.auv_id);
  announce_msg.setSourceEntity(params_.entity_id);
  announce_msg.setTimeStamp(ros::Time::now().toSec());
  announce_msg.sys_name = params_.system_name;
  announce_msg.sys_type = IMC::SYSTEMTYPE_UUV;
  announce_msg.lat = nav_sts_.global_position.latitude*M_PI/180.0;
  announce_msg.lon = nav_sts_.global_position.longitude*M_PI/180.0;
  announce_msg.height = -nav_sts_.position.depth;
  announce_msg.services = "imc+info://0.0.0.0/version/5.4.8;imc+udp://192.168.1.199:6002;imc+tcp://192.168.1.199:32603";
  announce_pub_.publish(announce_msg);

  // Tell we are alive
  IMC::Heartbeat heartbeat_msg;
  heartbeat_msg.setSource(params_.auv_id);
  heartbeat_msg.setSourceEntity(params_.entity_id);
  heartbeat_msg.setTimeStamp(announce_msg.getTimeStamp());
  heartbeat_pub_.publish(heartbeat_msg);

  IMC::VehicleState vehicle_state_msg;

  vehicle_state_msg.setSource(params_.auv_id);
  vehicle_state_msg.setSourceEntity(params_.entity_id);
  vehicle_state_msg.setTimeStamp(ros::Time::now().toSec());
  if (is_plan_loaded_) { // if plan is loaded, op. mode= MANEUVER (a maneuver is executing)
    vehicle_state_msg.op_mode=3;
    //! Maneuver -- ETA.
    vehicle_state_msg.maneuver_eta=m_eta;
  }
  else{
    vehicle_state_msg.op_mode=0; // else, the vehicle is in op.=SERVICE (ready to service request)
    //! Maneuver -- ETA.
    vehicle_state_msg.maneuver_eta=65535; // value when no maneuver
  }
  // still to define how to capture an error .....
  vehicle_state_msg.error_count=0;
  vehicle_state_msg.error_ents="no error";
  //! Maneuver -- Type.
  vehicle_state_msg.maneuver_type=0;
  //! Maneuver -- Start Time.
  vehicle_state_msg.maneuver_stime=0;
  //! Control Loops.
  vehicle_state_msg.control_loops=0x00000000;  // no use
  //! Flags.
  vehicle_state_msg.flags=0x00; //0x01 when the maneuver is done, 0 elsewhere
  //! Last Error -- Description.
  vehicle_state_msg.last_error="no error";
  //! Last Error -- Time.
  vehicle_state_msg.last_error_time=0;
  vehicle_state_pub_.publish(vehicle_state_msg);

}

#ifdef UDG
void TurbotIMCBroker::CaptainStatusCallback(const cola2_msgs::CaptainStatus& msg) {
  IMC::PlanControlState plan_control_state;
  plan_control_state.setSource(params_.auv_id);
  plan_control_state.setSourceEntity(params_.entity_id);
  plan_control_state.setTimeStamp(ros::Time::now().toSec());

  // Default values
  plan_control_state.plan_eta = 0;
  plan_control_state.plan_progress = 0;
  plan_control_state.man_id = "";
  // plan_control_state.man_id ??;
  plan_control_state.man_eta = -1;

  if (msg.mission_active) { // A plan is under execution
    plan_control_state.state = IMC::PlanControlState::PCS_EXECUTING;
    plan_control_state.plan_eta = msg.total_steps * TIME_PER_MISSION_STEP;
    plan_control_state.plan_progress = int((float(msg.current_step)/float(msg.total_steps))*100);
    plan_control_state.man_id = std::to_string(msg.current_step);
    // plan_control_state.man_id ??;
    plan_control_state.man_eta = -1;
  }
  else { // No plan under execution ...
    if (msg.active_controller == 0) { // ... and no actions are being executed ...
      if (is_plan_loaded_) { // ... and a plan is loaded.
        plan_control_state.state = IMC::PlanControlState::PCS_READY;
      }
      else { // ... and no plan is loaded.
        plan_control_state.state = IMC::PlanControlState::PCS_BLOCKED;
      }
    }
    else { // ... but an action (goto, keep position, ...) is under execution
      plan_control_state.state = IMC::PlanControlState::PCS_BLOCKED;
    }
  }
  if (is_plan_loaded_) {
    plan_control_state.plan_id = "last_plan";
  }
  else {
    plan_control_state.plan_id = "";
  }

  plan_control_state.last_outcome = plan_control_state.state;
  plan_control_state_pub_.publish(plan_control_state);
}
#endif

#ifdef UIB
void TurbotIMCBroker::MissionStatusCallback(const safety::MissionStatus& msg) {
  IMC::PlanControlState plan_control_state;
  plan_control_state.setSource(params_.auv_id);
  plan_control_state.setSourceEntity(params_.entity_id);
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
#endif


void TurbotIMCBroker::RhodamineCallback(const cyclops_rhodamine_ros::RhodamineConstPtr& msg){
  // publish rhodamine message
  IMC::RhodamineDye rhodamine_msg;
  rhodamine_msg.setSource(params_.auv_id);
  rhodamine_msg.setSourceEntity(params_.entity_id);
  rhodamine_msg.setTimeStamp(ros::Time::now().toSec());
  rhodamine_msg.value = static_cast<float>(msg->concentration_ppb);
  rhodamine_pub_.publish(rhodamine_msg);

  // Handle data for CSV logging
  double seconds = msg->header.stamp.toSec();
  double latitude = nav_sts_.global_position.latitude*M_PI/180.0;
  double longitude = nav_sts_.global_position.longitude*M_PI/180.0;
  double depth = nav_sts_.position.depth;

  // Save data in CSV file, required for rsync
  std::string csv_file = params_.outdir + "/" + params_.filename;
  std::fstream f_csv(csv_file.c_str(), std::ios::out | std::ios::app);
  f_csv << std::fixed << std::setprecision(6)
        << seconds << ","
        << latitude << ","
        << longitude << ","
        << depth << ","
        << msg->concentration_ppb << ","
        << msg->concentration_raw << ","
        << -1 <<  endl;
  f_csv.close();
}


void TurbotIMCBroker::NavStsCallback(const auv_msgs::NavStsConstPtr& msg) {
  IMC::EstimatedState imc_msg;
  imc_msg.setSource(params_.auv_id);
  imc_msg.setSourceEntity(params_.entity_id);
  imc_msg.setTimeStamp(ros::Time::now().toSec());

  // NED origin
  imc_msg.lat = msg->origin.latitude*M_PI/180.0;
  imc_msg.lon = msg->origin.longitude*M_PI/180.0;

  imc_msg.height = 0;

  // Offset WRT NED origin
  imc_msg.x = msg->position.north;
  imc_msg.y = msg->position.east;
  imc_msg.z = msg->position.depth;
  imc_msg.depth = msg->position.depth;
  imc_msg.alt = msg->altitude;

  // Vehicle orientation
  imc_msg.phi = msg->orientation.roll;
  imc_msg.theta = msg->orientation.pitch;
  imc_msg.psi = msg->orientation.yaw;

  // Body-fixed frame speeds
  imc_msg.u = msg->body_velocity.x;
  imc_msg.v = msg->body_velocity.y;
  imc_msg.w = msg->body_velocity.z;

  // Body-fixed angular velocity
  imc_msg.p = msg->orientation_rate.roll;
  imc_msg.q = msg->orientation_rate.pitch;
  imc_msg.r = msg->orientation_rate.yaw;

  // Transform velocities to NED frame
  tf::Matrix3x3 R;
  R.setRPY(msg->orientation.roll,
           msg->orientation.pitch,
           msg->orientation.yaw);
  tf::Vector3 body_velocity(msg->body_velocity.x,
                            msg->body_velocity.y,
                            msg->body_velocity.z);
  tf::Vector3 ned_velocity = R.inverse()*body_velocity;

  // NED frame speeds
  imc_msg.vx = ned_velocity.x();
  imc_msg.vy = ned_velocity.y();
  imc_msg.vz = ned_velocity.z();

  nav_sts_ = *msg;
  if (!nav_sts_received_) {
    nav_sts_received_ = true;
  }
  estimated_state_pub_.publish(imc_msg);
}

void TurbotIMCBroker::PlanDBCallback(const IMC::PlanDB& msg) {
  ROS_INFO("IMC::PlanDB message received!");

  if (msg.op == IMC::PlanDB::DBOP_SET) { // if planDB operation = 0, then the argument contains a Plan Specification in the structure of a type Message
    //! Example of possible implementation. NOT TESTED!
    const IMC::Message* cmsg = msg.arg.get(); // obtain data and cast into a constant pointer type Message
    IMC::Message* ncmsg = const_cast<IMC::Message*>(cmsg); // cast to no constant message because the cast operation in PlanSpecification.hpp is not defined as constant
    IMC::PlanSpecification* plan_specification = IMC::PlanSpecification::cast(ncmsg); // cast to no constant PlanSpecification message
    mission_->parse(*plan_specification);




  } else if (msg.op == IMC::PlanDB::DBOP_DEL) {
    // Should delete a record
  } else if (msg.op == IMC::PlanDB::DBOP_GET) {
    // Should return PlanSpecification
  } else if (msg.op == IMC::PlanDB::DBOP_GET_INFO) {
    // Should return PlanDBInformation
  } else if (msg.op == IMC::PlanDB::DBOP_CLEAR) {
    // Should delete all DB records
  } else if (msg.op == IMC::PlanDB::DBOP_GET_STATE) {
    // Should return PlanDbState
  } else {
    ROS_INFO("IMC::PlanDB operation not implemented");
  }
}

void TurbotIMCBroker::PlanControlCallback(const IMC::PlanControl& msg) {
  ROS_INFO("IMC::PlanControl message received!");

  if (msg.op == IMC::PlanControl::PC_START) {
    //! Start Plan.
  } else if (msg.op == IMC::PlanControl::PC_STOP) {
    //! Stop Plan.
  } else if (msg.op == IMC::PlanControl::PC_LOAD) {
    //! Load Plan.
  } else if (msg.op == IMC::PlanControl::PC_GET) {
    //! Get Plan.
  } else {
    ROS_INFO("IMC::PlanControl operation not implemented");
  }
}


void TurbotIMCBroker::AbortCallback(const IMC::Abort& msg) {
  ROS_INFO("IMC::Abort message received!: EMERGENCY SURFACE");
  if (msg.getName() == "Abort")
    auv_.Abort();

  //! Stop mission and disable thrusters
}