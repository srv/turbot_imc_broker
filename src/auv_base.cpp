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

#include <turbot_imc_broker/auv_base.h>

#include <tf/tf.h>

#include <fstream>

AuvBase::AuvBase() : nh("~"), nav_sts_received_(false), is_plan_loaded_(false) {
  // Get parameters from parameter server
  nh.param("outdir", params.outdir, std::string(""));
  nh.param("filename", params.filename, std::string(""));
  nh.param("auv_id", params.auv_id, 0x2000); // 8192
  nh.param("entity_id", params.entity_id, 0xFF);  // LSTS said 255
  nh.param<std::string>("system_name", params.system_name, std::string("turbot"));
  nh.param<std::string>("outdir", params.outdir, std::string("/tmp"));
  nh.param<std::string>("filename", params.filename, std::string("rhodamine.csv"));

  // Plan control state default values
  plan_control_state_.setSource(params.auv_id);
  plan_control_state_.setSourceEntity(params.entity_id);
  plan_control_state_.plan_eta = 0;
  plan_control_state_.plan_progress = 0;
  plan_control_state_.man_id = "";
  // plan_control_state_.man_id ??;
  plan_control_state_.man_eta = -1;

  // Vehicle state default values
  vehicle_state_.setSource(params.auv_id);
  vehicle_state_.setSourceEntity(params.entity_id);

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

  // Advertise ROS or IMC/Out messages
  estimated_state_pub_ = nh.advertise<IMC::EstimatedState>("/IMC/Out/EstimatedState", 100);
  rhodamine_pub_ = nh.advertise<IMC::RhodamineDye>("/IMC/Out/RhodamineDye", 100);
  vehicle_state_pub_ = nh.advertise<IMC::VehicleState>("/IMC/Out/VehicleState", 100);
  plan_control_state_pub_ = nh.advertise<IMC::PlanControlState>("/IMC/Out/PlanControlState", 100);
  plan_db_pub_ = nh.advertise<IMC::PlanDB>("/IMC/Out/PlanDB", 100);
  plan_control_pub_ = nh.advertise<IMC::PlanControl>("/IMC/Out/PlanControl", 100);

  // Subscribe to ROS or IMC/In messages
  abort_sub_ = nh.subscribe("/IMC/In/Abort", 1, &AuvBase::AbortCallback, this);
  plan_db_sub_ = nh.subscribe("/IMC/In/PlanDB", 1, &AuvBase::PlanDBCallback, this);
  plan_control_sub_ = nh.subscribe("/IMC/In/PlanControl", 1, &AuvBase::PlanControlCallback, this);

  // Topics to be renamed depending on vehicle
  rhodamine_sub_ = nh.subscribe("rhodamine", 1, &AuvBase::RhodamineCallback, this);
  nav_sts_sub_ = nh.subscribe("nav_sts", 1, &AuvBase::NavStsCallback, this);

  // Create timers
  timer_ = nh.createTimer(ros::Duration(1), &AuvBase::Timer, this);
}

void AuvBase::Timer(const ros::TimerEvent&) {
  if (!nav_sts_received_) return;
  vehicle_state_pub_.publish(GetVehicleState());
  plan_control_state_pub_.publish(GetPlanControlState());
}

void AuvBase::RhodamineCallback(const cyclops_rhodamine_ros::RhodamineConstPtr& msg){
  // publish rhodamine message
  IMC::RhodamineDye rhodamine_msg;
  rhodamine_msg.setSource(params.auv_id);
  rhodamine_msg.setSourceEntity(params.entity_id);
  rhodamine_msg.setTimeStamp(ros::Time::now().toSec());
  rhodamine_msg.value = static_cast<float>(msg->concentration_ppb);
  rhodamine_pub_.publish(rhodamine_msg);

  // Handle data for CSV logging
  double seconds = msg->header.stamp.toSec();
  double latitude = nav_sts_.global_position.latitude*M_PI/180.0;
  double longitude = nav_sts_.global_position.longitude*M_PI/180.0;
  double depth = nav_sts_.position.depth;

  // Save data in CSV file, required for rsync
  std::string csv_file = params.outdir + "/" + params.filename;
  std::fstream f_csv(csv_file.c_str(), std::ios::out | std::ios::app);
  f_csv << std::fixed << std::setprecision(6)
        << seconds << ","
        << latitude << ","
        << longitude << ","
        << depth << ","
        << msg->concentration_ppb << ","
        << msg->concentration_raw << ","
        << -1 <<  std::endl;
  f_csv.close();
}

void AuvBase::NavStsCallback(const auv_msgs::NavStsConstPtr& msg) {
  // Copy message
  nav_sts_ = *msg;

  IMC::EstimatedState imc_msg;
  imc_msg.setSource(params.auv_id);
  imc_msg.setSourceEntity(params.entity_id);
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

  if (!nav_sts_received_) {
    nav_sts_received_ = true;
  }
  estimated_state_pub_.publish(imc_msg);
}

void AuvBase::PlanDBCallback(const IMC::PlanDB& msg) {
  // Copy message
  plan_db_ = msg;

  plan_db_.type = IMC::PlanDB::DBT_SUCCESS;

  if (msg.op == IMC::PlanDB::DBOP_SET) { // if planDB operation = 0, then the argument contains a Plan Specification in the structure of a type Message
    //! Example of possible implementation. NOT TESTED!
    ROS_INFO("IMC::PlanDB SET");
    const IMC::Message* cmsg = msg.arg.get(); // obtain data and cast into a constant pointer type Message
    IMC::Message* ncmsg = const_cast<IMC::Message*>(cmsg); // cast to no constant message because the cast operation in PlanSpecification.hpp is not defined as constant
    IMC::PlanSpecification* plan_specification = IMC::PlanSpecification::cast(ncmsg); // cast to no constant PlanSpecification message
    plan_specification_ = *plan_specification;
    mission.parse(*plan_specification);
    plan_db_.arg.set(plan_specification_);
  } else if (msg.op == IMC::PlanDB::DBOP_DEL) {
    // Should delete a record
    ROS_INFO("IMC::PlanDB DELETE");
    plan_db_.arg.clear();
  } else if (msg.op == IMC::PlanDB::DBOP_GET) {
    // Should return PlanSpecification
    ROS_INFO("IMC::PlanDB GET");
    plan_db_.arg.set(plan_specification_);
  } else if (msg.op == IMC::PlanDB::DBOP_GET_INFO) {
    // Should return PlanDBInformation
    ROS_INFO("IMC::PlanDB GET INFO");
    IMC::PlanDBInformation info = CreateInfo(plan_specification_);
    plan_db_.arg.set(info);
  } else if (msg.op == IMC::PlanDB::DBOP_CLEAR) {
    // Should delete all DB records
    ROS_INFO("IMC::PlanDB CLEAR");
    plan_db_.arg.clear();
    mission.points.clear();
  } else if (msg.op == IMC::PlanDB::DBOP_GET_STATE) {
    // Should return PlanDbState
    ROS_INFO("IMC::PlanDB GET STATE");
    IMC::PlanDBState state = CreateState(plan_specification_);
    plan_db_.arg.set(state);
  } else {
    ROS_INFO("IMC::PlanDB operation not implemented");
    plan_db_.type = IMC::PlanDB::DBT_FAILURE;
  }

  // Publish reply
  plan_db_pub_.publish(plan_db_);
}

IMC::PlanDBState AuvBase::CreateState(const IMC::PlanSpecification& spec) {
  IMC::PlanDBState state;
  state.plan_count = static_cast<uint16_t>(is_plan_loaded_);
  state.plan_size = spec.getSerializationSize();
  state.change_time = spec.getTimeStamp();
  state.change_sid = spec.getSourceEntity();
  state.change_sname = spec.getName();  // TODO: change for source name. Neptus?
  state.plans_info.push_back(CreateInfo(spec));
  state.md5 = ComputeMD5(spec.toString());
  return state;		
}

IMC::PlanDBInformation AuvBase::CreateInfo(const IMC::PlanSpecification& spec) {
  IMC::PlanDBInformation info;
  info.plan_id = spec.plan_id;
  info.plan_size = spec.getSerializationSize();
  info.change_time = spec.getTimeStamp();
  info.change_sid = spec.getSourceEntity();
  info.change_sname = spec.getName();  // TODO: change for source name. Neptus?
  info.md5 = ComputeMD5(spec.toString());
  return info;		
}

void AuvBase::PlanControlCallback(const IMC::PlanControl& msg) {
  // Copy message
  plan_control_ = msg;

  if (msg.op == IMC::PlanControl::PC_START) {
    //! Start Plan.
    ROS_INFO("IMC::PlanControl START");
    
    if (!msg.arg.isNull()) {
      // If arg exists: it's a quick plan
      const IMC::Message* cmsg = msg.arg.get();
      IMC::Message* ncmsg = const_cast<IMC::Message*>(cmsg);
      IMC::PlanSpecification* plan_specification = IMC::PlanSpecification::cast(ncmsg);
      mission.parse(*plan_specification);
    }

    for (size_t i = 0; i < mission.size(); i++) {
      Goto(mission.points[i]);
    }
  } else if (msg.op == IMC::PlanControl::PC_STOP) {
    //! Stop Plan.
    ROS_INFO("IMC::PlanControl STOP");
    StopMission();
  } else if (msg.op == IMC::PlanControl::PC_LOAD) {
    //! Load Plan.
    ROS_INFO("IMC::PlanControl LOAD");
  } else if (msg.op == IMC::PlanControl::PC_GET) {
    //! Get Plan.
    ROS_INFO("IMC::PlanControl GET");
  } else {
    ROS_INFO("IMC::PlanControl operation not implemented");
  }
}

void AuvBase::AbortCallback(const IMC::Abort& msg) {
  ROS_INFO("IMC::Abort message received!: EMERGENCY SURFACE");
  if (msg.getName() == "Abort")
    Abort();
}