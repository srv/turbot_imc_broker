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

// Base IMC template
#include <ros_imc_broker/ImcTypes.hpp>
#include <IMC/Base/Packet.hpp>
#include <IMC/Spec/AllMessages.hpp>

// IMC messages to use: classes will be IMC::MessageType
#include <IMC/Spec/EstimatedState.hpp>
#include <IMC/Spec/Announce.hpp>
#include <IMC/Spec/Heartbeat.hpp>

TurbotIMCBroker::TurbotIMCBroker() : nav_sts_received_(false) {
  ros::NodeHandle nhp("~");

  nhp.param("auv_id", auv_id_, 0x1A);
  nhp.param("entity_id", entity_id_, 255);  // LSTS said
  nhp.param<std::string>("system_name", system_name_, std::string("turbot-auv"));

  // Advertise ROS or IMC/Out messages
  estimated_state_pub_ = nhp.advertise<IMC::EstimatedState>("/IMC/Out/EstimatedState", 100);
  heartbeat_pub_ = nhp.advertise<IMC::Heartbeat>("/IMC/Out/Heartbeat", 100);
  announce_pub_ = nhp.advertise<IMC::Announce>("/IMC/Out/Announce", 100);

  // Subscribe to ROS or IMC/In messages
  nav_sts_sub_ = nhp.subscribe("/navigation/nav_sts", 1, &TurbotIMCBroker::NavStsCallback, this);

  // Create timers
  announce_timer_ = nhp.createTimer(ros::Duration(1), &TurbotIMCBroker::AnnounceTimer, this);
}

void TurbotIMCBroker::AnnounceTimer(const ros::TimerEvent&) {
  if (!nav_sts_received_) return;
  IMC::Announce announce_msg;
  announce_msg.setSource(auv_id_);
  announce_msg.setSourceEntity(entity_id_);
  announce_msg.setTimeStamp(ros::Time::now().toSec());
  announce_msg.sys_name = system_name_;
  announce_msg.sys_type = IMC::SYSTEMTYPE_UUV;
  announce_msg.owner = 0xFFFF;  // ?
  announce_msg.lat = nav_sts_.global_position.latitude*M_PI/180.0;
  announce_msg.lon = nav_sts_.global_position.longitude*M_PI/180.0;
  announce_msg.height = -nav_sts_.position.depth;
  //TODO announce services info:
  announce_msg.services = "imc+info://0.0.0.0/version/5.4.8;imc+udp://10.0.10.80:6002";
  announce_pub_.publish(announce_msg);

  // Tell we are alive
  IMC::Heartbeat heartbeat_msg;
  heartbeat_msg.setSource(auv_id_);
  heartbeat_msg.setSourceEntity(entity_id_);
  heartbeat_msg.setTimeStamp(announce_msg.getTimeStamp());
  heartbeat_pub_.publish(heartbeat_msg);
}

void TurbotIMCBroker::NavStsCallback(const auv_msgs::NavStsConstPtr& msg) {
  IMC::EstimatedState imc_msg;
  imc_msg.setSource(auv_id_);
  imc_msg.setSourceEntity(entity_id_);
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
  tf::Vector3 body_velocity(msg->body_velocity.x, msg->body_velocity.y, msg->body_velocity.z);
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
