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

TurbotIMCBroker::TurbotIMCBroker() {
  ros::NodeHandle nhp("~");
  estimated_state_pub_ = nhp.advertise<IMC::EstimatedState>("/IMC/In/EstimatedState", 1);
  nav_sts_sub_ = nhp.subscribe("/navigation/nav_sts", 1, &TurbotIMCBroker::NavStsCallback, this);
}

void TurbotIMCBroker::NavStsCallback(const auv_msgs::NavStsConstPtr& msg) {
  IMC::EstimatedState imc_msg;
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

  estimated_state_pub_.publish(imc_msg);
}
