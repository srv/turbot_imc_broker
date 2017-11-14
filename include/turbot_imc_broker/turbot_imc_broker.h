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
#include <string>
#include <fstream>

using namespace std;


class TurbotIMCBroker {
 public:

  struct Params
  {
    string outdir;            //!> Output directory
    string filename;
    // Default settings
    Params () {
      outdir            = "";
      filename          = "";
      }
  };

  inline void setParams(const Params& params){params_ = params;}


  TurbotIMCBroker();


 protected:
  // Callbacks
  void NavStsCallback(const auv_msgs::NavStsConstPtr& msg);
  void AnnounceTimer(const ros::TimerEvent&);
  void RhodamineCallback(const cyclops_rhodamine_ros::RhodamineConstPtr& msg);


 private:

  Params params_; //!> Stores parameters.
  ros::Publisher estimated_state_pub_;
  ros::Publisher announce_pub_;
  ros::Subscriber nav_sts_sub_;
  ros::Publisher rhodamine_pub_;
  ros::Subscriber rhodamine_sub_;
  ros::Timer announce_timer_;

  auv_msgs::NavSts nav_sts_;
  bool nav_sts_received_;

   
};
