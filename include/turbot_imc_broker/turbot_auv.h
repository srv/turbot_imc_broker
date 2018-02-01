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

#ifndef INCLUDE_TURBOT_IMC_BROKER_TURBOT_AUV_H_
#define INCLUDE_TURBOT_IMC_BROKER_TURBOT_AUV_H_

#include <turbot_imc_broker/auv_base.h>
#include <std_srvs/Empty.h> // include the empty services for the enable keep position
#include <control/Goto.h>
#include <safety/RecoveryAction.h>
#include <safety/MissionStatus.h>
#include <boost/thread.hpp>


class TurbotAUV : public AuvBase {
 public:
  TurbotAUV() : AuvBase(), srv_stationkeeping_(false) {
    //plan_status_sub_ = nh.subscribe("/control/mission_status", 1 ,
    //                                &TurbotAUV::MissionStatusCallback, this);
    bool oneshot;
    ROS_INFO_STREAM_THROTTLE(1, "[AUV:] Waiting for safety service...");
    recovery_actions_ = nh.serviceClient<safety::RecoveryAction>("/safety/recovery_action");
    client_goto_block_ = nh.serviceClient<control::Goto>("/control/goto_local_block");
    client_enable_keep_position_ = nh.serviceClient<std_srvs::Empty>("/control/enable_keep_position");
    client_disable_keep_position_ = nh.serviceClient<std_srvs::Empty>("/control/disable_keep_position");
    client_disable_goto_ = nh.serviceClient<std_srvs::Empty>("/control/disable_goto");

    // create one shot timer, it fires only ones until is rescheduled again with a start()
   // timer_keep_pos_ = nh.createTimer(ros::Duration(t_station_keeping), &TurbotAUV::Timer_keep_pos, this, oneshot = true);
    bool is_available = recovery_actions_.waitForExistence(ros::Duration(10));
    if (!is_available) {
      ROS_ERROR_STREAM("[ turbot_imc_broker ]: RecoveryAction service is not available.");
    } else {
      ROS_INFO("Service found!");
    }

  }

  /**
   * @brief      Launch the emergency surface service
   *
   * @return     true if successful to launch
   */
  bool Abort() {
    ROS_INFO("ABORTING MISSION");
    safety::RecoveryAction srv_emergency;
    srv_emergency.request.error_level = srv_emergency.request.EMERGENCY_SURFACE; // 3
    if(recovery_actions_.call(srv_emergency)){
      ROS_INFO("Emergency Surface On");
      mission.state_ = MISSION_ABORTED;}
      else{ROS_INFO("A problem in the emergency surface");}

  }

  bool StopMission() {
    // it comes from a PlanControl. The Plan Specification carried on a Plan Control
    // can be a GOTO or a station keeping. At the moment, only controlled if GOTO.
    //if (srv_running_){
    ROS_INFO("[turbot_imc_broker]: STOPPING PLAN");
  //  ROS_INFO_STREAM("[turbot_imc_broker]:  STOPPING PLAN. mission State:" << mission.state_);
    mission.clear();
    //is_plan_loaded_= false;
    if (mission.state_ == MISSION_RUNNING) {
      ROS_INFO("[turbot_imc_broker]: STOPPING GOTO MISSION");
      std_srvs::Empty disable_goto;
      client_disable_goto_.call(disable_goto); // dissable the goto maneuver
      mission.state_ = MISSION_STOPPED;
    }
    //}
    if(mission.state_ == MISSION_STATION_KEEPING){ 
      ROS_INFO("[turbot_imc_broker]: STOPPING STATION KEEPING MISSION");
      std_srvs::Empty disable_keep_position;
      client_disable_keep_position_.call(disable_keep_position); // dissable the station keeping}
      mission.state_ = MISSION_STOPPED;
      srv_stationkeeping_=false;
      // variable stopped is not unset here, but in the Goto process.
      // the problem is that, if we unset it here, in the Goto Subrutine will be unset, when it needs to be set.
    }
  }

// timer interruption service routine for keep position disabling: fires after
  // the programmed duration
  void FinishKeepPos() {
      ROS_INFO("[turbot_imc_broker]: FINISHING THE STATION KEEPING MANEUVER");
      mission.state_ = MISSION_STOPPED; // indicates that the station keeping is inactive.
      std_srvs::Empty disable_keep_position;
      client_disable_keep_position_.call(disable_keep_position); // dissable the station keeping
      srv_stationkeeping_=false;
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
      control::Goto srv;
      srv.request.north_lat = p.north;
      srv.request.east_lon = p.east;
      srv.request.z = p.z;
      srv.request.altitude_mode = p.is_altitude;
      srv.request.tolerance = params.goto_tolerance;

    ROS_INFO_STREAM("[turbot_imc_broker]: call to GOTO subrutine, duration:" << p.duration);
    if (p.duration > -1) { // for station keeping , first goto to the desired location and then call keep position service.
      // ALERT !! duration in seconds = 0 for unlimited
      ROS_INFO_STREAM("[Station Keeping]: GOTO to the goal point !");
      //srv.request.yaw = yaw; //take the yaw fro the Goto Maneuver
      srv.request.tolerance = params.goto_tolerance;
      // goes to a point and once it is at the desired position, it does a station keeping.
      mission.state_ = MISSION_RUNNING;
      if (client_goto_block_.call(srv)) {  // init bloking go to --> waits until the threat is terminated. the return
        // indicates the result of the Go to: goal reached = true, a problem = false.
        if (mission.state_ == MISSION_RUNNING ) { // if the mission is stoped before it reaches the goal point this variable is set in another threat
        // and the call service returns before the vehicle reaches the goal.
          ROS_INFO_STREAM("[Station Keeping]: GOTO to the goal point terminated !");
          std_srvs::Empty keep_position;
          ROS_INFO_STREAM("[Station Keeping]: Enable KEEP POSITION !");
          mission.state_ = MISSION_STATION_KEEPING;
        //  ROS_INFO_STREAM("[turbot_imc_broker]: mission State:" << mission.state_);
          client_enable_keep_position_.call(keep_position); // enable no blocking keep position.
          if (p.duration > 0) { // keep position of limited duration. If duration=0 --> unlimited. 
            t_station_keeping=p.duration; //set the duration of the station keeping time
            ros::Duration(t_station_keeping).sleep(); // sleep for t_station_keeping seconds
            FinishKeepPos(); // finish station keeping.
          }
        //if duration = 0 --> unlimited time until a stop mission is requested
          return true;
        } else { // mission stopped during the Goto. //mission.state_ == MISSION_STOPPED; // keep the state to running for the next goto. If this is the last,the mission is cleaned and te state 
          //is set to EMPTY
          ROS_INFO_STREAM("[Station Keeping]: Mission Stoped before the AUV reached the Goal Point");
          return false;
        }

      } else { // the go to service has had a problem
        ROS_ERROR_STREAM("[Station Keeping]: Failed to call service GOTO " ); //
        mission.state_ = MISSION_FAILED;
        return false;
      }

        // TODO

        /* control if the vehicle is in the desired position is missing. Do the station keeping, only if the desired goal has been reached */

    } else { // for simply goto. Do the goto only if the desired goal has been reached
      ROS_INFO_STREAM("[GOTO]: GOTO to the goal point !");
      mission.state_ = MISSION_RUNNING;
      if (client_goto_block_.call(srv)) { // bloking go to
        if (mission.state_ == MISSION_RUNNING) { // if the mission is stoped before it reaches the goal point this variable is set in another threat
          //, the mission state is set to STOPPED and the call service returns before the vehicle reaches the goal.
          ROS_INFO_STREAM("[GOTO]: Blocking GOTO service Finished!");
          //mission.state_ == MISSION_STOPPED; // keep the state to running for the next goto. If this is the last,the mission is cleaned and the state 
          //is set to EMPTY
          return true;
        } else { // state set to STOPPED
          ROS_INFO_STREAM("[GOTO]: Blocking GOTO service Stoped !");
          return false;
        }

      } else {
        mission.state_ = MISSION_FAILED;
        ROS_ERROR_STREAM("[GOTO]: Failed to call service GOTO " ); //
        return false;
      }

      // necessites un GOTO que bloqueji, per tal de que quan el cridis, no avanci
      // el FOR dels mission points.
      // el llenço a un altre THREAD per tal de que no BLOQUEJI tambe el BROKER
      // De manera que poguem rebre aborts...
    }
  }

  /**
   * @brief      Play current mission if a mission is defined
   *
   * @return     true if successful
   */
  bool PlayMission() {
    if (mission.state_ != MISSION_LOADED) {
      ROS_INFO_STREAM("[ turbot_imc_broker ] Mission is not loaded!");
      return false;
    }
    ROS_INFO_STREAM("[ turbot_imc_broker ] Running Mission !");
   // mission.SetStartingPosition(nav_sts_.position.north, nav_sts_.position.east);

    // calling the GOTOs from a different thread assures the response to an external request of STOP mission from a PLanDBControl
    boost::thread* t;
    t = new boost::thread(&TurbotAUV::PlayMissionThread, this);
  }

  void PlayMissionThread() {
    bool success;
    for (size_t i = 0; i < mission.size(); i++) {
      // break en funció alguna varible si hi ha STOP perque no segueixi enviant GOTOS
      success = Goto(mission.GetNextPoint());
      if (!success) { // if not success (stopped, no goto service, or no keep position service...)), do not continue the mission....
        break;
      }
    } // once the mission has been finished or stoped, clear the vector
    mission.clear();
    if (mission.state_ != MISSION_STATION_KEEPING) { 
      mission.state_ = MISSION_EMPTY; // we can not set the state to empty is there is 
      // a station keeping on line. Otherwise the STOP indefinide Station keeping does not work
    }
  }

 private:

  // void MissionStatusCallback(const safety::MissionStatus& msg) {
  //   if (msg.current_wp > 0) { // A plan is under execution
  //     plan_control_state_.state = IMC::PlanControlState::PCS_EXECUTING;
  //     plan_control_state_.plan_eta = msg.total_wp * TIME_PER_MISSION_STEP;
  //     plan_control_state_.plan_progress = (float(msg.current_wp)/float(msg.total_wp))*100;
  //     plan_control_state_.man_id = std::to_string(msg.current_wp);
  //     // plan_control_state_.man_id ??;
  //     plan_control_state_.man_eta = -1;
  //     plan_control_state_.plan_id = plan_db_.plan_id; // posar nom missió capturada del planDB
  //     //ROS_INFO_STREAM("[turbot_imc_broker]: Mission Status data: " << plan_control_state_.plan_eta << ", "
  //     //  << plan_control_state_.plan_progress << ", " << plan_control_state_.man_id);
  //   } else { // No plan under execution ...
  //     if (is_plan_loaded_) { // ... and a plan is loaded.
  //       ROS_INFO_STREAM("[turbot_imc_broker]: is plan loaded, plan id: " << is_plan_loaded_ << ", " << plan_db_.plan_id);
  //       plan_control_state_.state = IMC::PlanControlState::PCS_READY;
  //       plan_control_state_.plan_id = plan_db_.plan_id; // posar nom missió capturada del planDB
  //     } else { // ... and no plan is loaded.
  //       //ROS_INFO_STREAM("[turbot_imc_broker]: plan not loaded, plan id: " << is_plan_loaded_ << ", " << plan_db_.plan_id);
  //       plan_control_state_.state = IMC::PlanControlState::PCS_BLOCKED;
  //       plan_control_state_.plan_id = "Mission status -- No plan Loaded";
  //     }
  //   }

  //   plan_control_state_.last_outcome = plan_control_state_.state;
  // }


  ros::ServiceClient recovery_actions_;
  ros::ServiceClient client_goto_block_;
  ros::ServiceClient client_disable_goto_;
  ros::ServiceClient client_enable_keep_position_;
  ros::ServiceClient client_disable_keep_position_;

  bool srv_stationkeeping_;
 // bool srv_running_;
};

#endif  // INCLUDE_TURBOT_IMC_BROKER_TURBOT_AUV_H_
