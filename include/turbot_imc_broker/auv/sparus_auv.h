#ifndef SPARUS_AUV_H
#define SPARUS_AUV_H

#include <turbot_imc_broker/auv/auv_base.h>

class SparusAUV : public AuvBase {
 public:
  SparusAUV() : AuvBase(auv_id, entity_id) {
    plan_status_sub_ = nh_.subscribe("/cola2_control/captain_status", 1,
                                    &SparusAUV::CaptainStatusCallback, this);
  }

  bool Abort(){

  }

  bool Goto(const MissionPoint& p){

  }

 private:
  void CaptainStatusCallback(const cola2_msgs::CaptainStatus& msg) {

  }

};

#endif // SPARUS_AUV_H
