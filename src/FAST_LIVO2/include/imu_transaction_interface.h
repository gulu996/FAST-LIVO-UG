#pragma once
#include "IMU_Processing.h"
#include <memory>
#include <stdexcept>
namespace imu_transaction {
// ponytail: single estimator thread; callbacks may append only under the caller's existing buffer lock.
struct InputView {
  std::deque<sensor_msgs::ImuConstPtr> samples;
  size_t eligible=0, releasable=0;
  bool ready=false, right_support=false;
};
inline InputView view(const std::deque<sensor_msgs::ImuConstPtr>& received,
                      const ImuProcess& persistent,double target) {
  InputView v; const double start=persistent.coverageStateTime();
  if(!std::isfinite(target)||target<start)throw std::invalid_argument("query precedes persistent state");
  sensor_msgs::ImuConstPtr right;double previous=-1;
  for(const auto& u:received){
    if(!u)throw std::invalid_argument("null IMU");
    double t=u->header.stamp.toSec();
    if(t<previous)throw std::invalid_argument("unordered received IMU");previous=t;
    if(t>target){right=u;break;}
    ++v.releasable;
    if(t>start+1e-6)v.samples.push_back(u);
  }
  v.eligible=v.samples.size();
  const double last=v.samples.empty()?persistent.coverageCarryTime():v.samples.back()->header.stamp.toSec();
  v.right_support=persistent.coverageNeedsRight(target,v.eligible,last);
  if(v.right_support&&!right)return v;
  if(v.right_support)v.samples.push_back(right);
  v.ready=true;return v;
}
struct Core {
  StatesGroup state;
  ImuProcess::Snapshot imu;
  bool measurement_accepted=false,scan_completed=false;
  explicit Core(const StatesGroup& s,const ImuProcess& p,bool accepted=false,bool scan=false)
    :state(s),imu(p.causalCaptureSnapshot()),measurement_accepted(accepted),scan_completed(scan) {}
};
inline void commit(std::unique_ptr<Core>& destination,const StatesGroup& state,
                   const ImuProcess& process,bool measurement,bool scan) {
  if(!measurement&&!scan)throw std::logic_error("query is not a persistent commit");
  if(!state.rot_end.allFinite()||!state.pos_end.allFinite()||!state.vel_end.allFinite()||
     !state.bias_g.allFinite()||!state.bias_a.allFinite()||!state.gravity.allFinite()||
     !state.cov.allFinite()||!std::isfinite(state.inv_expo_time)||
     !process.coveragePredictionComplete(process.coverageStateTime()))
    throw std::logic_error("invalid persistent handoff");
  auto next=std::make_unique<Core>(state,process,measurement,scan);
  destination.swap(next); // all allocations before this single-thread publication point
}
inline void release(std::deque<sensor_msgs::ImuConstPtr>& received,double committed_time) {
  while(!received.empty()&&received.front()->header.stamp.toSec()<=committed_time)received.pop_front();
}
class StateBinding {
  StatesGroup*& state_;StatesGroup*& prior_;StatesGroup* old_state_;StatesGroup* old_prior_;
public:
  StateBinding(StatesGroup*& state,StatesGroup*& prior,StatesGroup& working,StatesGroup& prediction)
    :state_(state),prior_(prior),old_state_(state),old_prior_(prior){
    if(&working==&prediction)throw std::invalid_argument("working aliases prior");
    state_=&working;prior_=&prediction;
  }
  StateBinding(const StateBinding&)=delete;StateBinding& operator=(const StateBinding&)=delete;
  ~StateBinding(){state_=old_state_;prior_=old_prior_;}
};
// ponytail: single estimator-thread transaction; no visual-map clone or concurrent reader.
class WorkingCheckpoint {
  std::unique_ptr<Core> before_;
  bool coverage_before_=false;
public:
  void begin(const StatesGroup& state,const ImuProcess& process) {
    if(before_)throw std::logic_error("unclosed estimator transaction");
    before_=std::make_unique<Core>(state,process);coverage_before_=process.coverage_complete_;
  }
  void finish(StatesGroup& state,ImuProcess& process,bool measurement,bool scan) {
    if(!before_)throw std::logic_error("missing estimator checkpoint");
    if(measurement||scan){std::unique_ptr<Core> validated;commit(validated,state,process,measurement,scan);}
    else {state=before_->state;process.causalRestoreSnapshot(before_->imu);process.coverage_complete_=coverage_before_;}
    before_.reset();
  }
  bool active()const{return bool(before_);}
};

}
