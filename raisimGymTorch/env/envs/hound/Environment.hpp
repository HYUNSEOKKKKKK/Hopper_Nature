//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) {

    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
    hound_ = world_->addArticulatedSystem(resourceDir_+"../hound/rsc/Hound/Hound_foot_cylinder_20230328.urdf");
    hound_->setName("hound");
    hound_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /// initialize containers
    gc_.setZero(19); gcInit_.setZero();
    gv_.setZero(18); gvInit_.setZero();
    gcDes_.setZero(); gvDes_.setZero();
    pTarget_.setZero(); prevTarget_.setZero(); prevPrevTarget_.setZero();

    /// this is nominal configuration of anymal
    gcInit_ << 0, 0, 0.51875, 1.0, 0.0, 0.0, 0.0, -0.0, 0.7854, -1.5708, 0.0, 0.7854, -1.5708, -0.0, 0.7854, -1.5708, 0.0, 0.7854, -1.5708;
    gcInit_.segment(3,4).normalize();

    /// set pd gains
    Eigen::Vector<double,18> jointPgain, jointDgain;
    jointPgain.setZero(); jointPgain.tail(12).setConstant(50.0);
    jointDgain.setZero(); jointDgain.tail(12).setConstant(1.0);
    hound_->setPdGains(jointPgain, jointDgain);
    hound_->setGeneralizedForce(Eigen::VectorXd::Zero(18));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 34;
    actionDim_ = 12;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gcInit_.tail(12);
    double action_std;
    READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
    actionStd_.setConstant(action_std);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.push_back(hound_->getBodyIdx("RR_calf"));
    footIndices_.push_back(hound_->getBodyIdx("RL_calf"));
    footIndices_.push_back(hound_->getBodyIdx("FR_calf"));
    footIndices_.push_back(hound_->getBodyIdx("FL_calf"));
    footFrames_.push_back("RR_foot_fixed");
    footFrames_.push_back("RL_foot_fixed");
    footFrames_.push_back("FR_foot_fixed");
    footFrames_.push_back("FL_foot_fixed");

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(hound_);
    }

    /// set limit for log barrier function
    for (int i=0;i<4;i++){
      limitJointPos_.row(i*3+0) << -0.523599,0.523599; // roll : (-pi/6, pi/6)
      limitJointPos_.row(i*3+1) << 0,1.570796; // hip : 0, pi*1/2
      limitJointPos_.row(i*3+2) << -2.6179933,-0.5235987; // knee : -pi*5/6, -pi/6
    }
    limitBodyHeight_ << 0.48, 0.62;
    limitBaseMotion_ << -0.3,0.3;
    limitJointVel_ << -8,8;
    limitTargetVel_ << -0.4,0.4;
    limitFootContact_ << -0.3,3;
    limitFootClearance_ << -0.12,0.12; // 어차피 desired_foot_clearance 를

    /// initialize
    command_.setZero();
    footContact_.setZero();
    footVel_.resize(4); footPos_.resize(4);
    footContactPhase_.setZero();
    footClearance_.setZero();
    standingMode_ = false;

    /// initialize history
    jointPosErrorHist_ = std::vector<Eigen::Vector<double,12>>(18,Eigen::Vector<double,12>::Zero());
    jointVelHist_ = std::vector<Eigen::Vector<double,12>>(18,Eigen::Vector<double,12>::Zero());
  }

  void init() final { }

  void reset() final {
    hound_->setState(gcInit_, gvInit_);
    updateObservation();

    command_ << 1.5 * uniDist_(gen_), 0.6 * uniDist_(gen_), 0.6 * uniDist_(gen_); // [1.5, 0.6, 0.6]
    standingMode_ = false;

    pTarget_ = gc_.tail(12);
    gcDes_.tail(12) = pTarget_; prevTarget_ = pTarget_; prevPrevTarget_ = pTarget_;
    for (auto& vec : jointPosErrorHist_) { vec.setZero(); }
    for (auto& vec : jointVelHist_) { vec.setZero(); }
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    pTarget_ = action.cast<double>();
    pTarget_ = pTarget_.cwiseProduct(actionStd_);
    pTarget_ += actionMean_;                                   /// joint P target
    gcDes_.tail(12) = pTarget_;
    hound_->setPdTarget(gcDes_, gvDes_);

    /// simulation
    double avgReward = 0.0;
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
      updateObservation();
      avgReward += getReward();
    }
    avgReward /= (control_dt_ / simulation_dt_ + 1e-10);

    return avgReward;
  }

  void updateHistory(){
      prevPrevTarget_ = prevTarget_;
      prevTarget_ = pTarget_;

      jointVelHist_.erase(jointVelHist_.begin());
      jointVelHist_.push_back(gv_.tail(nJoints_));

      jointPosErrorHist_.erase(jointPosErrorHist_.begin());
      jointPosErrorHist_.push_back(pTarget_ - gc_.tail(nJoints_));
  }

  double getReward(){
      double desiredFootZPosition = 0.1;
//      updateObservation(world);        // update contact state, body state (ori, lin vel, ang vel), foot pos
//      updateFootToTerrain(heightMap_); // sample surrounding height (5 sample for each foot)

      /// A variable for foot slip reward and foot clearance reward
      double footTangentialForSlip = 0;
      for(int i = 0; i < 4; i++) {
          if (footContact_(i)) { // contact 이면
              footTangentialForSlip += footVel_[i].e().head(2).squaredNorm();
          }
      }

      /// for gait enforcing & foot clearance
      phase_ += simulation_dt_;
      double gait_hz = 0.68; // 1/0.68 -> stance swing : 0.34, 0.34
      footContactPhase_(0) = sin(phase_/gait_hz * 2*3.141592); // RR
      footContactPhase_(1) = -footContactPhase_(0); // RL
      footContactPhase_(2) = -footContactPhase_(0); // FR
      footContactPhase_(3) = footContactPhase_(0); // FL
//
      if (!standingMode_){ /// walking
          /// footContactDouble_ -> limit_foot_contact 에 있도록 (-0.3,3) -> Gait Enforcing
          for(int i=0; i<4; i++){
              if (footContact_(i)) {footContactDouble_(i) = 1.0 * footContactPhase_(i);}
              else {footContactDouble_(i) = -1.0 * footContactPhase_(i);}
          }
          /// footClearance_ -> limit_foot_clearance 에 있도록 (-0.12,0.12) -> foot 드는 거 enforcing
          desiredFootZPosition = 0.15;
          for (int i=0; i<4; i++){
              if (footContactPhase_(i) < -0.5) {
                  footClearance_(i) =
                          footToTerrain_.segment(i * 5, 5).minCoeff() - desiredFootZPosition; // 대략, 0.17 sec, 0 보다 크거나 같으면 됨 (enforcing clearance)
              }else{ footClearance_(i) = 0.0; } // max reward (not enforcing clearance)
          }
      } else { /// under standingMode_
          /// standingMode_ 는 zero command 로 부터 유추 가능, command 는 obs 이기 때문에, robot 은 standingMode_인지 아닌지 충분히 알 수 있음
          for (int i=0; i<4; i++){
              footContactDouble_(i) = 1.0; // around max reward, where this value should go under (-0.3,3)
              footClearance_(i) = 0.0; // max reward (not enforcing clearance)
          }
      }

      /// positive reward
      rewards_.record("rewBodyAngularVel", std::exp(-1.5 * pow((command_(2) - bodyAngularVel_(2)), 2)));
      rewards_.record("rewLinearVel", std::exp(-1.0 * (command_.head(2) - bodyLinearVel_.head(2)).squaredNorm()));
      /// negative reward
      double rewJointVelStanding,rewJointPosStanding,rewJointAcc;
      if (!standingMode_){
          rewJointVelStanding = 0.0;
          rewJointPosStanding = 0.0;
          rewJointAcc = 0.0;
          limitBaseMotion_ << -0.3,0.3;
      } else {
          rewards_.record("rewJointVelStanding", gv_.tail(12).squaredNorm());
          rewards_.record("rewJointPosStanding", (gc_.tail(12)-gcInit_.tail(12)).squaredNorm());
//          rewards_.record("rewJointAcc", (gv_.tail(12) - preJointVel_).squaredNorm());
          limitBaseMotion_ << -0.1,0.1;
      }
      rewards_.record("rewTorque", hound_->getGeneralizedForce().squaredNorm());
      rewards_.record("rewFootSlip", footTangentialForSlip);
      rewards_.record("rewBodyOri", std::acos(rot_(8)) * std::acos(rot_(8)));
      rewards_.record("rewSmoothness1", (pTarget_ - prevTarget_).squaredNorm());
      rewards_.record("rewSmoothness2", (pTarget_ - 2 * prevTarget_ + prevPrevTarget_).squaredNorm());

      /// relaxed log barrier
      // Log Barrier - limit_joint_pos
      double rewRLOGjointPos = 0.0, temp_reward_joint_limit = 0.0;
      for (int i=0;i<4;i++){
          for (int j=0;j<3;j++){
              int index_leg = i*3+j;
              if (j==0 and i%2 == 1){ // RL, FL roll 만
                  relaxedLogBarrier(0.09,limitJointPos_(index_leg,0),limitJointPos_(index_leg,1),-gc_(7+index_leg),temp_reward_joint_limit);
              }else{
                  relaxedLogBarrier(0.09,limitJointPos_(index_leg,0),limitJointPos_(index_leg,1),gc_(7+index_leg),temp_reward_joint_limit);
              }
              rewRLOGjointPos += temp_reward_joint_limit;
          }
      }
      rewRLOGjointPos *= 1e-1;
      // Log Barrier - limit_body_height
      double rewRLOGbodyHeight = 0.0;
      double body_height_to_terrain = 0.0;
//      if(isHeightMap_) {
//          // naive foot to terrain
//          for (int i=0; i<4; i++){
//              body_height_to_terrain += gc_(2) - heightMap_->getHeight(footPos_[i].e()(0), footPos_[i].e()(1));
//          }
//          body_height_to_terrain /= 4;
//      }else{
          body_height_to_terrain = gc_(2);
//      }
      relaxedLogBarrier(0.05,limitBodyHeight_(0),limitBodyHeight_(1),body_height_to_terrain,rewRLOGbodyHeight);
      rewRLOGbodyHeight *= 1e-1;
      // Log Barrier - limit_base_motion
      double rewRLOGbaseMotion = 0.0, temp_reward_base_motion = 0.0;
      relaxedLogBarrier(0.2,limitBaseMotion_(0),limitBaseMotion_(1),bodyLinearVel_(2),temp_reward_base_motion);
      rewRLOGbaseMotion += temp_reward_base_motion;
      for (int i=0;i<2;i++){
          relaxedLogBarrier(0.2,limitBaseMotion_(0),limitBaseMotion_(1),bodyAngularVel_(i),temp_reward_base_motion);
          rewRLOGbaseMotion += temp_reward_base_motion;
      }
      rewRLOGbaseMotion *= 1e-1;
      // Log Barrier - limit_joint_vel
      double rewRLOGjointVel = 0.0, temp_reward_joint_vel_limit = 0.0;
      for (int i=0;i<12;i++){
          relaxedLogBarrier(2.0,limitJointVel_(0),limitJointVel_(1),gv_(6+i),temp_reward_joint_vel_limit);
          rewRLOGjointVel += temp_reward_joint_vel_limit;
      }
      rewRLOGjointVel *= 1e-1;
      // Log Barrier - limit_target_vel
      double rewRLOGtargetVel = 0.0, temp_reward_target_vel = 0.0;
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(0)-command_(0),temp_reward_target_vel);
      rewRLOGtargetVel += temp_reward_target_vel;
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(1)-command_(1),temp_reward_target_vel);
      rewRLOGtargetVel += temp_reward_target_vel;
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyAngularVel_(2)-command_(2),temp_reward_target_vel);
      rewRLOGtargetVel += temp_reward_target_vel;
      rewRLOGtargetVel *= 1e-1;
      // Log Barrier - limit_foot_contact
      double rewRLOGfootContact = 0.0, temp_reward_foot_contact = 0.0;
      for (int i=0;i<4;i++){
          relaxedLogBarrier(0.1,limitFootContact_(0),limitFootContact_(1),footContactDouble_(i),temp_reward_foot_contact);
          rewRLOGfootContact += temp_reward_foot_contact;
      }
      rewRLOGfootContact *= 1e-1;
      // Log Barrier - limit_foot_clearance
      double rewRLOGfootClearance = 0.0, temp_reward_foot_clearance = 0.0;
      for (int i=0;i<4;i++){
          relaxedLogBarrier(0.03,limitFootClearance_(0),limitFootClearance_(1),footClearance_(i),temp_reward_foot_clearance);
          rewRLOGfootClearance += temp_reward_foot_clearance;
      }
      rewRLOGfootClearance *= 1e-1;


      return rewards_.sum();
  }

  void relaxedLogBarrier(const double& delta,const double& alpha_lower,const double& alpha_upper,const double& x, double& y){
      /// positive reward, boundary 밖에서 gradient 가 큼
      double x_temp = x-alpha_lower;
      // lower bound
      if (x_temp < delta){
          y = 0.5*(pow((x_temp-2*delta)/delta,2)-1) - log(delta);
      }else{
          y = -log(x_temp);
      }
      // upper bound
      x_temp = -(x-alpha_upper);
      if (x_temp < delta){
          y += 0.5*(pow((x_temp-2*delta)/delta,2)-1) - log(delta);
      }else{
          y += -log(x_temp);
      }
      y *= -1;
  }


    void updateObservation() {
    hound_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot_);
    bodyLinearVel_ = rot_.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot_.e().transpose() * gv_.segment(3, 3);
    for(int i = 0; i < 4; i++) {
      hound_->getFramePosition(footFrames_[i], footPos_[i]);
      hound_->getFrameVelocity(footFrames_[i], footVel_[i]);
    }

    obDouble_ << gc_[2], /// body height
        rot_.e().row(2).transpose(), /// body orientation
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.tail(12); /// joint velocity

    /// foot contact update
    footContact_.setZero();
    for(auto& contact: hound_->getContacts()){
        for (size_t i=0; i<4; i++){
            if(contact.getlocalBodyIndex() == footIndices_[i]){
                footContact_(i) = 1;
            }
        }
    }
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: hound_->getContacts())
        if (std::find(footIndices_.begin(), footIndices_.end(), contact.getlocalBodyIndex()) == footIndices_.end()) {
            return true;
        }

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() { };

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  double terminalRewardCoeff_ = -10.;
  raisim::ArticulatedSystem* hound_;

  Eigen::VectorXd gc_, gv_;
  Eigen::Vector<double,19> gcInit_, gcDes_;
  Eigen::Vector<double,18> gvInit_, gvDes_;
  Eigen::Vector<double,12> pTarget_, prevTarget_, prevPrevTarget_;
  raisim::Mat<3,3> rot_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::vector<size_t> footIndices_;
  /// additional
  Eigen::Vector3d command_;                     // vx, vy, w
  std::vector<std::string> footFrames_;
  Eigen::Vector4i footContact_;
  std::vector<raisim::Vec<3>> footPos_,footVel_;
  double phase_;
  Eigen::Matrix<double,4,1> footContactDouble_; // gait
  Eigen::Matrix<double,4,1> footContactPhase_;  // gait hz
  Eigen::Matrix<double,4,1> footClearance_;     // foot clearance
  Eigen::Matrix<double,20,1> footToTerrain_; // 5 sample point for each foot
  bool standingMode_;
  /// log barrier function
  Eigen::Matrix<double,4,2> limitJointPos_;
  Eigen::Matrix<double,1,2> limitBodyHeight_;
  Eigen::Matrix<double,1,2> limitBaseMotion_; // y,z vel, roll,pitch vel
  Eigen::Matrix<double,1,2> limitJointVel_;
  Eigen::Matrix<double,1,2> limitTargetVel_;
  Eigen::Matrix<double,1,2> limitFootClearance_;
  Eigen::Matrix<double,1,2> limitFootContact_; // for gait enforcing
  ///
  std::vector<Eigen::Vector<double,12>> jointPosErrorHist_, jointVelHist_;




  thread_local static std::mt19937 gen_;
  thread_local static std::normal_distribution<double> normDist_;
  thread_local static std::uniform_real_distribution<double> uniDist_;

};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;
thread_local std::normal_distribution<double> raisim::ENVIRONMENT::normDist_(0.0,1.0);
thread_local std::uniform_real_distribution<double> raisim::ENVIRONMENT::uniDist_(-1.0,1.0);
}

