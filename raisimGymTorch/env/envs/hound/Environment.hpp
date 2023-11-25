//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"
#include "HeightMap.hpp"

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
    gc_.setZero(19); gcInit_.setZero(); gcNoise_.setZero();
    gv_.setZero(18); gvInit_.setZero(); gvNoise_.setZero();
    gcDes_.setZero(); gvDes_.setZero();
    pTarget_.setZero(); prevTarget_.setZero(); prevPrevTarget_.setZero(); preJointVel_.setZero();

    /// this is nominal configuration of anymal
    double hip = 0.62;
    gcInit_ << 0, 0, 0.58-0.002, 1.0, 0.0, 0.0, 0.0, 0.0, hip, -2*hip, 0.0, hip, -2*hip, 0.0, hip, -2*hip, 0.0, hip, -2*hip;
//    double hip = 0.7854;
//    gcInit_ << 0, 0, 0.51875, 1.0, 0.0, 0.0, 0.0, 0.0, hip, -2*hip, 0.0, hip, -2*hip, 0.0, hip, -2*hip, 0.0, hip, -2*hip;
    gcInit_.segment(3,4).normalize();
    gc_ = gcInit_;

    /// set pd gains
    Eigen::Vector<double,18> jointPgain, jointDgain;
    jointPgain.setZero(); jointPgain.tail(12).setConstant(50.0);
    jointDgain.setZero(); jointDgain.tail(12).setConstant(1.0);
    hound_->setPdGains(jointPgain, jointDgain);
    hound_->setGeneralizedForce(Eigen::VectorXd::Zero(18));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 144;
    valueObDim_ = 171;
    actionDim_ = 12;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);
    valueObDouble_.setZero(valueObDim_);

    /// action scaling
    actionMean_ = gcInit_.tail(12);
    for (int i=0; i<4; i++){
//      actionStd_.segment(i*3,3) << 0.1, 0.2, 0.2;
      actionStd_.segment(i*3,3) << 0.1, 0.15, 0.2;
    }

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
      arrows_.push_back(server_->addVisualArrow("command_xy",0.1,0.05,0,1,0,1));
      arrows_.push_back(server_->addVisualArrow("command_yaw",0.1,0.05,1,0,0,1));
    }
    visualizationOn_ = false;

    /// set limit for log barrier function
    for (int i=0;i<4;i++){
        limitJointPos_.row(i*3+0) << -0.523599,0.523599; // roll : (-pi/6, pi/6)
//        limitJointPos_.row(i*3+1) << 0,1.570796; // hip : 0, pi*1/2
        limitJointPos_.row(i*3+1) << hip-0.785398,hip+0.785398; // hip : 0, pi*1/2
//        limitJointPos_.row(i*3+1) << 0,1.45; // hip
//        limitJointPos_.row(i*3+2) << -2.6179933,-0.5235987; // knee : -pi*5/6, -pi/6
        limitJointPos_.row(i*3+2) << -2*hip-1.04720-0.20,-2*hip+1.04720-0.20; // knee : -pi*5/6, -pi/6
//        limitJointPos_.row(i*3+2) << -2.0943946,-0.5235987; // knee : -pi*5/6, -pi/6
    }
//    limitBodyHeight_ << 0.48, 0.62; // -> 52,66
//    limitBodyHeight_ << 0.52, 0.66;
    limitBodyHeight_ << 0.52, 0.68;
    limitBaseMotion_.row(0) << -0.3,0.3;
    limitBaseMotion_.row(1) << -0.5,0.5;
    limitJointVel_ << -8,8;
    limitTargetVel_ << -0.2,0.2;
    limitFootContact_ << -0.3,2;
//    limitFootClearance_ << -0.10,0.10; // 어차피 desired_foot_clearance 를
    limitFootClearance_ << -0.12,0.12; // 어차피 desired_foot_clearance 를
    limitFootSlip_    << -1.0, 1.0;
    limitBodyOri_     << -0.3, 0.3;
    limitSmoothness1_ << -0.2, 0.2;
    limitSmoothness2_ << -1.0, 1.0;

    /// initialize
    command_.setZero();
    footContact_.setZero();
    footVel_.resize(4); footPos_.resize(4);
    footContactPhase_.setZero();
    footClearance_.setZero();
    footSlip_.setZero();
    standingMode_ = false;

    /// initialize history
    jointPosErrorHist_ = std::vector<Eigen::Vector<double,12>>(18,Eigen::Vector<double,12>::Zero());
    jointVelHist_ = std::vector<Eigen::Vector<double,12>>(18,Eigen::Vector<double,12>::Zero());

    /// initialize gait
    phase_ = 0.0;
    gait_hz_ = 0.68;

    /// heightMap_ initialization
    heightMap_ = HeightMapSample(world_.get(),0,0.,gen_,uniDist_);
    curriculum_ = 0.0;
    iter_ = 0;
    mu_ = 0.7 + 0.3 * uniDist_(gen_);  // [0.4, 1.0]
    world_->setDefaultMaterial(mu_, 0, 0);
  }

  void init() final { }

  void reset() final {
    double comCurriculum = (double)iter_ * 1.0/3600;
    comCurriculum = (comCurriculum > 1.0) ? 1.0 : comCurriculum;
    if (iter_%4==0){
//      command_ << (1.0+comCurriculum) * uniDist_(gen_), 0.6 * uniDist_(gen_), 0.6 * uniDist_(gen_); // [2.0, 0.6, 0.6]
      command_ << (1.0+comCurriculum*1.2) * uniDist_(gen_), 0.6 * uniDist_(gen_), 0.6 * uniDist_(gen_); // [2.2, 0.6, 0.6]
    }else{
      command_ << (1.0+comCurriculum*0.5) * uniDist_(gen_), 0.6 * uniDist_(gen_), 0.6 * uniDist_(gen_); // [1.5, 0.6, 0.6]
    }
    if (command_(0) < -1.0){
      command_(0) = abs(command_(0));
    }

    /// with standing mode
    if (uniDist_(gen_) > 0.8) {
        standingMode_ = true;      // 10 %
        command_.setZero();
    }else{
        standingMode_ = false;
    }

    mu_ = 0.7 + 0.3 * uniDist_(gen_);
    world_->setDefaultMaterial(mu_, 0, 0);

//    double keepState = uniDist_(gen_);
    double keepState = -1.0;

    if (keepState >= 0.5){
        /// keep current state
        gcNoise_.head(3) = gcInit_.head(3); /// prevent falling case due to sudden low height map
        gcNoise_.tail(16) = gc_.tail(16);
        gvNoise_ = gv_;
    }else{
        /// initialize with noise
        gcNoise_ = gcInit_;
        /// rot noise
        yawNoise_ = uniDist_(gen_) * 3.141592;
//        yawNoise_ = 3.141592/2.0;
        rotYawNoise_ << cos(yawNoise_),-sin(yawNoise_),0,sin(yawNoise_),cos(yawNoise_),0,0,0,1;
        quat_.coeffs() << uniDist_(gen_)*0.2, uniDist_(gen_)*0.2, 0.0, 1.0; // xyz w
        quat_.normalize();
        rotTotalNoise_ = quat_;
        rotTotalNoise_ = rotTotalNoise_.eval() * rotYawNoise_;
        quat_ = rotTotalNoise_;
        quat_.normalize();
        gcNoise_.segment(3,4) << quat_.coeffs().w(), quat_.coeffs().head(3);
        /// joint noise
        for (int i = 7; i < 19; i++){
            gcNoise_(i) += uniDist_(gen_) * 0.2;
        }
        /// Generalized Velocities randomization.
        gvNoise_.setZero();
        for (int i = 0; i < 18; i++) {
            if (i < 3) {
                gvNoise_(i) = uniDist_(gen_) * 0.5;
            } else if (i < 6) {
                gvNoise_(i) = uniDist_(gen_) * 0.5;
//                gvNoise_(i) = uniDist_(gen_) * 0.8;
            } else {
                gvNoise_(i) = uniDist_(gen_) * 2.0;
//                gvNoise_(i) = uniDist_(gen_) * 2.5;
            }
        }
    }

    /// preventing foot penetration
    hound_->setState(gcNoise_,gvNoise_);
    double heightShift = 1e3;  /// -> 공중에 있는 것도 데리고 옴
//    double heightShift = 0.0; /// -> penetration 만 compensation (중간에 발산함)
    double temp = 0.0;
    for (int i = 0; i < 4; i++){
        hound_->getFramePosition(footFrames_[i], footPos_[i]);
//        temp = footPos_[i](2) - 0.025 - heightMap_->getHeight(footPos_[i](0), footPos_[i](1));
        temp = footPos_[i](2) - heightMap_->getHeight(footPos_[i](0), footPos_[i](1));
        if (temp < heightShift){heightShift = temp;}
    }
    gcNoise_(2) -= heightShift;
    hound_->setState(gcNoise_, gvNoise_);
    updateObservation();

    if (keepState < 0.5){
        pTarget_ = gc_.tail(12);
        gcDes_.tail(12) = pTarget_; prevTarget_ = pTarget_; prevPrevTarget_ = pTarget_; preJointVel_.setZero();
        for (auto& vec : jointPosErrorHist_) { vec.setZero(); }
        for (auto& vec : jointVelHist_) { vec.setZero(); }

        phase_ = 0.0;
        footContactPhase_.setZero();
    }
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// delay
    int delayIdx = int((0.002 / simulation_dt_ + 1e-10)); // 2ms delay

    /// simulation
    double avgReward = 0.0;
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      if (i == delayIdx){
          /// action scaling
          pTarget_ = action.cast<double>();
          pTarget_ = pTarget_.cwiseProduct(actionStd_);
          pTarget_ += actionMean_;                                   /// joint P target
          gcDes_.tail(12) = pTarget_;
          hound_->setPdTarget(gcDes_, gvDes_);
      }
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
      updateObservation();
      avgReward += getReward();

      if(visualizationOn_){
          visualizeCommand();
      }
    }

    avgReward /= (control_dt_ / simulation_dt_ + 1e-10);
    updateHistory();

    return avgReward;
  }

  void updateHistory(){
      prevPrevTarget_ = prevTarget_;
      prevTarget_ = pTarget_;

      jointVelHist_.erase(jointVelHist_.begin());
      jointVelHist_.push_back(gv_.tail(12));

      jointPosErrorHist_.erase(jointPosErrorHist_.begin());
      jointPosErrorHist_.push_back(pTarget_ - gc_.tail(12));
  }

  double getReward(){
      /// for gait enforcing & foot clearance
      phase_ += simulation_dt_;
      footContactPhase_(0) = sin(phase_/gait_hz_ * 2*3.141592); // RR
      footContactPhase_(1) = -footContactPhase_(0); // RL
      footContactPhase_(2) = -footContactPhase_(0); // FR
      footContactPhase_(3) = footContactPhase_(0); // FL
//
      if (!standingMode_){ /// walking
          /// footContactDouble_ -> limit_foot_contact 에 있도록 (-0.3,3) -> Gait Enforcing (요 -0.3 이 벗어나도 되는 범위)
          for(int i=0; i<4; i++) {
              if (footContact_(i)) { footContactDouble_(i) = 1.0 * footContactPhase_(i); }
              else { footContactDouble_(i) = -1.0 * footContactPhase_(i); }
          }
          /// footClearance_ -> limit_foot_clearance 에 있도록 (-0.12,0.12) -> foot 드는 거 enforcing
          double desiredFootZPosition = 0.15;
          for (int i=0; i<4; i++){
              if (footContactPhase_(i) < -0.5) { /// during swing
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
      footSlip_.setZero();
      for (int i=0; i<4; i++){
          if (footContact_(i)){
                  footSlip_(i) = footVel_[i].e().head(2).squaredNorm();
          }
      }

      /// (exp(neg1)+exp(neg2))/2.0 * (exp(pos1) + exp(pos2)) + relaxedLogBarrier + standingNegReward
      rewards_.record("negSumPos",getNegPosReward());
      rewards_.record("standingNegReward", getStandingReward());
      rewards_.record("relaxedLog", getLogBarReward()); /// relaxed log barrier
      return rewards_.sumModified();
  }

  float getStandingReward(){  /// for standingMode_
      Eigen::VectorXd jointPosTemp(12), jointPosWeight(12), jointVelTemp(12),jointAccTemp(12);
//      jointPosWeight << 2.0, 0.,0.,2.,0.,0.,2.,0.,0.,2.,0.,0.;
      jointPosWeight << 2.0, 0.5,0.5,2.,0.5,0.5,2.,0.5,0.5,2.,0.5,0.5;
      if (!standingMode_){
          jointPosTemp.setZero();
          jointVelTemp.setZero();
          jointAccTemp.setZero();
          limitBaseMotion_.row(0) << -0.3,0.3;
          limitBaseMotion_.row(1) << -0.5,0.5;
      } else {
          jointPosTemp = gc_.tail(12)-gcInit_.tail(12);
          jointVelTemp = gv_.tail(12);
          jointAccTemp = gv_.tail(12) - preJointVel_;
          limitBaseMotion_.row(0) << -0.1,0.1;
          limitBaseMotion_.row(1) << -0.3,0.3;
      }
      rewards_.record("standingJointPos", jointPosTemp.squaredNorm());
      rewards_.record("standingJointVel", jointVelTemp.squaredNorm());
      rewards_.record("standingJointAcc", jointAccTemp.squaredNorm());

      float tempReward;
//      if (!standingMode_){
//          return 0.0;
//      }else{
          tempReward = rewards_.getReward("standingJointPos") + rewards_.getReward("standingJointVel") + rewards_.getReward("standingJointAcc");
          return (float)(std::exp(0.2 * tempReward)); /// same weight with positive reward
//      }
  }

  float getNegPosReward(){
      Eigen::Vector3d tempCommand;
      tempCommand.setZero(); tempCommand(2) = command_(2);
      rewards_.record("comAngularVel", std::exp(-1.0 * (tempCommand - bodyAngularVel_).squaredNorm())); // regulation 같이
      rewards_.record("comLinearVel", std::exp(-1.0 * (command_.head(2) - bodyLinearVel_.head(2)).squaredNorm()));

      Eigen::VectorXd jointPosTemp(12), jointPosWeight(12);
      jointPosWeight << 2.0, 0.,0.,2.,0.,0.,2.,0.,0.,2.,0.,0.;
      jointPosTemp = gc_.tail(12) - gcInit_.tail(12);
      jointPosTemp = jointPosWeight.cwiseProduct(jointPosTemp.eval());

      rewards_.record("footSlip", footSlip_.sum());
      rewards_.record("bodyOri", std::acos(rot_(8)) * std::acos(rot_(8)));
      rewards_.record("smoothness1",(pTarget_ - prevTarget_).squaredNorm());
      rewards_.record("smoothness2", (pTarget_ - 2 * prevTarget_ + prevPrevTarget_).squaredNorm());
      rewards_.record("jointPos", jointPosTemp.squaredNorm());
      rewards_.record("pTarget", (pTarget_-actionMean_).squaredNorm());
      rewards_.record("torque", hound_->getGeneralizedForce().squaredNorm());

      float posReward, negReward;
      posReward = (float)(rewards_.getReward("comAngularVel") + rewards_.getReward("comLinearVel"));
      negReward = (float)(rewards_.getReward("pTarget") + rewards_.getReward("jointPos") + rewards_.getReward("torque") + rewards_.getReward("footSlip") + rewards_.getReward("bodyOri") + rewards_.getReward("smoothness1") + rewards_.getReward("smoothness2"));
      rewards_.record("negReward2", negReward);

      return (float)(std::exp(0.2 * negReward) * posReward);
  }

  float getLogBarReward(){
      double barrierJointPos = 0.0, barrierBodyHeight = 0.0, barrierBaseMotion = 0.0, barrierJointVel = 0.0, barrierTargetVel = 0.0, barrierFootContact = 0.0, barrierFootClearance = 0.0,
              barrierFootSlip = 0.0, barrierSmoothness1 = 0.0, barrierSmoothness2 = 0.0, barrierBodyOri = 0.0;
      double tempReward = 0.0;
      // Log Barrier - limit_joint_pos
      for (int i=0;i<4;i++){
          for (int j=0;j<3;j++){
              int index_leg = i*3+j;
              relaxedLogBarrier(0.09,limitJointPos_(index_leg,0),limitJointPos_(index_leg,1),gc_(7+index_leg),tempReward);
              barrierJointPos += tempReward;
          }
      }
      // Log Barrier - limit_body_height
      double tempHeight = 0.0;
      for (int i=0; i<4; i++){
          tempHeight += gc_(2) - heightMap_->getHeight(footPos_[i](0), footPos_[i](1));
      }
      tempHeight /= 4;
//      tempHeight = gc_(2);
      relaxedLogBarrier(0.05,limitBodyHeight_(0),limitBodyHeight_(1),tempHeight,barrierBodyHeight);
      // Log Barrier - limit_base_motion
      relaxedLogBarrier(0.2,limitBaseMotion_(0,0),limitBaseMotion_(0,1),bodyLinearVel_(2),tempReward);
      barrierBaseMotion += tempReward;
      for (int i=0;i<2;i++){
          relaxedLogBarrier(0.3,limitBaseMotion_(1,0),limitBaseMotion_(1,1),bodyAngularVel_(i),tempReward);
          barrierBaseMotion += tempReward;
      }
      // Log Barrier - limit_joint_vel
      for (int i=0;i<12;i++){
          relaxedLogBarrier(3.0,limitJointVel_(0),limitJointVel_(1),gv_(6+i),tempReward);
          barrierJointVel += tempReward;
      }
      // Log Barrier - limit_target_vel
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(0)-command_(0),tempReward);
      barrierTargetVel += tempReward;
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(1)-command_(1),tempReward);
      barrierTargetVel += tempReward;
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyAngularVel_(2)-command_(2),tempReward);
      barrierTargetVel += tempReward;
      // Log Barrier - limit_foot_contact
      for (int i=0;i<4;i++){
          relaxedLogBarrier(0.1,limitFootContact_(0),limitFootContact_(1),footContactDouble_(i),tempReward);
          barrierFootContact += tempReward;
      }
      // Log Barrier - limit_foot_clearance
      for (int i=0;i<4;i++){
          relaxedLogBarrier(0.03,limitFootClearance_(0),limitFootClearance_(1),footClearance_(i),tempReward);
          barrierFootClearance += tempReward;
      }

      // Log Barrier - limit_foot_slip
//      for(int i = 0; i < 4; i++) {
////          if (footContact_(i)) { // contact 이면
////              relaxedLogBarrier(0.4,limitFootSlip_(0),limitFootSlip_(1),footVel_[i].e().head(2).norm(),tempReward);
////              barrierFootSlip += tempReward;
////          }
//          relaxedLogBarrier(0.5,limitFootSlip_(0),limitFootSlip_(1),footSlip_(i),tempReward);
//          barrierFootSlip += tempReward;
//      }
      // Log Barrier - limit_body_ori
//      relaxedLogBarrier(0.1,limitBodyOri_(0),limitBodyOri_(1),1-rot_(8),tempReward);
//      barrierBodyOri += tempReward;
//      // Log Barrier - limit_smoothness1
//      for (int i=0; i<12; i++){
//          relaxedLogBarrier(0.1,limitSmoothness1_(0),limitSmoothness1_(1),pTarget_(i) - prevTarget_(i),tempReward);
//          barrierSmoothness1 += tempReward;
//      }
//      // Log Barrier - limit_smoothness2
//      for (int i=0; i<12; i++){
//          relaxedLogBarrier(0.2,limitSmoothness2_(0),limitSmoothness2_(1),pTarget_(i) - 2 * prevTarget_(i) + prevPrevTarget_(i),tempReward);
//          barrierSmoothness2 += tempReward;
//      }

      double logClip = -100.0; // -100.0
      barrierJointPos = fmax(barrierJointPos,logClip);           /// 여기 밖 부분은 gradient 안 받겠다
      barrierBodyHeight = fmax(barrierBodyHeight,logClip);
      barrierBaseMotion = fmax(barrierBaseMotion,logClip);
      barrierJointVel = fmax(barrierJointVel,logClip);
      barrierTargetVel = fmax(barrierTargetVel,logClip);
      barrierFootContact = fmax(barrierFootContact,logClip);
      barrierFootClearance = fmax(barrierFootClearance,logClip);
      rewards_.record("barrierJointPos", barrierJointPos);
      rewards_.record("barrierBodyHeight", barrierBodyHeight);
      rewards_.record("barrierBaseMotion", barrierBaseMotion);
      rewards_.record("barrierJointVel", barrierJointVel);
      rewards_.record("barrierTargetVel", barrierTargetVel);
      rewards_.record("barrierFootContact", barrierFootContact);
      rewards_.record("barrierFootClearance", barrierFootClearance);
      return (float)(1e-1*(barrierJointPos + barrierBodyHeight + barrierBaseMotion + barrierJointVel + barrierTargetVel + barrierFootContact + barrierFootClearance));
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
    /// update previous footVel
    preJointVel_ = gv_.tail(12);
    /// update state
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

    /// foot contact update
    footContact_.setZero();
    for(auto& contact: hound_->getContacts()){
        for (size_t i=0; i<4; i++){
            if(contact.getlocalBodyIndex() == footIndices_[i]){
                footContact_(i) = 1;
            }
        }
    }

    /// update foot terrain
    updateFootToTerrain();
  }

  void updateFootToTerrain(){
    Eigen::Matrix<double, 3, 5> sample_point;
    double point = 0.05;
    sample_point.col(0) << point, 0.0, 0.0;
    sample_point.col(1) << 0.0, point, 0.0;
    sample_point.col(2) << -point, 0.0, 0.0;
    sample_point.col(3) << 0.0, -point, 0.0;
    sample_point.col(4).setZero();
    for (int i = 0; i < 4; i++) {
        sample_point.col(i) = rot_.e().transpose() * sample_point.col(i).eval();
    }
    Eigen::Matrix<double, 5, 1> temp_foot;
    Eigen::Matrix<double, 3, 1> temp3;
    for (int k = 0; k < 4; k++) {
        for (int i = 0; i < 5; i++) {
            temp3 = footPos_[k].e() + sample_point.col(i);
            footToTerrain_(5 * k + i) = footPos_[k].e()(2) - heightMap_->getHeight(temp3(0), temp3(1));
        }
    }
    if (abs(footToTerrain_.minCoeff()) >  3.0){ /// print error !!!
        std::cout << "gc_[2] : " << gc_[2] << std::endl;
        std::cout << "height map : " << heightMap_->getHeight(temp3(0), temp3(1)) <<  std::endl;
        std::cout << "Error too big here : " << footToTerrain_.transpose() << std::endl;
    }
  }

  void visualizeCommand(){
      Eigen::Matrix<double,3,3> rot_robot, rot_pitch_90, rot_command;
      Eigen::Quaterniond quaternion;
      Eigen::Vector3d command;
      Eigen::VectorXd gc_head_7(7);
      double theta_command;
      Eigen::Matrix<double,3,1> arrow_pos_offset;

      command = command_;
      gc_head_7 = gc_.head(7);

      quaternion.coeffs() << gc_head_7.tail(3),gc_head_7(3);
      rot_robot = quaternion;

      rot_pitch_90 << 0,0,1,0,1,0,-1,0,0;
      theta_command = -atan2(command(1),command(0));
      rot_command << 1,0,0,0,cos(theta_command),-sin(theta_command),0,sin(theta_command),cos(theta_command);

      arrow_pos_offset << 0,0,0.15;
      arrow_pos_offset = rot_robot * arrow_pos_offset.eval();
      quaternion = rot_robot.eval() * rot_pitch_90 * rot_command;

      arrows_[0]->setCylinderSize(0.2,command.head(2).norm()*0.3);
      arrows_[0]->setPosition(gc_head_7.head(3) + arrow_pos_offset);
      arrows_[0]->setOrientation(quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z());

      arrows_[1]->setCylinderSize(0.2,command(2)*0.3);
      arrows_[1]->setPosition(gc_head_7.head(3) + arrow_pos_offset);
      arrows_[1]->setOrientation(gc_head_7.segment(3,4));
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
      if (standingMode_){
          footContactPhase_.setZero();
      }
      obDouble_ << rot_.e().row(2).transpose(),                               /// body orientation. 3
          bodyAngularVel_,                                                      /// body angular velocity. 3
          gc_.tail(12),                                                      /// joint pos 12
          gv_.tail(12),                                                      /// joint velocity 12

          prevTarget_,                                                          /// previous action 12
          prevPrevTarget_,                                                      /// preprevious action 12
          jointPosErrorHist_[0], jointPosErrorHist_[6], jointPosErrorHist_[12], /// joint History 36 (0.18, 0.12, 0.6)
          jointVelHist_[0], jointVelHist_[6], jointVelHist_[12],                /// joint History 36 (0.18, 0.12, 0.6)
          rot_.e().transpose() * (footPos_[0].e() - gc_.head(3)), rot_.e().transpose() * (footPos_[1].e() - gc_.head(3)),
          rot_.e().transpose() * (footPos_[2].e() - gc_.head(3)), rot_.e().transpose() * (footPos_[3].e() - gc_.head(3)),
          /// relative foot position with respect to the body COM, expressed in the body frame 12
          command_,                                                             /// command 3
          footContactPhase_.head(2), /// footContactPhase 2
          static_cast<double>(standingMode_);                                   /// standingMode 1

      double noise = 0.0;
      for (int i=0; i<obDim_; i++){
          if (i<3)       {noise = 0.03;}  /// body orientation
          else if(i<6)   {noise = 0.1;}   /// body angular velocity (rad/sec)
          else if(i<18)  {noise = 0.05;}  /// joint pos             (rad)
          else if(i<30)  {noise = 0.5;}   /// joint vel             (rad/sec)
          else if(i<54)  {noise = 0.01;}  /// action related
          else if(i<90)  {noise = 0.0;}   /// action related
          else if(i<126) {noise = 0.1;}  /// vel history
          else if(i<138) {noise = 0.02;} /// relative foot pos (2 cm)
          else           {noise = 0.0;}

          obDouble_(i) += uniDist_(gen_) * noise;
      }

    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  void valueObserve(Eigen::Ref<EigenVec> ob) final {
      if (standingMode_){
          footContactPhase_.setZero();
      }
      valueObDouble_ << rot_.e().row(2).transpose(),                               /// body orientation. 3
              bodyAngularVel_,                                                      /// body angular velocity. 3
              gc_.tail(12),                                                      /// joint pos 12
              gv_.tail(12),                                                      /// joint velocity 12

              prevTarget_,                                                          /// previous action 12
              prevPrevTarget_,                                                      /// preprevious action 12
              jointPosErrorHist_[0], jointPosErrorHist_[6], jointPosErrorHist_[12], /// joint History 36 (0.18, 0.12, 0.6)
              jointVelHist_[0], jointVelHist_[6], jointVelHist_[12],                /// joint History 36 (0.18, 0.12, 0.6)
              rot_.e().transpose() * (footPos_[0].e() - gc_.head(3)), rot_.e().transpose() * (footPos_[1].e() - gc_.head(3)),
              rot_.e().transpose() * (footPos_[2].e() - gc_.head(3)), rot_.e().transpose() * (footPos_[3].e() - gc_.head(3)),
              /// relative foot position with respect to the body COM, expressed in the body frame 12
              command_,                                                             /// command 3
              footContactPhase_.head(2), /// footContactPhase 2
              static_cast<double>(standingMode_),                                   /// standingMode 1

              bodyLinearVel_,                                                       /// body linear velocity. 3
              footToTerrain_,                                                       /// foot z position 20 (5 sample * 4 foot)
              footContact_.cast<double>();

      /// convert it to float
      ob = valueObDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    if (iter_>4200 and (iter_%4==2 or iter_%4==3)){
        if ((pTarget_-actionMean_).squaredNorm() > 1e2)   {return true;}
    }else{
        for(auto& contact: hound_->getContacts())
            if (std::find(footIndices_.begin(), footIndices_.end(), contact.getlocalBodyIndex()) == footIndices_.end()) {
                return true;
            }
    }

    for (int i=0; i<4; i++){
      if (gc_(9+i*3)>-0.1)  {return true;}
    }
    if (rewards_.getReward("smoothness1") < -5e2) {return true;}

    terminalReward = -0.f;
    return false;
  }

  void curriculumUpdate() {
      /// for each iteration
      iter_ ++;
      curriculum_ = (double)iter_ * (1.0/1800.0); /// 1600 iter -> 1.0
      curriculum_ = (curriculum_ > 3.0) ? 3.0 : curriculum_;

      world_->removeObject(heightMap_);
      heightMap_ = HeightMapSample(world_.get(),iter_%4,curriculum_,gen_,uniDist_);
//      heightMap_ = HeightMapSample(world_.get(),2,curriculum_,gen_,uniDist_);
  }

  void setSeed(int seed) {gen_.seed(seed);}


  /// for tester.py
  void setCommand(Eigen::Vector3d command){
    command_ = command;
    if (command_.norm()<0.2){
      standingMode_ = true;
      command_.setZero();
    }else{
      standingMode_ = false;
    }
  }
  void setTerrain(int type, double curriculum, double mu){
      world_->removeObject(heightMap_);
      heightMap_ = HeightMapSample(world_.get(),type,curriculum,gen_,uniDist_);
      world_->setDefaultMaterial(mu, 0, 0);
  }
  void setInitial(int type){
      if (type == 0){
          hound_->setState(gcInit_,gvInit_);
          double heightShift = 1e2, temp = 0.0;
          for (int i = 0; i < 4; i++){
              hound_->getFramePosition(footFrames_[i], footPos_[i]);
              temp = footPos_[i](2) - heightMap_->getHeight(footPos_[i](0), footPos_[i](1));
              if (temp < heightShift){heightShift = temp;}
          }
          gcNoise_(2) -= heightShift;

          /// reset
          hound_->setState(gcNoise_, gvInit_);
          updateObservation();
      }
  }

 private:
  int gcDim_, gvDim_;
  bool visualizable_ = false;
  double terminalRewardCoeff_ = -10.0;
  raisim::ArticulatedSystem* hound_;

  Eigen::VectorXd gc_, gv_;
  Eigen::Vector<double,19> gcInit_, gcNoise_, gcDes_;
  Eigen::Vector<double,18> gvInit_, gvNoise_, gvDes_;
  Eigen::Vector<double,12> pTarget_, prevTarget_, prevPrevTarget_, preJointVel_;
  raisim::Mat<3,3> rot_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_, valueObDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::vector<size_t> footIndices_;
  /// additional
  Eigen::Vector3d command_;                     // vx, vy, w
  std::vector<std::string> footFrames_;
  Eigen::Vector4i footContact_;
  std::vector<raisim::Vec<3>> footPos_,footVel_;
  double phase_;
  double gait_hz_;
  Eigen::Matrix<double,4,1> footContactDouble_; // gait
  Eigen::Matrix<double,4,1> footContactPhase_;  // gait hz
  Eigen::Matrix<double,4,1> footClearance_;     // foot clearance
  Eigen::Matrix<double,4,1> footSlip_;     // foot clearance
  Eigen::Matrix<double,20,1> footToTerrain_; // 5 sample point for each foot
  bool standingMode_;
  /// log barrier function
  Eigen::Matrix<double,12,2> limitJointPos_;
  Eigen::Matrix<double,1,2> limitBodyHeight_;
  Eigen::Matrix<double,2,2> limitBaseMotion_; // z vel, roll,pitch vel
  Eigen::Matrix<double,1,2> limitJointVel_;
  Eigen::Matrix<double,1,2> limitTargetVel_;
  Eigen::Matrix<double,1,2> limitFootClearance_;
  Eigen::Matrix<double,1,2> limitFootContact_; // for gait enforcing
  Eigen::Matrix<double,1,2> limitFootSlip_;
  Eigen::Matrix<double,1,2> limitBodyOri_;
  Eigen::Matrix<double,1,2> limitSmoothness1_;
  Eigen::Matrix<double,1,2> limitSmoothness2_;
//  Eigen::Matrix<double,1,2> limitOtherContact_;
  ///
  std::vector<Eigen::Vector<double,12>> jointPosErrorHist_, jointVelHist_;
  /// initialize
  Eigen::Matrix<double,3,3> rotYawNoise_,rotTotalNoise_;
  Eigen::Quaterniond quat_;
  double yawNoise_;
  ///
  std::vector<raisim::Visuals*> arrows_;
  raisim::HeightMap* heightMap_;
  /// curriculum
  double curriculum_;
  int iter_;
  double mu_;

  thread_local static std::mt19937 gen_;
  thread_local static std::normal_distribution<double> normDist_;
  thread_local static std::uniform_real_distribution<double> uniDist_;

};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;
thread_local std::normal_distribution<double> raisim::ENVIRONMENT::normDist_(0.0,1.0);
thread_local std::uniform_real_distribution<double> raisim::ENVIRONMENT::uniDist_(-1.0,1.0);
}

