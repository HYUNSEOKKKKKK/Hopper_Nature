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
    dhal_ = world_->addArticulatedSystem(resourceDir_+"../hound/rsc/Hop_verSimple_ver20250114/Hop_verSimple.urdf");
    dhal_->setName("dhal");
    dhal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /// Dim
    gcDim_ = 10;
    gvDim_ = 9;
    numLegs_ = 1;
    numEdges_ = 4;
    actionDim_ = 3;
    obDim_ = 51;
    estDim_ = 7;
    valueObDim_ = obDim_ + estDim_;

    /// initialize
    gc_.setZero(gcDim_); gcInit_.setZero(); gcNoise_.setZero();
    gv_.setZero(gvDim_); gvInit_.setZero(); gvNoise_.setZero();
    gcDes_.setZero(); gvDes_.setZero();
    pTarget_.setZero(); prevTarget_.setZero(); prevPrevTarget_.setZero(); preJointVel_.setZero();
    jointFrictions_.setZero();

    /// this is nominal configuration of robot
    gcInit_.segment(0,7) << 0.0,0.0,0.71,   0.9847265,0.0,-0.1741081,0.0;
    gcInit_.tail(3) << 0.7, -0.35, 0.0;
    gcInit_.segment(3,4).normalize();
    gc_ = gcInit_;

    /// set pd gains
    jointPgain_.setZero(); jointPgain_.tail(actionDim_).setConstant(30.0);
    jointDgain_.setZero(); jointDgain_.tail(actionDim_).setConstant(1.0);
    dhal_->setPdGains(Eigen::VectorXd::Zero(gvDim_), Eigen::VectorXd::Zero(gvDim_));
    dhal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);
    valueObDouble_.setZero(valueObDim_);
            estDouble_.setZero(estDim_);

    /// action scaling
    actionMean_ = gcInit_.tail(actionDim_);
    actionStd_.setConstant(0.3);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.push_back(dhal_->getBodyIdx("Foot"));
    calfIndices_.push_back(dhal_->getBodyIdx("Calf"));
    footJointFrames_.push_back("02_ankle_roll_joint");  /// joint
//    hipJointFrames_.push_back("hip_abduction_left");

      bodyIndices_.push_back(dhal_->getBodyIdx("Thigh"));

       /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(dhal_);
      arrows_.push_back(server_->addVisualArrow("command_xy",0.1,0.05,0,1,0,1));
      arrows_.push_back(server_->addVisualArrow("command_yaw",0.1,0.05,1,0,0,1));
        for (int i=0; i<8; i++){
            visualEdge_.push_back(server_->addVisualSphere(std::to_string(i),0.01,0.0,1.0,0.0,1));
        }
    }
    visualizationOn_ = false;

    /// set limit for log barrier function
    limitJointPos_.row(0) << 0.0, 2.2; // knee
    limitJointPos_.row(1) << -0.873, 0.698; // ankle pitch [-50, 40]
//    limitJointPos_.row(2) << -0.5236, 0.5236; // ankle roll [-30,30] -> hardward limit
            limitJointPos_.row(2) << -0.4, 0.4; // ankle roll [-22,22] -> more conservative

    Eigen::Matrix<double,3,1> tempJointPos = limitJointPos_.col(1)-limitJointPos_.col(0);
    limitJointPos_.col(0) += tempJointPos*0.05;
    limitJointPos_.col(1) -= tempJointPos*0.05;

    limitBodyHeight_ << 0.30, 1.10;
    limitBaseMotion_.row(0) << -1.0,1.0; // z, pitch
      limitBaseMotion_.row(1) << -0.6,0.6; // roll
    limitJointVel_ << -6,6; // max vel limit is 9
    limitTargetVel_ << -0.6,0.6;
    limitFootContact_ << -0.3,2;
    limitFootClearance_ << -0.08,1.0; // 어차피 desired_foot_clearance 를

    /// initialize
    command_.setZero();
    footContact_ = 0;
    bodyContact_ = 0;
    footVel_.resize(numLegs_); footPos_.resize(numLegs_), hipJointPos_.resize(numLegs_), refBodyToFoot_.resize(numLegs_), footOrientation_.resize(numLegs_);
    footContactPhase_.setZero();
    footClearance_.setZero();
    footSlip_.setZero();
    standingMode_ = false;
    standingRegulation_ = 0.0;
    phaseSin_.setZero();
    footObsNoise_.setZero();
    standingSmoothness_ = 1.0;
    smoothnessWeight_.setZero(actionDim_);
    footPosWeight_.setZero();
            terminalStack_ = 0;

      /// initialize history
    jointPosErrorHist_ = std::vector<Eigen::VectorXd>(18,Eigen::VectorXd::Zero(actionDim_));
    jointVelHist_ = std::vector<Eigen::VectorXd>(18,Eigen::VectorXd::Zero(actionDim_));
    genForceTargetHist_ = std::vector<Eigen::VectorXd>(3,Eigen::VectorXd::Zero(gvDim_));  /// delay 는 2 ms 으로 설정 -> 아마 더 클 수 있음
    /// initialize gait
    phase_ = 0.0;
    gait_hz_ = 1.0;

    /// heightMap_ initialization
    heightMap_ = HeightMapSample(world_.get(),0,0.,gen_,uniDist_);
    curriculum_ = 0.0;
    iter_ = 0;
    mu_ = 0.7 + 0.3 * uniDist_(gen_);  // [0.4, 1.0]
    world_->setDefaultMaterial(mu_, 0, 0);

    /// initial body to foot pos
      dhal_->setState(gcInit_,gvInit_);
      for(int i = 0; i < numLegs_; i++) {
          dhal_->getFramePosition(footJointFrames_[i], footPos_[i]);
          refBodyToFoot_[i].e() =   footPos_[i].e() - gc_.head(3);
//          dhal_->getFramePosition(hipJointFrames_[i], hipJointPos_[i]);
//          refBodyToFoot_[i] =   footPos_[i] - hipJointPos_[i];
      }

      edgePosLocal_.col(0) << -0.12, -0.04, -0.015-0.065;
      edgePosLocal_.col(1) << -0.12, 0.04, -0.015-0.065;
      edgePosLocal_.col(2) << 0.12, -0.04, -0.015-0.065;
      edgePosLocal_.col(3) << 0.12, 0.04, -0.015-0.065;

      for (int i=0; i<4; i++){
          for (int j= 0; j<2; j++){
              sampleEdgePosLocal_.col(i*2+j) = edgePosLocal_.col(i);
          }
      }
      double sampling_point= 0.02;
      sampleEdgePosLocal_.col(0)(0) += -sampling_point;
      sampleEdgePosLocal_.col(1)(1) += -sampling_point;
      sampleEdgePosLocal_.col(2)(0) += -sampling_point;
      sampleEdgePosLocal_.col(3)(1) += sampling_point;
      sampleEdgePosLocal_.col(4)(0) += sampling_point;
      sampleEdgePosLocal_.col(5)(1) += -sampling_point;
      sampleEdgePosLocal_.col(6)(0) += sampling_point;
      sampleEdgePosLocal_.col(7)(1) += sampling_point;

      /// rot conversion to initial base pos
      rotConversion_ << 0.9393727,  0.0000000,  0.3428978, 0.0000000,  1.0000000,  0.0000000, -0.3428978,  0.0000000,  0.9393727; // 0.35 rad pitch rot

      /// joint regulating weight
      jointRegulatingWeight_.setZero(actionDim_);
      jointRegulatingWeight_ << 0.5, 2.0, 2.0; // knee, ankle pitch, ankle roll
  }

  void init() final { }

  void reset() final {
    jointPgain_.setZero(); jointPgain_.tail(actionDim_).setConstant(30.0 + 2.5*uniDist_(gen_));
    jointDgain_.setZero(); jointDgain_.tail(actionDim_).setConstant(1.0 + 0.1*uniDist_(gen_));
    /// foot obs noise
    for (int i=0;i<(3*numLegs_);i++){
      footObsNoise_(i) = 0.02 * uniDist_(gen_);
    }

    /// curriculum factor
    double comCurriculum = (double)iter_ * 1.0/1500; /// command curriculum
    comCurriculum = (comCurriculum > 1.0) ? 1.0 : comCurriculum; // [0,1.0]
            double initializeCurriculum = (double)iter_ * 1.0/1000; /// initialize curriculum
            initializeCurriculum = (initializeCurriculum > 1.0) ? 1.0 : initializeCurriculum;
//            double initializeCurriculum = 1.0; /// no curriculum

    /// with standing mode
    if (uniDist_(gen_) > 0.8) { // 10 %
        standingMode_ = true;
        command_.setZero();
    }else{
        standingMode_ = false;
        do {
            double maxCommand = 0.4 + comCurriculum * 0.8; // 평지 lin x max 1.5
            command_ << maxCommand * uniDist_(gen_), 0.6 * uniDist_(gen_), 0.6 * uniDist_(gen_);     // [lix x max, 0.6, 0.6]
            command_(0) = (command_(0) < -0.8) ? command_(0)+1.6 : command_(0);           // 뒤로가는 건 max -0.8
        } while (command_.norm() < 0.2);
    }

    mu_ = 0.7 + 0.3 * uniDist_(gen_);
    world_->setDefaultMaterial(mu_, 0, 0);

    /// initialize the pose
    bool reset = !standingMode_;
    if (standingMode_){ reset = uniDist_(gen_) > 0.0;}

    if(!reset){ /// command -> sudden stop
        gcNoise_ = gc_;
        gvNoise_ = gv_;
        gcNoise_.head(3) = gcInit_.head(3);
    }else{ /// reset
        /// initialize with noise
        gcNoise_ = gcInit_;
        /// rot noise
//        if (uniDist_(gen_)>0.2 and !standingMode_){ // 40 % -> 올라가는 거 고정
//            yawNoise_ = 3.141592/2.0;
//            command_.tail(2).setZero();
//            command_(0) = abs(command_(0));
//        }else{
        yawNoise_ = uniDist_(gen_) * 3.141592;
//        }
        rotYawNoise_ << cos(yawNoise_),-sin(yawNoise_),0,sin(yawNoise_),cos(yawNoise_),0,0,0,1;
        quat_.coeffs() << uniDist_(gen_)*0.1*initializeCurriculum, uniDist_(gen_)*0.1*initializeCurriculum, 0.0, 1.0; // xyz w
        quat_.normalize();
        rotTotalNoise_ = quat_;
        rotTotalNoise_ = rotTotalNoise_.eval() * rotYawNoise_ * rotConversion_.transpose();
        quat_ = rotTotalNoise_;
        quat_.normalize();
        gcNoise_.segment(3,4) << quat_.coeffs().w(), quat_.coeffs().head(3);
        for (int j=0; j<3; j++){
            gcNoise_(7+j) += uniDist_(gen_) * 0.3 * ((standingMode_)? 2.0 : 1.0);
        }
        /// Generalized Velocities randomization.
        gvNoise_.setZero();
        for (int i = 0; i < gvDim_; i++) {
            if (i < 3) {
                gvNoise_(i) = uniDist_(gen_) * 0.3 * initializeCurriculum;
            } else if (i < 6) {
                gvNoise_(i) = uniDist_(gen_) * 0.3 * initializeCurriculum;
            } else {
                gvNoise_(i) = uniDist_(gen_) * 1.0;
            }
            if (standingMode_) {gvNoise_(i) *= 2.0;}
        }
    }

    /// preventing foot penetration
    dhal_->setState(gcNoise_,gvNoise_);
    double heightShift = 1e3;  /// -> 공중에 있는 것도 데리고 옴
    double temp = 0.0;
    for (int i = 0; i < numLegs_; i++){
        dhal_->getFramePosition(footJointFrames_[i], footPos_[i]);
        dhal_->getFrameOrientation(footJointFrames_[i], footOrientation_[i]);
                edgePosWorld_ = footOrientation_[i].e() * edgePosLocal_;
                for (int j=0; j<numEdges_; j++){
                    edgePosWorld_.col(j) += footPos_[i].e();
                    temp = edgePosWorld_.col(j)(2) - 0.01 - heightMap_->getHeight(edgePosWorld_.col(j)(0), edgePosWorld_.col(j)(1)); /// but, 돌아가면 땅에 푹 파일겨
                    if (temp < heightShift){heightShift = temp;}
                }
    }
    gcNoise_(2) -= heightShift;
    dhal_->setState(gcNoise_, gvNoise_);
    updateObservation();

    for (auto& vec : genForceTargetHist_) { vec.setZero(); }
    /// reset (except the standingMode_ -> which preserves previous state for sudden command stop)
    if (reset){
        pTarget_ = gc_.tail(actionDim_);
        gcDes_.tail(actionDim_) = pTarget_; prevTarget_ = pTarget_; prevPrevTarget_ = pTarget_; preJointVel_.setZero();
        for (auto& vec : jointPosErrorHist_) { vec.setZero(); }
        for (auto& vec : jointVelHist_) { vec.setZero(); }

//        if (uniDist_(gen_)<=0.0){
//            phase_ = 0.0;
//        }else{
//            phase_ = gait_hz_/2.0;
//        }
                phase_ = 0.0 + uniDist_(gen_) * gait_hz_ * 0.3;
        footContactPhase_.setZero();
        footClearance_.setZero();
    }

    terminalStack_ = 0;
    /// random joint friction
//    for (int i=0;i<actionDim_;i++){
//        jointFrictions_(i) = 0.2 + 0.2 * uniDist_(gen_); // small friction
//    }
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    pTarget_ = action.cast<double>();
    pTarget_ = pTarget_.cwiseProduct(actionStd_);
    pTarget_ += actionMean_;                                   /// joint P target

    /// simulation
    double avgReward = 0.0;
    barrierReward_ = 0.0;
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
                /// compute target torque
        computeTorque();
        dhal_->setGeneralizedForce(genForceTargetHist_[0]); /// 2ms delay (torque command in PC -> actual torque in real robot)
                /// simpulation
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
      updateObservation();
      avgReward += getReward();
      barrierReward_+= getLogBarReward();

      if(visualizationOn_){
          visualizeCommand();
      }
    }

    avgReward /= (control_dt_ / simulation_dt_ + 1e-10);
    barrierReward_ /=(control_dt_ / simulation_dt_ + 1e-10);
    updateHistory();

    return avgReward;
  }

  void computeTorque(){
        genForceTargetHist_.erase(genForceTargetHist_.begin());
        Eigen::VectorXd tempGenForce(gvDim_); tempGenForce.head(6).setZero();
        tempGenForce.tail(actionDim_) = jointPgain_.tail(actionDim_).cwiseProduct(pTarget_-gc_.tail(actionDim_))
                              + jointDgain_.tail(actionDim_).cwiseProduct(-gv_.tail(actionDim_));

        /// joint friction (static friction, torque 잡아먹는 효과)
//        for (int i = 0; i < actionDim_; i++){
//          double jTorque = tempGenForce.tail(actionDim_)(i);
//          jTorque = (jTorque>0) ? std::min(jointFrictions_(i), jTorque) : std::max(-jointFrictions_(i), jTorque);
//          tempGenForce.tail(actionDim_)(i) -= jTorque;
//        }
        genForceTargetHist_.push_back(tempGenForce);
  }

  float getBarrierReward() final {
            return barrierReward_;
  }

  void updateHistory(){
      prevPrevTarget_ = prevTarget_;
      prevTarget_ = pTarget_;

      jointVelHist_.erase(jointVelHist_.begin());
      jointVelHist_.push_back(gv_.tail(actionDim_));

      jointPosErrorHist_.erase(jointPosErrorHist_.begin());
      jointPosErrorHist_.push_back(pTarget_ - gc_.tail(actionDim_));
  }

  double getReward(){
      standingReward(); /// there is order
      rewards_.record("negSumPos",getNegPosReward());
      return rewards_.getReward("negSumPos");
  }

  void standingReward(){
      /// for standingMode
      if (!standingMode_){
          limitBaseMotion_.row(0) << -1.0,1.0;
          limitBaseMotion_.row(1) << -0.6,0.6;
          standingSmoothness_ = 1.0;
          footPosWeight_ << 0.6,1.0,0.4;
      } else {
          limitBaseMotion_.row(0) << -0.4,0.4;
          limitBaseMotion_.row(1) << -0.2,0.2;
          standingSmoothness_ = 2.0;
          footPosWeight_ << 1.0,1.0,1.0;
      }
  }

  float getNegPosReward(){
      /// pos reward
      rewards_.record("comAngularVel", std::exp(-5.0 * pow(command_(2) - bodyAngularVel_(2),2)));
      rewards_.record("comLinearVel", std::exp(-5.0 * (command_.head(2) - bodyLinearVel_.head(2)).squaredNorm()));

      /// neg reward
      footSlip_.setZero();
      for (int i=0; i<numLegs_; i++){
          if (footContact_ > 1){
              footSlip_(i) = footVel_[i].e().head(2).squaredNorm();
          }
      }

      rewards_.record("footSlip", footSlip_.sum());
      if (rot_(8)>1.0){ rot_(8) = 1.0; } /// preventing acos nan
      rewards_.record("bodyOri", std::acos(rot_(8)) * std::acos(rot_(8)));
      rewards_.record("smoothness2", (pTarget_ - 2 * prevTarget_ + prevPrevTarget_).squaredNorm()  * standingSmoothness_);
      rewards_.record("baseMotion", 0.2*pow(bodyLinearVel_(2),2) + 0.2*abs(bodyAngularVel_(0)) + 0.2*abs(bodyAngularVel_(1)));

      /// task space foot pos regulation -> not used
      Eigen::Vector3d  tempVec;
      double tempReward = 0.0;
      for(int index_leg = 0; index_leg < numLegs_; index_leg++){
          tempVec = (footPos_[index_leg].e() - gc_.head(3));
          tempVec = rot_.e().transpose() *  tempVec.eval();
          tempReward += footPosWeight_.cwiseProduct(tempVec-refBodyToFoot_[index_leg].e()).squaredNorm();
      }
      /// pos vel acc regulation
      jointRegulatingWeight_ << 0.5, 1.0, 1.0; // knee, ankle pitch, ankle roll
      rewards_.record("jointPos", (jointRegulatingWeight_.cwiseProduct(gc_.tail(actionDim_)-gcInit_.tail(actionDim_))).squaredNorm());
            jointRegulatingWeight_ << 0.5, 2.0, 2.0; // knee, ankle pitch, ankle roll
      rewards_.record("jointVel", (jointRegulatingWeight_.cwiseProduct(gv_.tail(actionDim_))).squaredNorm());
      rewards_.record("jointAcc", (jointRegulatingWeight_.cwiseProduct((gv_.tail(actionDim_) - preJointVel_))).squaredNorm());
      jointRegulatingWeight_ << 0.5, 1.0, 1.0; // knee, ankle pitch, ankle roll
      rewards_.record("torque", (jointRegulatingWeight_.cwiseProduct(dhal_->getGeneralizedForce().e().tail(actionDim_))).squaredNorm());
/// body contact reward
      rewards_.record("bodyContact",(double) bodyContact_);

      /// sum
      float posReward, negReward;
      posReward = (float)(rewards_.getReward("comAngularVel") + rewards_.getReward("comLinearVel"));
      negReward = (float)(rewards_.getReward("bodyOri") + rewards_.getReward("jointPos") + rewards_.getReward("footPos") + rewards_.getReward("jointVel") + rewards_.getReward("jointAcc") + rewards_.getReward("torque")
              + rewards_.getReward("footSlip") + rewards_.getReward("smoothness2") + rewards_.getReward("bodyContact") + rewards_.getReward("baseMotion"));
      rewards_.record("negReward2", negReward); /// only for recording

      return (float)(std::exp(0.2 * negReward) * posReward);
  }

  float getLogBarReward(){
      /// for gait enforcing & foot clearance
      phase_ += simulation_dt_;
      footContactPhase_(0) = sin(phase_/gait_hz_ * 2*3.141592); // left
//      footContactPhase_(1) = -footContactPhase_(0); // right

      phaseSin_(0) = sin(phase_/gait_hz_ * 2*3.141592); // for observation
      phaseSin_(1) = cos(phase_/gait_hz_ * 2*3.141592); // for observation
//
      if (!standingMode_){ /// walking
          /// footContactDouble_ -> limit_foot_contact 에 있도록 (-0.3,3) -> Gait Enforcing (요 -0.3 이 벗어나도 되는 범위)
          for(int i=0; i<numLegs_; i++) {
              if (footContact_ > 1) { footContactDouble_(i) = 1.0 * footContactPhase_(i); }
              else { footContactDouble_(i) = -1.0 * footContactPhase_(i); }
          }
          /// footClearance_ -> limit_foot_clearance 에 있도록 (-0.12,0.12) -> foot 드는 거 enforcing
          double desiredFootZPosition = 0.20;
          for (int i=0; i<numLegs_; i++){
              if (footContactPhase_(i) < -0.6) { /// during swing, 전체시간의 33 %
                  footClearance_(i) =
                          footToTerrain_.segment(i * 8, 8).minCoeff() - desiredFootZPosition; // 대략, 0.17 sec, 0 보다 크거나 같으면 됨 (enforcing clearance)
              }else{ footClearance_(i) = 0.0; } // max reward (not enforcing clearance)
          }
      } else { /// under standingMode_
          /// standingMode_ 는 zero command 로 부터 유추 가능, command 는 obs 이기 때문에, robot 은 standingMode_인지 아닌지 충분히 알 수 있음
          for (int i=0; i<numLegs_; i++){
              footContactDouble_(i) = 1.0; // around max reward, where this value should go under (-0.3,3)
              footClearance_(i) = 0.0; // max reward (not enforcing clearance)
          }
      }

      /// compute barrier reward
      double barrierJointPos = 0.0, barrierBodyHeight = 0.0, barrierBaseMotion = 0.0, barrierJointVel = 0.0, barrierTargetVel = 0.0, barrierFootContact = 0.0, barrierFootClearance = 0.0;
      double tempReward = 0.0;
      /// Log Barrier - limit_joint_pos
      for (int index_joint=0;index_joint<actionDim_;index_joint++){
          relaxedLogBarrier(0.08,limitJointPos_(index_joint,0),limitJointPos_(index_joint,1),gc_(7+index_joint),tempReward);
          barrierJointPos += tempReward;
      }
      /// Log Barrier - limit_body_height
      double tempHeight = 0.0;
      for (int i=0; i<numLegs_; i++){
          tempHeight += gc_(2) - heightMap_->getHeight(footPos_[i](0), footPos_[i](1));
      }
      tempHeight /= static_cast<double>(numLegs_);
      relaxedLogBarrier(0.04,limitBodyHeight_(0),limitBodyHeight_(1),tempHeight,barrierBodyHeight);

      /// Log Barrier - limit_base_motion
      relaxedLogBarrier(1.0,limitBaseMotion_(0,0),limitBaseMotion_(0,1),bodyLinearVel_(2),tempReward);
      barrierBaseMotion += tempReward;
              relaxedLogBarrier(1.0,limitBaseMotion_(0,0),limitBaseMotion_(0,1),bodyAngularVel_(0),tempReward);
              barrierBaseMotion += tempReward;
              relaxedLogBarrier(0.6,limitBaseMotion_(1,0),limitBaseMotion_(1,1),bodyAngularVel_(1),tempReward);
              barrierBaseMotion += tempReward;
      /// Log Barrier - limit_joint_vel
      for (int i=0;i<actionDim_;i++){
          relaxedLogBarrier(2.0,limitJointVel_(0),limitJointVel_(1),gv_(6+i),tempReward);
          barrierJointVel += tempReward;
      }
      /// Log Barrier - limit_target_vel
      relaxedLogBarrier(0.6,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(0)-command_(0),tempReward);
      barrierTargetVel += tempReward;
      relaxedLogBarrier(0.6,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(1)-command_(1),tempReward);
      barrierTargetVel += tempReward;
      relaxedLogBarrier(0.6,limitTargetVel_(0),limitTargetVel_(1),bodyAngularVel_(2)-command_(2),tempReward);
      barrierTargetVel += tempReward;
      /// Log Barrier - limit_foot_contact
      for (int i=0;i<numLegs_;i++){
          relaxedLogBarrier(0.08,limitFootContact_(0),limitFootContact_(1),footContactDouble_(i),tempReward);
          barrierFootContact += tempReward;
      }
      /// Log Barrier - limit_foot_clearance
      for (int i=0;i<numLegs_;i++){
          relaxedLogBarrier(0.01,limitFootClearance_(0),limitFootClearance_(1),footClearance_(i),tempReward);
          barrierFootClearance += tempReward;
      }

      double logClip = -500.0;
      barrierJointPos = fmax(barrierJointPos,logClip);           /// 여기 밖 부분은 gradient 안 받겠다
      rewards_.record("barrierJointPos", barrierJointPos);
      rewards_.record("barrierBodyHeight", barrierBodyHeight);
      rewards_.record("barrierBaseMotion", barrierBaseMotion);
      rewards_.record("barrierJointVel", barrierJointVel);
      rewards_.record("barrierTargetVel", barrierTargetVel);
      rewards_.record("barrierFootContact", barrierFootContact);
      rewards_.record("barrierFootClearance", barrierFootClearance);


      float logBarReward =  (float)(1e-1*(barrierJointPos + barrierBodyHeight + barrierBaseMotion + barrierJointVel + barrierTargetVel + barrierFootContact + barrierFootClearance));
          rewards_.record("relaxedLog", logBarReward); /// relaxed log barrier
      return  logBarReward;
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
    preJointVel_ = gv_.tail(actionDim_);
    /// update state
    dhal_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot_);
    rot_.e() = rot_.e() * rotConversion_; // R_wb * R_bc
    bodyLinearVel_ = rot_.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot_.e().transpose() * gv_.segment(3, 3);
    for(int i = 0; i < numLegs_; i++) {
      dhal_->getFramePosition(footJointFrames_[i], footPos_[i]);
      dhal_->getFrameVelocity(footJointFrames_[i], footVel_[i]);
                dhal_->getFrameOrientation(footJointFrames_[i], footOrientation_[i]);
                /// currently, works for one leg
        edgePosWorld_ = footOrientation_[i].e() * edgePosLocal_;
        for (int j=0; j<numEdges_; j++) {
            edgePosWorld_.col(j) += footPos_[i].e();
        }
//      dhal_->getFramePosition(hipJointFrames_[i], hipJointPos_[i]);
    }

    /// foot contact update
    footContact_ = 0;
    for(auto& contact: dhal_->getContacts()){
        for (size_t i=0; i<numLegs_; i++){
            if(contact.getlocalBodyIndex() == footIndices_[i]){
                footContact_ += 1;
            }
        }
    }

            /// body contact update (only used for true state)
            bodyContact_ = 0;
      for(auto& contact: dhal_->getContacts()){
          for (size_t i=0; i<1; i++){
              if(contact.getlocalBodyIndex() == bodyIndices_[i]){
                  bodyContact_ += 1;
              }
          }
      }

    /// update foot terrain
    updateFootToTerrain();
  }

  void updateFootToTerrain(){
            /// one leg setting (0-th leg)
      sampleEdgePosWorld_ = footOrientation_[0].e() * sampleEdgePosLocal_;
      for (int j=0; j<8; j++){
          sampleEdgePosWorld_.col(j) += footPos_[0].e();
          footToTerrain_(j) = sampleEdgePosWorld_.col(j)(2) - heightMap_->getHeight(sampleEdgePosWorld_.col(j)(0),sampleEdgePosWorld_.col(j)(1));
      }

    if (abs(footToTerrain_.minCoeff()) >  3.0){ /// print error !!!
        std::cout << "gc_[2] : " << gc_[2] << std::endl;
        std::cout << "height map : " << heightMap_->getHeight(sampleEdgePosWorld_.col(0)(0), sampleEdgePosWorld_.col(0)(1)) <<  std::endl;
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
      rot_robot = rot_.e();

      rot_pitch_90 << 0,0,1,0,1,0,-1,0,0;
      theta_command = -atan2(command(1),command(0));
      rot_command << 1,0,0,0,cos(theta_command),-sin(theta_command),0,sin(theta_command),cos(theta_command);

      arrow_pos_offset << 0,0,0.20;
      arrow_pos_offset = rot_robot * arrow_pos_offset.eval();
      quaternion = rot_robot.eval() * rot_pitch_90 * rot_command;

      arrows_[0]->setCylinderSize(0.2,command.head(2).norm()*0.3);
      arrows_[0]->setPosition(gc_head_7.head(3) + arrow_pos_offset);
      arrows_[0]->setOrientation(quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z());

      arrows_[1]->setCylinderSize(0.2,command(2)*0.3);
      arrows_[1]->setPosition(gc_head_7.head(3) + arrow_pos_offset);
      quaternion = rot_robot.eval();
//      arrows_[1]->setOrientation(gc_head_7.segment(3,4));
      arrows_[1]->setOrientation(quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z());

            /// visualize foot edges
                        for (int j=0; j<8; j++){
                            visualEdge_[j] ->setPosition(sampleEdgePosWorld_.col(j));
                        }
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
      if (standingMode_){
//          footContactPhase_.setZero();
          phaseSin_.setZero();
      }
      obDouble_ << rot_.e().row(2).transpose(),                                     /// body orientation. 3
          bodyAngularVel_,                                                      /// body angular velocity. 3
          gc_.tail(actionDim_),                                                      /// joint pos 3
          gv_.tail(actionDim_),                                                      /// joint velocity 3

          prevTarget_ - actionMean_,                                                          /// previous action 3
          prevPrevTarget_ - actionMean_,                                                      /// preprevious action 3
          jointPosErrorHist_[0], jointPosErrorHist_[6], jointPosErrorHist_[12], /// joint History 9 (0.18, 0.12, 0.6)
          jointVelHist_[0], jointVelHist_[6], jointVelHist_[12],                /// joint History 9 (0.18, 0.12, 0.6)
          rot_.e().transpose() * (footPos_[0].e() - gc_.head(3)),               /// relative foot position with respect to the body COM, expressed in the body frame 3
                    rot_.e().transpose() * ((edgePosWorld_.col(0)+edgePosWorld_.col(1))/2.0 - gc_.head(3)),
              rot_.e().transpose() * ((edgePosWorld_.col(2)+edgePosWorld_.col(3))/2.0 - gc_.head(3)),/// relative edge pos (heel, toe)
          command_,                                                             /// command 3
          phaseSin_, /// phase encoding 2
          static_cast<double>(standingMode_);  /// standingMode 1

      double noise = 0.0;
//      for (int i=0; i<obDim_; i++){
//          if (i<3)       {noise = 0.03;}  /// body orientation
//          else if(i<6)   {noise = 0.1;}   /// body angular velocity (rad/sec)
//          else if(i<28)  {noise = 0.05;}  /// joint pos             (rad)
//          else if(i<50)  {noise = 0.5;}   /// joint vel             (rad/sec)
//          else if(i<94)  {noise = 0.0;}  /// action related
//          else if(i<160) {noise = 0.0;}   /// action related
//          else if(i<226) {noise = 0.1;}  /// vel history
//          else if(i<232) {noise = footObsNoise_(i-226);} /// relative foot pos (2 cm)
//          else           {noise = 0.0;}
//
//          obDouble_(i) += uniDist_(gen_) * noise;
//      }

    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  void valueObserve(Eigen::Ref<EigenVec> ob) final { /// obs + (true) estimated_state
      if (standingMode_){
          phaseSin_.setZero();
      }
      valueObDouble_ << rot_.e().row(2).transpose(),                               /// body orientation. 3
              bodyAngularVel_,                                                      /// body angular velocity. 3
              gc_.tail(actionDim_),                                                      /// joint pos 3
              gv_.tail(actionDim_),                                                      /// joint velocity 3

              prevTarget_- actionMean_,                                                          /// previous action 3
              prevPrevTarget_- actionMean_,                                                      /// preprevious action 3
              jointPosErrorHist_[0], jointPosErrorHist_[6], jointPosErrorHist_[12], /// joint History 9 (0.18, 0.12, 0.6)
              jointVelHist_[0], jointVelHist_[6], jointVelHist_[12],                /// joint History 9 (0.18, 0.12, 0.6)
              rot_.e().transpose() * (footPos_[0].e() - gc_.head(3)),        /// relative foot position with respect to the body COM, expressed in the body frame 3
              rot_.e().transpose() * ((edgePosWorld_.col(0)+edgePosWorld_.col(1))/2.0 - gc_.head(3)),
              rot_.e().transpose() * ((edgePosWorld_.col(2)+edgePosWorld_.col(3))/2.0 - gc_.head(3)),/// relative edge pos (heel, toe)
              command_,                                                             /// command 3
              phaseSin_, /// phase sin cos 2
              static_cast<double>(standingMode_),                                   /// standingMode 1

              bodyLinearVel_,                                                       /// body linear velocity. 3
//              footClearance_ * 4.0,                                                       /// min foot z
              (footToTerrain_(0) + footToTerrain_(2)) * 2.0,
              (footToTerrain_(4) + footToTerrain_(6)) * 2.0,                         /// heel & toe height
              static_cast<double>(footContact_)/4.0,                                /// 1 foot contact num
              static_cast<double>(bodyContact_)/4.0;                                /// 1 body contact num

      /// convert it to float
      ob = valueObDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);
    /// if the contact body is not feet
    for(auto& contact: dhal_->getContacts())
        if ((std::find(footIndices_.begin(), footIndices_.end(), contact.getlocalBodyIndex()) == footIndices_.end())
                and (std::find(calfIndices_.begin(), calfIndices_.end(), contact.getlocalBodyIndex()) == calfIndices_.end())) {
            terminalStack_ += 1;
        }
    if (terminalStack_ > 50){
        return true;
    }
    terminalReward = -0.f;
    return false;
  }

  void curriculumUpdate() {
      /// for each iteration
      iter_ ++;
      curriculum_ = 0.0;
      world_->removeObject(heightMap_);
      heightMap_ = HeightMapSample(world_.get(),0,curriculum_,gen_,uniDist_);
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
              /// preventing foot penetration
              dhal_->setState(gcNoise_,gvNoise_);
              double heightShift = 1e3;  /// -> 공중에 있는 것도 데리고 옴
              double temp = 0.0;
              for (int i = 0; i < numLegs_; i++){
                  dhal_->getFramePosition(footJointFrames_[i], footPos_[i]);
                  dhal_->getFrameOrientation(footJointFrames_[i], footOrientation_[i]);
                  edgePosWorld_ = footOrientation_[i].e() * edgePosLocal_;
                  for (int j=0; j<numEdges_; j++){
                      edgePosWorld_.col(j) += footPos_[i].e();
                      temp = edgePosWorld_.col(j)(2) - 0.01 - heightMap_->getHeight(edgePosWorld_.col(j)(0), edgePosWorld_.col(j)(1)); /// but, 돌아가면 땅에 푹 파일겨
                      if (temp < heightShift){heightShift = temp;}
                  }
              }
          gcNoise_(2) -= heightShift;

          /// reset
          dhal_->setState(gcNoise_, gvInit_);
          updateObservation();
      }
  }

 private:
  int gcDim_, gvDim_, numLegs_, numEdges_;
  bool visualizable_ = false;
  double terminalRewardCoeff_ = -10.0;
  raisim::ArticulatedSystem* dhal_;

  Eigen::VectorXd gc_, gv_;
  Eigen::Vector<double,10> gcInit_, gcNoise_, gcDes_;
  Eigen::Vector<double,9> gvInit_, gvNoise_, gvDes_;
  Eigen::Vector<double,3> pTarget_, prevTarget_, prevPrevTarget_, preJointVel_, jointFrictions_;
  Eigen::Vector<double,3> jointPgain_, jointDgain_;
    Eigen::VectorXd jointRegulatingWeight_;

  Eigen::Matrix<double,3,3> rotConversion_;
  raisim::Mat<3,3> rot_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_, valueObDouble_, estDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::vector<size_t> footIndices_, calfIndices_;
    /// collision reward
    std::vector<size_t> bodyIndices_;

  /// additional
  Eigen::Vector3d command_;                     // vx, vy, w
  std::vector<std::string> footJointFrames_;
  std::vector<std::string> hipJointFrames_;
  int footContact_;
    int bodyContact_; // 1
    std::vector<raisim::Vec<3>> footPos_,footVel_, hipJointPos_, refBodyToFoot_;
        std::vector <raisim::Mat<3,3>> footOrientation_;
  double phase_;
  double gait_hz_;
  Eigen::Matrix<double,1,1> footContactDouble_; // gait (numLegs_,1)
  Eigen::Matrix<double,1,1> footContactPhase_;  // gait hz (numLegs_,1)
  Eigen::Matrix<double,1,1> footClearance_;     // foot clearance (numLegs_,1)
  Eigen::Matrix<double,1,1> footSlip_;     // foot clearance (numLegs_,1)
  Eigen::Matrix<double,8,1> footToTerrain_; // 8 sample point for each foot (numLegs_ * 8,1)
  Eigen::Matrix<double,2,1> phaseSin_;  // sin cos representation of phase
  Eigen::Matrix<double,6,1> footObsNoise_;
        Eigen::Matrix<double,3,4> edgePosLocal_, edgePosWorld_; // sqaure foot eade -> (currently) for one foot, 4 for edges
        Eigen::Matrix<double,3,8> sampleEdgePosLocal_, sampleEdgePosWorld_; // sqaure foot eade -> (currently) for one foot, 8 for sampling points
  bool standingMode_;
  double standingRegulation_;
  Eigen::VectorXd smoothnessWeight_;
  Eigen::Vector3d footPosWeight_;

  /// log barrier function
  Eigen::Matrix<double,3,2> limitJointPos_;
  Eigen::Matrix<double,1,2> limitBodyHeight_;
  Eigen::Matrix<double,2,2> limitBaseMotion_; // z vel, roll,pitch vel
  Eigen::Matrix<double,1,2> limitJointVel_;
  Eigen::Matrix<double,1,2> limitTargetVel_;
  Eigen::Matrix<double,1,2> limitFootClearance_;
  Eigen::Matrix<double,1,2> limitFootContact_; // for gait enforcing
  ///
  std::vector<Eigen::VectorXd> jointPosErrorHist_, jointVelHist_;
  std::vector<Eigen::VectorXd> genForceTargetHist_;
  double standingSmoothness_;

  /// initialize
  Eigen::Matrix<double,3,3> rotYawNoise_,rotTotalNoise_;
  Eigen::Quaterniond quat_;
  double yawNoise_;
  ///
  std::vector<raisim::Visuals*> arrows_;
    std::vector<raisim::Visuals*> visualEdge_;
    raisim::HeightMap* heightMap_;
  /// curriculum
  double curriculum_;
  int iter_;
  double mu_;
  /// for barrier
  float barrierReward_;
        int terminalStack_;

  thread_local static std::mt19937 gen_;
  thread_local static std::normal_distribution<double> normDist_;
  thread_local static std::uniform_real_distribution<double> uniDist_;

};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;
thread_local std::normal_distribution<double> raisim::ENVIRONMENT::normDist_(0.0,1.0);
thread_local std::uniform_real_distribution<double> raisim::ENVIRONMENT::uniDist_(-1.0,1.0);
}

