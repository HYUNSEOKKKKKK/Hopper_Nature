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
    digit_ = world_->addArticulatedSystem(resourceDir_+"../hound/rsc/digit/urdf/digit_model_revised7DOFleg_collision.urdf");
    digit_->setName("digit");
    digit_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /// Dim
    gcDim_ = 29;
    gvDim_ = 28;
    numLegs_ = 2;
    actionDim_ = 22;
    obDim_ = 238;
    estDim_ = 7;
    valueObDim_ = obDim_ + estDim_;

    /// initialize
    gc_.setZero(gcDim_); gcInit_.setZero(); gcNoise_.setZero();
    gv_.setZero(gvDim_); gvInit_.setZero(); gvNoise_.setZero();
    gcDes_.setZero(); gvDes_.setZero();
    pTarget_.setZero(); prevTarget_.setZero(); prevPrevTarget_.setZero(); preJointVel_.setZero();
    jointFrictions_.setZero();

    /// this is nominal configuration of anymal
    gcInit_.segment(0,7) << 0.0,0.0,0.942,   1.0,0.0,0.0,0.0;
    gcInit_.segment(7,7) << 0.337,0.0,   0.0,0.1,-0.1,   -0.126,-0.05;
    gcInit_.segment(14,4) << 0.0,0.863,0.0,0.0;
    gcInit_.segment(18,11) = -gcInit_.segment(7,11);
    gcInit_.segment(3,4).normalize();
    gc_ = gcInit_;

    /// set pd gains
    jointPgain_.setZero(); jointPgain_.tail(actionDim_).setConstant(30.0);
    jointDgain_.setZero(); jointDgain_.tail(actionDim_).setConstant(1.0);
    digit_->setPdGains(Eigen::VectorXd::Zero(gvDim_), Eigen::VectorXd::Zero(gvDim_));
    digit_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

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
    footIndices_.push_back(digit_->getBodyIdx("left_toe_roll"));
    footIndices_.push_back(digit_->getBodyIdx("right_toe_roll"));
    tarsusIndices_.push_back(digit_->getBodyIdx("left_tarsus"));
    tarsusIndices_.push_back(digit_->getBodyIdx("right_tarsus"));
    footJointFrames_.push_back("toe_roll_joint_left");  /// joint
    footJointFrames_.push_back("toe_roll_joint_right");
    hipJointFrames_.push_back("hip_abduction_left");
    hipJointFrames_.push_back("hip_abduction_right");

       /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(digit_);
      arrows_.push_back(server_->addVisualArrow("command_xy",0.1,0.05,0,1,0,1));
      arrows_.push_back(server_->addVisualArrow("command_yaw",0.1,0.05,1,0,0,1));
    }
    visualizationOn_ = false;

    /// set limit for log barrier function
    limitJointPos_.row(0) << -1.0472, 1.0472; // hip_abduction_left
    limitJointPos_.row(1) << -0.6981, 0.6981; // hip_rotation_left
    limitJointPos_.row(2) << -1.0472, 1.5707; // hip_flexion_left
    limitJointPos_.row(3) << -1.2392, 0.8727; // knee_joint_left
    limitJointPos_.row(4) << -0.8779, 1.2497; // shin_to_tarsus_left
    limitJointPos_.row(5) << -0.7853, 0.7853; // toe_pitch_joint_left
    limitJointPos_.row(6) << -0.6109, 0.6109; // toe_roll_joint_left

//    limitJointPos_.row(7) << -1.309, 0.4; // shoulder_roll_joint_left
            limitJointPos_.row(7) << -0.8, 0.4; // shoulder_roll_joint_left
//    limitJointPos_.row(8) << -2.5307, 2.5307; // shoulder_pitch_joint_left
//            limitJointPos_.row(8) << -0.237, 1.863; // shoulder_pitch_joint_left
            limitJointPos_.row(8) << -0.237, 1.263; // shoulder_pitch_joint_left
    limitJointPos_.row(9) << -1.7453, 1.7453; // shoulder_yaw_joint_left
    limitJointPos_.row(10) << -1.3526, 1.3526; // elbow_joint_left

    Eigen::Matrix<double,11,1> tempJointPos = limitJointPos_.topRows(11).col(1)-limitJointPos_.topRows(11).col(0);
    limitJointPos_.topRows(11).col(0) += tempJointPos*0.05;
    limitJointPos_.topRows(11).col(1) -= tempJointPos*0.05;
    limitJointPos_.bottomRows(11).col(0) = -limitJointPos_.topRows(11).col(1);
    limitJointPos_.bottomRows(11).col(1) = -limitJointPos_.topRows(11).col(0);
//    std::cout << "limit joint pos for left \n" << limitJointPos_.topRows(11) << std::endl;
//    std::cout << "limit joint pos for right \n" << limitJointPos_.bottomRows(11) << std::endl;

    limitBodyHeight_ << 0.8, 1.0;
    limitBaseMotion_ << -0.3,0.3;
    limitJointVel_ << -8,8;
    limitTargetVel_ << -0.4,0.4;
    limitFootContact_ << -0.6,2;
    limitFootClearance_ << -0.08,1.0; // 어차피 desired_foot_clearance 를
      limitArmLegCoupling_ << -0.4, 0.4;

    /// initialize
    command_.setZero();
    footContact_.setZero();
    footVel_.resize(numLegs_); footPos_.resize(numLegs_), hipJointPos_.resize(numLegs_), refBodyToFoot_.resize(numLegs_);
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
      armLegCoupling_.setZero(); // left hip flexion == right shoulder pitch

    /// initialize history
    jointPosErrorHist_ = std::vector<Eigen::VectorXd>(18,Eigen::VectorXd::Zero(actionDim_));
    jointVelHist_ = std::vector<Eigen::VectorXd>(18,Eigen::VectorXd::Zero(actionDim_));
    genForceTargetHist_ = std::vector<Eigen::VectorXd>(3,Eigen::VectorXd::Zero(gvDim_));  /// delay 는 2 ms 으로 설정 -> 아마 더 클 수 있음
    /// initialize gait
    phase_ = 0.0;
    gait_hz_ = 0.82;

    /// heightMap_ initialization
    heightMap_ = HeightMapSample(world_.get(),0,0.,gen_,uniDist_);
    curriculum_ = 0.0;
    iter_ = 0;
    mu_ = 0.7 + 0.3 * uniDist_(gen_);  // [0.4, 1.0]
    world_->setDefaultMaterial(mu_, 0, 0);

    /// initial body to foot pos
      digit_->setState(gcInit_,gvInit_);
      for(int i = 0; i < numLegs_; i++) {
          digit_->getFramePosition(footJointFrames_[i], footPos_[i]);
          digit_->getFramePosition(hipJointFrames_[i], hipJointPos_[i]);
          refBodyToFoot_[i] =   footPos_[i] - hipJointPos_[i];
      }
  }

  void init() final { }

  void reset() final {
    jointPgain_.setZero(); jointPgain_.tail(actionDim_).setConstant(30.0 + 2.5*uniDist_(gen_));
    jointDgain_.setZero(); jointDgain_.tail(actionDim_).setConstant(1.0 + 0.1*uniDist_(gen_));
//    digit_->setPdGains(jointPgain_, jointDgain_);
    /// foot obs noise
    for (int i=0;i<(3*numLegs_);i++){
      footObsNoise_(i) = 0.02 * uniDist_(gen_);
    }



    /// curriculum factor
    double comCurriculum = (double)iter_ * 1.0/3000; /// command curriculum
    comCurriculum = (comCurriculum > 1.0) ? 1.0 : comCurriculum; // [0,1.0]
            double initializeCurriculum = (double)iter_ * 1.0/4000; /// initialize curriculum
            initializeCurriculum = (initializeCurriculum > 1.0) ? 1.0 : initializeCurriculum;
//            double initializeCurriculum = 1.0; /// no curriculum

    /// with standing mode
    if (uniDist_(gen_) > 0.8) { // 10 %
        standingMode_ = true;
        command_.setZero();
    }else{
        standingMode_ = false;
        do {
//            double maxCommand = (iter_ % 4 == 0) ? (0.8 + comCurriculum * 1.2) : (1.0 + comCurriculum * 0.5); // 평지 lin x max 2.0, other 1.5
            double maxCommand = 0.8 + comCurriculum * 1.2; // 평지 lin x max 2.0, other 1.5
            command_ << maxCommand * uniDist_(gen_), 0.6 * uniDist_(gen_), 0.6 * uniDist_(gen_);     // [lix x max, 0.6, 0.6]
            command_(0) = (command_(0) < -1.0) ? command_(0)+1.2 : command_(0);           // 뒤로가는 건 max -1.0
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
        rotTotalNoise_ = rotTotalNoise_.eval() * rotYawNoise_;
        quat_ = rotTotalNoise_;
        quat_.normalize();
        gcNoise_.segment(3,4) << quat_.coeffs().w(), quat_.coeffs().head(3);
        /// joint noise
        for (int j =0; j<2; j++){
            for (int i = 0; i < 11; i++){
                if (i!=4){ /// four bar linkage coupling issue
                    gcNoise_(7 + 11*j + i) += uniDist_(gen_) * 0.3 * ((standingMode_)? 2.0 : 1.0);
                }else{
                    gcNoise_(7 + 11*j + i) = -gcNoise_(7 + 11*j + i - 1);
                }
            }
        }
        /// Generalized Velocities randomization.
        gvNoise_.setZero();
        for (int i = 0; i < gvDim_; i++) {
            if (i < 3) {
                gvNoise_(i) = uniDist_(gen_) * 0.5 * initializeCurriculum;
            } else if (i < 6) {
                gvNoise_(i) = uniDist_(gen_) * 0.5 * initializeCurriculum;
            } else {
                gvNoise_(i) = uniDist_(gen_) * 1.5;
            }
            if (standingMode_) {gvNoise_(i) *= 2.0;}
        }
        gvNoise_(7 + 4) = -gvNoise_(7 + 3);          /// four bar linkage
        gvNoise_(7 + 11 + 4) = -gvNoise_(7 + 11+ 3); /// four bar linkage
    }

    /// preventing foot penetration
    digit_->setState(gcNoise_,gvNoise_);
    double heightShift = 1e3;  /// -> 공중에 있는 것도 데리고 옴
    double temp = 0.0;
    for (int i = 0; i < numLegs_; i++){
        digit_->getFramePosition(footJointFrames_[i], footPos_[i]);
        temp = footPos_[i](2) - 0.065 - heightMap_->getHeight(footPos_[i](0), footPos_[i](1)); /// but, 돌아가면 땅에 푹 파일겨
        if (temp < heightShift){heightShift = temp;}
    }
    gcNoise_(2) -= heightShift;
    digit_->setState(gcNoise_, gvNoise_);
    updateObservation();

    for (auto& vec : genForceTargetHist_) { vec.setZero(); }
    /// reset (except the standingMode_ -> which preserves previous state for sudden command stop)
    if (reset){
        pTarget_ = gc_.tail(actionDim_);
        gcDes_.tail(actionDim_) = pTarget_; prevTarget_ = pTarget_; prevPrevTarget_ = pTarget_; preJointVel_.setZero();
        for (auto& vec : jointPosErrorHist_) { vec.setZero(); }
        for (auto& vec : jointVelHist_) { vec.setZero(); }

        if (uniDist_(gen_)<=0.0){
            phase_ = 0.0;
        }else{
            phase_ = gait_hz_/2.0;
        }
        footContactPhase_.setZero();
        footClearance_.setZero();
    }

    /// random joint friction
    for (int i=0;i<actionDim_;i++){
        jointFrictions_(i) = 0.2 + 0.2 * uniDist_(gen_); // small friction
    }
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
        digit_->setGeneralizedForce(genForceTargetHist_[0]); /// 2ms delay (torque command in PC -> actual torque in real robot)
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
        for (int i = 0; i < actionDim_; i++){
          double jTorque = tempGenForce.tail(actionDim_)(i);
          jTorque = (jTorque>0) ? std::min(jointFrictions_(i), jTorque) : std::max(-jointFrictions_(i), jTorque);
          tempGenForce.tail(actionDim_)(i) -= jTorque;
        }
        //            std::cout << "afterenForce : " << tempGenForce.transpose() << std::endl;
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
          limitBaseMotion_ << -0.3,0.3;
          standingSmoothness_ = 1.0;
//          smoothnessWeight_ << 1.0, 0.6,0.6,1.,0.6,0.6,1.,0.6,0.6,1.,0.6,0.6;
          footPosWeight_ << 0.6,1.0,0.4;
      } else {
          limitBaseMotion_ << -0.1,0.1;
          standingSmoothness_ = 2.0;
//          smoothnessWeight_ << 1.0, 0.8,0.8,1.,0.8,0.8,1.,0.8,0.8,1.,0.8,0.8;
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
          if (footContact_(i) > 1){
              footSlip_(i) = footVel_[i].e().head(2).squaredNorm();
          }
      }

      rewards_.record("footSlip", footSlip_.sum());
      if (rot_(8)>1.0){ rot_(8) = 1.0; } /// preventing acos nan
      rewards_.record("bodyOri", std::acos(rot_(8)) * std::acos(rot_(8)));
      rewards_.record("smoothness2", (pTarget_ - 2 * prevTarget_ + prevPrevTarget_).squaredNorm()  * standingSmoothness_);
      rewards_.record("torque", digit_->getGeneralizedForce().squaredNorm());

      /// task space foot pos regulation -> used
      Eigen::Vector3d  tempVec;
      double tempReward = 0.0;
      for(int index_leg = 0; index_leg < numLegs_; index_leg++){
          tempVec = (footPos_[index_leg].e() - hipJointPos_[index_leg].e());
          tempVec = rot_.e().transpose() *  tempVec.eval();
          tempReward += footPosWeight_.cwiseProduct(tempVec-refBodyToFoot_[index_leg].e()).squaredNorm();
      }
      rewards_.record("footPos", tempReward);
      /// arm regulation + hip_rotation regulation
      tempReward = 0.0;
      Eigen::Matrix<double,4,1> tempArmWeigth; tempArmWeigth << 1.0,0.7,1.0,1.0; /// arm pitch motion low regulation
      for(int i = 0; i < 2; i++){
          tempReward += tempArmWeigth.cwiseProduct(gc_.segment(14 + i * 11,4) - gcInit_.segment(14 + i * 11,4)).squaredNorm();
          tempReward += pow(gc_(8 + i*11) - gcInit_(8 + i*11),2.0); /// hip rotation
      }
      rewards_.record("jointPos", tempReward);
      /// vel acc regulation
      rewards_.record("jointVel", gv_.tail(actionDim_).squaredNorm());                 /// only for standingMode_
      rewards_.record("jointAcc", (gv_.tail(actionDim_) - preJointVel_).squaredNorm()); /// only for standingMode_
//      std::cout << "smoothness2 : " << rewards_.getReward("smoothness2") << std::endl;
//      if (rewards_.getReward("smoothness2") < -2e3){
//          std::cout << "smoothness is too big here "<< (pTarget_ - 2 * prevTarget_ + prevPrevTarget_).squaredNorm() <<"\n" << (pTarget_ - 2 * prevTarget_ + prevPrevTarget_).transpose() << std::endl;
//          std::cout << "pTarget_ \n " << pTarget_.transpose() << std::endl;
//          std::cout << "prevTarget_ \n " << prevTarget_.transpose() << std::endl;
//          std::cout << "prevPrevTarget_ \n " << prevPrevTarget_.transpose() << std::endl;
//      }

      /// sum
      float posReward, negReward;
      posReward = (float)(rewards_.getReward("comAngularVel") + rewards_.getReward("comLinearVel"));
      negReward = (float)(rewards_.getReward("bodyOri") + rewards_.getReward("jointPos") + rewards_.getReward("footPos") + rewards_.getReward("jointVel") + rewards_.getReward("jointAcc") + rewards_.getReward("torque") + rewards_.getReward("footSlip") + rewards_.getReward("smoothness2"));
      rewards_.record("negReward2", negReward); /// only for recording

      return (float)(std::exp(0.2 * negReward) * posReward);
  }

  float getLogBarReward(){
      /// for gait enforcing & foot clearance
      phase_ += simulation_dt_;
      footContactPhase_(0) = sin(phase_/gait_hz_ * 2*3.141592); // left
      footContactPhase_(1) = -footContactPhase_(0); // right

      phaseSin_(0) = sin(phase_/gait_hz_ * 2*3.141592); // for observation
      phaseSin_(1) = cos(phase_/gait_hz_ * 2*3.141592); // for observation
//
      if (!standingMode_){ /// walking
          /// footContactDouble_ -> limit_foot_contact 에 있도록 (-0.3,3) -> Gait Enforcing (요 -0.3 이 벗어나도 되는 범위)
          for(int i=0; i<numLegs_; i++) {
              if (footContact_(i) > 1) { footContactDouble_(i) = 1.0 * footContactPhase_(i); }
              else { footContactDouble_(i) = -1.0 * footContactPhase_(i); }
          }
          /// footClearance_ -> limit_foot_clearance 에 있도록 (-0.12,0.12) -> foot 드는 거 enforcing
          double desiredFootZPosition = 0.21;
          for (int i=0; i<numLegs_; i++){
              if (footContactPhase_(i) < -0.6) { /// during swing, 전체시간의 33 %
                  footClearance_(i) =
                          footToTerrain_.segment(i * 5, 5).minCoeff() - desiredFootZPosition; // 대략, 0.17 sec, 0 보다 크거나 같으면 됨 (enforcing clearance)
              }else{ footClearance_(i) = 0.0; } // max reward (not enforcing clearance)
          }

                armLegCoupling_.setZero();
//                armLegCoupling_(0) = (gc_(7+3)-gcInit_(7+3)) - (gc_(7+11+8)-gcInit_(7+11+8));
//                armLegCoupling_(1) = (gc_(7+11+3)-gcInit_(7+11+3)) - (gc_(7+8)-gcInit_(7+8));
//
//                std::cout << "left knee : " << (gc_(7+3)-gcInit_(7+3)) << " , shoulder : " <<  (gc_(7+11+8)-gcInit_(7+11+8)) << std::endl;
//                std::cout << "right knee : " << (gc_(7+11+3)-gcInit_(7+11+3)) << " , shoulder : " <<  (gc_(7+8)-gcInit_(7+8)) << std::endl;
      } else { /// under standingMode_
          /// standingMode_ 는 zero command 로 부터 유추 가능, command 는 obs 이기 때문에, robot 은 standingMode_인지 아닌지 충분히 알 수 있음
          for (int i=0; i<numLegs_; i++){
              footContactDouble_(i) = 1.0; // around max reward, where this value should go under (-0.3,3)
              footClearance_(i) = 0.0; // max reward (not enforcing clearance)
          }
          armLegCoupling_.setZero();
      }

      /// compute barrier reward
      double barrierJointPos = 0.0, barrierBodyHeight = 0.0, barrierBaseMotion = 0.0, barrierJointVel = 0.0, barrierTargetVel = 0.0, barrierFootContact = 0.0, barrierFootClearance = 0.0, barrierArmLegCoupling = 0.0;
      double tempReward = 0.0;
      /// Log Barrier - limit_joint_pos
      for (int index_joint=0;index_joint<actionDim_;index_joint++){
          relaxedLogBarrier(0.08,limitJointPos_(index_joint,0),limitJointPos_(index_joint,1),gc_(7+index_joint),tempReward);
          barrierJointPos += tempReward;
//          if (tempReward < -100.0){
//              std::cout << index_joint << " th joint limit : " << limitJointPos_.row(index_joint) << " , and real : " << gc_(7+index_joint) << std::endl;
//          }
      }
      /// Log Barrier - limit_body_height
      double tempHeight = 0.0;
      for (int i=0; i<numLegs_; i++){
          tempHeight += gc_(2) - heightMap_->getHeight(footPos_[i](0), footPos_[i](1));
      }
      tempHeight /= static_cast<double>(numLegs_);
//      relaxedLogBarrier(0.04,limitBodyHeight_(0),limitBodyHeight_(1),tempHeight,barrierBodyHeight);
      relaxedLogBarrier(0.03,limitBodyHeight_(0),limitBodyHeight_(1),tempHeight,barrierBodyHeight);

      /// Log Barrier - limit_base_motion
      relaxedLogBarrier(0.2,limitBaseMotion_(0,0),limitBaseMotion_(0,1),bodyLinearVel_(2),tempReward);
      barrierBaseMotion += tempReward;
      for (int i=0;i<2;i++){
          relaxedLogBarrier(0.3,limitBaseMotion_(1,0),limitBaseMotion_(1,1),bodyAngularVel_(i),tempReward);
          barrierBaseMotion += tempReward;
      }
      /// Log Barrier - limit_joint_vel
      for (int i=0;i<actionDim_;i++){
          relaxedLogBarrier(2.0,limitJointVel_(0),limitJointVel_(1),gv_(6+i),tempReward);
          barrierJointVel += tempReward;
      }
      /// Log Barrier - limit_target_vel
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(0)-command_(0),tempReward);
      barrierTargetVel += tempReward;
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyLinearVel_(1)-command_(1),tempReward);
      barrierTargetVel += tempReward;
      relaxedLogBarrier(0.2,limitTargetVel_(0),limitTargetVel_(1),bodyAngularVel_(2)-command_(2),tempReward);
      barrierTargetVel += tempReward;
      /// Log Barrier - limit_foot_contact
      for (int i=0;i<numLegs_;i++){
          relaxedLogBarrier(0.1,limitFootContact_(0),limitFootContact_(1),footContactDouble_(i),tempReward);
          barrierFootContact += tempReward;
      }
      /// Log Barrier - limit_foot_clearance
      for (int i=0;i<numLegs_;i++){
          relaxedLogBarrier(0.02,limitFootClearance_(0),limitFootClearance_(1),footClearance_(i),tempReward);
          barrierFootClearance += tempReward;
      }
      /// Log Barrier - limitArmLegCoupling
      for (int i =0; i< numLegs_; i++){
          relaxedLogBarrier(0.08,limitArmLegCoupling_(0),limitArmLegCoupling_(1),armLegCoupling_(i),tempReward);
          barrierArmLegCoupling += tempReward;
      }

//      if (barrierFootClearance < -40) {
////          std::cout << "barrierJointPos : " <<  barrierJointPos << std::endl;
////          std::cout << "barrierBodyHeight : " <<  barrierBodyHeight << std::endl;
////          std::cout << "barrierBaseMotion : " <<  barrierBaseMotion << std::endl;
////          std::cout << "barrierJointVel : " <<  barrierJointVel << std::endl;
////          std::cout << "barrierTargetVel : " <<  barrierTargetVel << std::endl;
////          std::cout << "barrierFootContact : " <<  barrierFootContact << std::endl;
//          std::cout << "barrierFootClearance : " <<   barrierFootClearance << std::endl;
////                std::cout << "foot clearance : " << footClearance_.transpose() << std::endl;
//      }

      double logClip = -500.0;
      barrierJointPos = fmax(barrierJointPos,logClip);           /// 여기 밖 부분은 gradient 안 받겠다
//      barrierBodyHeight = fmax(barrierBodyHeight,logClip);
//      barrierBaseMotion = fmax(barrierBaseMotion,logClip);
//      barrierJointVel = fmax(barrierJointVel,logClip);
//      barrierTargetVel = fmax(barrierTargetVel,logClip);
//      barrierFootContact = fmax(barrierFootContact,logClip);
//      barrierFootClearance = fmax(barrierFootClearance,logClip);
      rewards_.record("barrierJointPos", barrierJointPos);
      rewards_.record("barrierBodyHeight", barrierBodyHeight);
      rewards_.record("barrierBaseMotion", barrierBaseMotion);
      rewards_.record("barrierJointVel", barrierJointVel);
      rewards_.record("barrierTargetVel", barrierTargetVel);
      rewards_.record("barrierFootContact", barrierFootContact);
      rewards_.record("barrierFootClearance", barrierFootClearance);
      rewards_.record("barrierArmLegCoupling", barrierArmLegCoupling);


      float logBarReward =  (float)(1e-1*(barrierJointPos + barrierBodyHeight + barrierBaseMotion + barrierJointVel + barrierTargetVel + barrierFootContact + barrierFootClearance + barrierArmLegCoupling));
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
    digit_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot_);
    bodyLinearVel_ = rot_.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot_.e().transpose() * gv_.segment(3, 3);
    for(int i = 0; i < numLegs_; i++) {
      digit_->getFramePosition(footJointFrames_[i], footPos_[i]);
      digit_->getFrameVelocity(footJointFrames_[i], footVel_[i]);
      digit_->getFramePosition(hipJointFrames_[i], hipJointPos_[i]);
    }

    /// foot contact update
    footContact_.setZero();
    for(auto& contact: digit_->getContacts()){
        for (size_t i=0; i<numLegs_; i++){
            if(contact.getlocalBodyIndex() == footIndices_[i]){
                footContact_(i) += 1;
            }
        }
    }

    /// update foot terrain
    updateFootToTerrain();
  }

  void updateFootToTerrain(){
    Eigen::Matrix<double, 3, 5> sample_point;
    double point = 0.12; /// foot size
    sample_point.col(0) << point, 0.0, 0.0;
    sample_point.col(1) << 0.0, point/2.0, 0.0;
    sample_point.col(2) << -point, 0.0, 0.0;
    sample_point.col(3) << 0.0, -point/2.0, 0.0;
    sample_point.col(4).setZero();
    for (int i = 0; i < 4; i++) {
        sample_point.col(i) = rot_.e().transpose() * sample_point.col(i).eval();
    }
    Eigen::Matrix<double, 5, 1> temp_foot;
    Eigen::Matrix<double, 3, 1> temp3;
    for (int k = 0; k < numLegs_; k++) {
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

      arrow_pos_offset << 0,0,0.60;
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
//          footContactPhase_.setZero();
          phaseSin_.setZero();
      }
      obDouble_ << rot_.e().row(2).transpose(),                               /// body orientation. 3
          bodyAngularVel_,                                                      /// body angular velocity. 3
          gc_.tail(actionDim_),                                                      /// joint pos 22
          gv_.tail(actionDim_),                                                      /// joint velocity 22

          prevTarget_,                                                          /// previous action 22
          prevPrevTarget_,                                                      /// preprevious action 22
          jointPosErrorHist_[0], jointPosErrorHist_[6], jointPosErrorHist_[12], /// joint History 66 (0.18, 0.12, 0.6)
          jointVelHist_[0], jointVelHist_[6], jointVelHist_[12],                /// joint History 66 (0.18, 0.12, 0.6)
          rot_.e().transpose() * (footPos_[0].e() - gc_.head(3)), rot_.e().transpose() * (footPos_[1].e() - gc_.head(3)),
          /// relative foot position with respect to the body COM, expressed in the body frame 6
          command_,                                                             /// command 3
//          footContactPhase_.head(2), /// footContactPhase 2
          phaseSin_, /// phase encoding 2
          static_cast<double>(standingMode_);  /// standingMode 1

      double noise = 0.0;
      for (int i=0; i<obDim_; i++){
          if (i<3)       {noise = 0.03;}  /// body orientation
          else if(i<6)   {noise = 0.1;}   /// body angular velocity (rad/sec)
          else if(i<28)  {noise = 0.05;}  /// joint pos             (rad)
          else if(i<50)  {noise = 0.5;}   /// joint vel             (rad/sec)
          else if(i<94)  {noise = 0.0;}  /// action related
          else if(i<160) {noise = 0.0;}   /// action related
          else if(i<226) {noise = 0.1;}  /// vel history
          else if(i<232) {noise = footObsNoise_(i-226);} /// relative foot pos (2 cm)
          else           {noise = 0.0;}

          obDouble_(i) += uniDist_(gen_) * noise;
      }

    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  void valueObserve(Eigen::Ref<EigenVec> ob) final { /// obs + (true) estimated_state
      if (standingMode_){
          phaseSin_.setZero();
      }
      valueObDouble_ << rot_.e().row(2).transpose(),                               /// body orientation. 3
              bodyAngularVel_,                                                      /// body angular velocity. 3
              gc_.tail(actionDim_),                                                      /// joint pos 22
              gv_.tail(actionDim_),                                                      /// joint velocity 22

              prevTarget_,                                                          /// previous action 22
              prevPrevTarget_,                                                      /// preprevious action 22
              jointPosErrorHist_[0], jointPosErrorHist_[6], jointPosErrorHist_[12], /// joint History 66 (0.18, 0.12, 0.6)
              jointVelHist_[0], jointVelHist_[6], jointVelHist_[12],                /// joint History 66 (0.18, 0.12, 0.6)
              rot_.e().transpose() * (footPos_[0].e() - gc_.head(3)), rot_.e().transpose() * (footPos_[1].e() - gc_.head(3)),
              /// relative foot position with respect to the body COM, expressed in the body frame 6
              command_,                                                             /// command 3
              phaseSin_, /// phase sin cos 2
              static_cast<double>(standingMode_),                                   /// standingMode 1

              bodyLinearVel_,                                                       /// body linear velocity. 3
              footClearance_,                                                       /// min foot z
              footContact_.cast<double>();

      /// convert it to float
      ob = valueObDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);
    /// if the contact body is not feet
    for(auto& contact: digit_->getContacts())
        if ((std::find(footIndices_.begin(), footIndices_.end(), contact.getlocalBodyIndex()) == footIndices_.end())
                and (std::find(tarsusIndices_.begin(), tarsusIndices_.end(), contact.getlocalBodyIndex()) == tarsusIndices_.end())) {
            return true;
        }
    terminalReward = -0.f;
    return false;
  }

  void curriculumUpdate() {
      /// for each iteration
      iter_ ++;
//      if (curriculum_<2.0){
//          curriculum_ = (double)iter_ * (1.0/1000.0); /// 1000 iter -> 1.0
//      }else{
//          curriculum_ = (double)(iter_-2000) * (1.0/1500.0) + 2.0; /// 1500 iter -> 1.0
//          curriculum_ = (curriculum_ > 3.0) ? 3.0 : curriculum_;
//      }

      curriculum_ = 0.0;

      world_->removeObject(heightMap_);
//      heightMap_ = HeightMapSample(world_.get(),iter_%4,curriculum_,gen_,uniDist_);
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
          digit_->setState(gcInit_,gvInit_);
          double heightShift = 1e2, temp = 0.0;
          for (int i = 0; i < numLegs_; i++){
              digit_->getFramePosition(footJointFrames_[i], footPos_[i]);
              temp = footPos_[i](2) - 0.065- heightMap_->getHeight(footPos_[i](0), footPos_[i](1));
              if (temp < heightShift){heightShift = temp;}
          }
          gcNoise_(2) -= heightShift;

          /// reset
          digit_->setState(gcNoise_, gvInit_);
          updateObservation();
      }
  }

 private:
  int gcDim_, gvDim_, numLegs_;
  bool visualizable_ = false;
  double terminalRewardCoeff_ = -10.0;
  raisim::ArticulatedSystem* digit_;

  Eigen::VectorXd gc_, gv_;
  Eigen::Vector<double,29> gcInit_, gcNoise_, gcDes_;
  Eigen::Vector<double,28> gvInit_, gvNoise_, gvDes_;
  Eigen::Vector<double,22> pTarget_, prevTarget_, prevPrevTarget_, preJointVel_, jointFrictions_;
  Eigen::Vector<double,28> jointPgain_, jointDgain_;

  raisim::Mat<3,3> rot_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_, valueObDouble_, estDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::vector<size_t> footIndices_, tarsusIndices_;
  /// additional
  Eigen::Vector3d command_;                     // vx, vy, w
  std::vector<std::string> footJointFrames_;
  std::vector<std::string> hipJointFrames_;
  Eigen::Vector2i footContact_;
  std::vector<raisim::Vec<3>> footPos_,footVel_, hipJointPos_, refBodyToFoot_;
  double phase_;
  double gait_hz_;
  Eigen::Matrix<double,2,1> footContactDouble_; // gait
  Eigen::Matrix<double,2,1> footContactPhase_;  // gait hz
  Eigen::Matrix<double,2,1> footClearance_;     // foot clearance
  Eigen::Matrix<double,2,1> footSlip_;     // foot clearance
  Eigen::Matrix<double,10,1> footToTerrain_; // 10 sample point for each foot
  Eigen::Matrix<double,2,1> phaseSin_;  // sin cos representation of phase
  Eigen::Matrix<double,6,1> footObsNoise_;
        Eigen::Matrix<double,2,1> armLegCoupling_;
  bool standingMode_;
  double standingRegulation_;
  Eigen::VectorXd smoothnessWeight_;
  Eigen::Vector3d footPosWeight_;

  /// log barrier function
  Eigen::Matrix<double,22,2> limitJointPos_;
  Eigen::Matrix<double,1,2> limitBodyHeight_;
  Eigen::Matrix<double,1,2> limitBaseMotion_; // z vel, roll,pitch vel
  Eigen::Matrix<double,1,2> limitJointVel_;
  Eigen::Matrix<double,1,2> limitTargetVel_;
  Eigen::Matrix<double,1,2> limitFootClearance_;
  Eigen::Matrix<double,1,2> limitFootContact_; // for gait enforcing
        Eigen::Matrix<double,1,2> limitArmLegCoupling_; // for natural arm motion, in pitch
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
  raisim::HeightMap* heightMap_;
  /// curriculum
  double curriculum_;
  int iter_;
  double mu_;
  /// for barrier
  float barrierReward_;

  thread_local static std::mt19937 gen_;
  thread_local static std::normal_distribution<double> normDist_;
  thread_local static std::uniform_real_distribution<double> uniDist_;

};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;
thread_local std::normal_distribution<double> raisim::ENVIRONMENT::normDist_(0.0,1.0);
thread_local std::uniform_real_distribution<double> raisim::ENVIRONMENT::uniDist_(-1.0,1.0);
}

