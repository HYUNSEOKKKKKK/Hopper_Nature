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
    dhal_ = world_->addArticulatedSystem(resourceDir_+"../hound/rsc/Hop_verParallelAnkleLinks_ver20250304/Hop_verParallelAnkleLinks_20250311.urdf");
    dhal_->setName("dhal");
    dhal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /// Dim
    gcDim_ = 16;
    gvDim_ = 15;
    numLegs_ = 1;
    numEdges_ = 4;
    actionDim_ = 3;
    obDim_ = 42;
//    obDim_ = 24; // without history (pos error, joint vel)
    estDim_ = 16;
    valueObDim_ = obDim_ + estDim_;

    /// initialize
    gc_.setZero(gcDim_); gcInit_.setZero(); gcNoise_.setZero();
    gv_.setZero(gvDim_); gvInit_.setZero(); gvNoise_.setZero();
    pTarget_.setZero(); prevTarget_.setZero(); prevPrevTarget_.setZero(); preJointVel_.setZero();
    jointFrictions_.setZero();

    /// this is nominal configuration of robot
    gcInit_.segment(0,7) << 0.0,0.0,0.73,   0.9887711, 0.0, -0.1494381, 0.0;
    gcInit_.segment(7,3) << 0.4, -0.1, 0.0; // knee, ankle output (passive)
    gcInit_.tail(6) << 0.727265, -0.002499, 0.505897, 0.001567, 0.628833, 0.346578; // universal passive, ankle input (active)
    gcInit_.segment(3,4).normalize();
    gc_ = gcInit_;

    /// set pd gains
    pGain_ = 50.0; dGain_ = 5.0;
    jointPgain_.setZero(); jointPgain_.tail(actionDim_).setConstant(pGain_); // knee, ankle input (active)
    jointDgain_.setZero(); jointDgain_.tail(actionDim_).setConstant(dGain_);
    dhal_->setPdGains(Eigen::VectorXd::Zero(gvDim_), Eigen::VectorXd::Zero(gvDim_));
    dhal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
    /// set pd gains for sub-step
    subStepPgain_.setZero(); subStepDgain_.setZero();
    subStepPgain_.segment(6,3).setConstant(200.0); // knee, ankle output (passive) -> only for sub-step
    subStepDgain_.segment(6,3).setConstant(3.0);

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);
    obDoubleLpf_.setZero(obDim_);
    valueObDouble_.setZero(valueObDim_);
            estDouble_.setZero(estDim_);

    /// action scaling
    actionMean_(0) = gcInit_(7);
    actionMean_.tail(2) = gcInit_.tail(2);
//            actionStd_.setConstant(0.3);
    actionStd_.setConstant(0.4);
//      actionStd_ << 0.4, 0.2, 0.2;

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.push_back(dhal_->getBodyIdx("Foot"));
    exceptionIndices_.push_back(dhal_->getBodyIdx("Calf")); // due to added links of closed-loop -> it can stand with these contacts
    exceptionIndices_.push_back(dhal_->getBodyIdx("Left_Ankle_Link"));
    exceptionIndices_.push_back(dhal_->getBodyIdx("Left_Ankle_Link_Input"));
    exceptionIndices_.push_back(dhal_->getBodyIdx("Right_Ankle_Link"));
    exceptionIndices_.push_back(dhal_->getBodyIdx("Right_Ankle_Link_Input"));
    footJointFrames_.push_back("02_ankle_roll_joint");  /// joint
//    hipJointFrames_.push_back("hip_abduction_left");

      bodyIndices_.push_back(dhal_->getBodyIdx("Thigh"));
      bodyIndices_.push_back(dhal_->getBodyIdx("Calf")); // calf contact 추가 -> due to closed loop contact points (should change the body contact num part)

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
    limitBaseMotion_.row(0) << -1.2,1.2; // z, pitch
    limitBaseMotion_.row(1) << -0.8,0.8; // roll
    limitJointVel_.row(0) << -6,6;       //  for knee (10.11)
    limitJointVel_.row(1) << -8,8;       // for ankle (18)
    limitTargetVel_ << -0.6,0.6;
    limitFootContact_ << -0.3,2;
    limitFootClearance_ << -0.08,1.0; // 어차피 desired_foot_clearance 를
    limitBodyContact_ << -1.0,1.0;
    limitCOMpos_ << -0.04, 0.04; /// only enforced standingMode

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
    prevTerminal_ = false;
    comPos_.setZero();
    comToFootLocalFrame_.setZero();

    /// initialize history
    jointPosErrorHist_ = std::vector<Eigen::VectorXd>(9,Eigen::VectorXd::Zero(actionDim_));
    jointVelHist_ = std::vector<Eigen::VectorXd>(9,Eigen::VectorXd::Zero(actionDim_));
    genForceTargetHist_ = std::vector<Eigen::VectorXd>(3,Eigen::VectorXd::Zero(gvDim_));  /// delay 는 2 ms _ 1 tick 으로 설정 -> 1~2 tick delay
    genForceTarget_.setZero(gvDim_);
    /// initialize gait
    phase_ = 0.0;
    gait_hz_ = 0.80;
//    gait_hz_ = 0.70; // v 2.2

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

      edgePosLocal_.col(0) << -0.12, -0.05, -0.015-0.065;
      edgePosLocal_.col(1) << -0.12, 0.05, -0.015-0.065;
      edgePosLocal_.col(2) << 0.12, -0.05, -0.015-0.065;
      edgePosLocal_.col(3) << 0.12, 0.05, -0.015-0.065;

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

      ///
      auto temp = dhal_->getMassMatrix();
      nominalMass_.push_back(dhal_->getMass()[0]); // trunk + thigh
      nominalMass_.push_back(dhal_->getMass()[1]); // calf
      nominalMass_.push_back(dhal_->getMass()[3]); // foot

      dhal_->getCollisionBody("Foot/0").setMaterial("rubber");
  }

  void init() final { }

  void reset() final {
    jointPgain_.setZero(); jointPgain_.tail(actionDim_).setConstant(pGain_ + pGain_*0.1*uniDist_(gen_));
    jointDgain_.setZero(); jointDgain_.tail(actionDim_).setConstant(dGain_ + dGain_*0.1*uniDist_(gen_));
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
            double maxCommand = 0.4 + comCurriculum * 0.6; // 평지 lin x max 1.5
            command_ << maxCommand * uniDist_(gen_), 0.6 * uniDist_(gen_), 0.6 * uniDist_(gen_);     // [lix x max, 0.6, 0.6]
//            command_(0) = (command_(0) < -0.8) ? command_(0)+1.6 : command_(0);           // 뒤로가는 건 max -0.8
        } while (command_.norm() < 0.2);
    }

    mu_ = 0.7 + 0.3 * uniDist_(gen_);
//    world_->setDefaultMaterial(mu_, 0, 0);
//    world_->setMaterialPairProp("default","rubber",mu_, 0.6+0.1*uniDist_(gen_), 0.001); // restitution [0.5,0.7]
    /// curriculum
    double restitution_curriculum = (double)(iter_)/2000.0;
    restitution_curriculum = (restitution_curriculum > 1.0) ? 1.0 : restitution_curriculum;
    restitution_curriculum = restitution_curriculum*0.5 + 0.1 + 0.1*uniDist_(gen_); // [0.0,0.2] -> [0.5, 0.7] (2000 iter)
    world_->setMaterialPairProp("default","rubber",mu_, restitution_curriculum, 0.001); // restitution [0.5,0.7]

    /// initialize the pose /// 넘어진 상태에서 그대로 reset 되는 경우가 생김
    bool reset = true;
    if (standingMode_ and !prevTerminal_){ reset = uniDist_(gen_) > 0.0;} /// very important

    if(!reset){ /// command -> sudden stop
        /// 굉장히 주의를 요함 (heightMap 이 변하는 경우, 넘어졌는데 reset 안되는 경우 등)
        gcNoise_ = gc_;
        gvNoise_ = gv_;
        gcNoise_.head(3) = gcInit_.head(3); // 요것 때문에 무조건 땅으로 데리고 오는 거 해줘야 함
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
        gcNoise_(7) += (uniDist_(gen_) * 0.5 * ((standingMode_)? 1.2 : 1.0)+0.22); // knee
        gcNoise_(8) += uniDist_(gen_) * 0.3 * ((standingMode_)? 1.2 : 1.0); // ankle output pitch (passive)
        gcNoise_(9) += uniDist_(gen_) * 0.2 * ((standingMode_)? 1.2 : 1.0); // ankle output roll (passive)

        /// Generalized Velocities randomization.
        gvNoise_.setZero();
        for (int i = 0; i < gvDim_; i++) {
            if (i < 6) {
                gvNoise_(i) = uniDist_(gen_) * 0.3 * initializeCurriculum;
            } else if (i == 6) {
                gvNoise_(i) = uniDist_(gen_) * 1.0; // knee, no noise for ankle parts
            }
            if (standingMode_) {gvNoise_(i) *= 1.5;}
        }
        subStep();
    }
    dhal_->setState(gcNoise_,gvNoise_);

    /// preventing foot penetration (주의, if map changed, then, it should be always executed, but, if the robot fails, then, it could be problematic !!!)
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
//        pTarget_ = gc_.tail(actionDim_);
        pTarget_ = actionMean_;
        prevTarget_ = pTarget_; prevPrevTarget_ = pTarget_; preJointVel_.setZero();
        for (auto& vec : jointPosErrorHist_) { vec.setZero(); }
        for (auto& vec : jointVelHist_) { vec.setZero(); }

        phase_ = 0.0 + uniDist_(gen_) * gait_hz_ * 0.3;
        footContactPhase_.setZero();
        footClearance_.setZero();
        obDoubleLpf_.setZero();
    }

    /// even though not reset, these values should be reset -> empirical result
    for (auto& vec : jointPosErrorHist_) { vec.setZero(); }
    for (auto& vec : jointVelHist_) { vec.setZero(); }

    terminalStack_ = 0;
    prevTerminal_ = false;
    /// random joint friction
    jointFrictions_(0) = 3.5 + 3.5 * uniDist_(gen_);
    jointFrictions_(1) = 0.5 + 0.5 * uniDist_(gen_);
    jointFrictions_(2) = 0.5 + 0.5 * uniDist_(gen_);
    jointFrictions_ /= 1e1;
    /// randomization for mass
    dhal_->getMass()[0] = nominalMass_[0] * (1+uniDist_(gen_)*0.05); // 0.95~1.05, trunk + thigh
    dhal_->getMass()[1] = nominalMass_[1] * (1+uniDist_(gen_)*0.05); // 0.95~1.05, calf
    dhal_->getMass()[3] = nominalMass_[2] * (1+uniDist_(gen_)*0.05); // 0.95~1.05, foot

//    std::cout << "dhal_->getMass()[0]: " << dhal_->getMass()[0] << std::endl;
//    std::cout << "dhal_->getMass()[1]: " << dhal_->getMass()[1] << std::endl;
//    std::cout << "dhal_->getMass()[2]: " << dhal_->getMass()[2] << std::endl;
//    std::cout << "dhal_->getMass()[3]: " << dhal_->getMass()[3] << std::endl;
  }

  void subStep() {
      /// Used in reset function to match the closed loop in ankle
      // For the randomized ankle output, this substep adjusts other closed-loop parts to ensure the loop is closed properly.
      // change gcNoise_ for closed-loop ankle part
      world_->setTimeStep(0.020);
      dhal_->setPdGains(subStepPgain_, subStepDgain_); // here, PD gain is enforced only for knee, ankle output (passive)
      dhal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
      gcNoise_(2) += 10.0;
      dhal_->setPdTarget(gcNoise_, gvNoise_);
      dhal_->setState(gcNoise_,gvNoise_);

      raisim::Vec<4> quat = gcNoise_.segment(3,4);
      // substep
      for (int i=0; i<150; i++){
          world_->integrate();
          // prevent the contact with ground
          dhal_->setBasePos(gcNoise_.head(3));
          dhal_->setBaseOrientation(quat);
          dhal_->setBaseVelocity({0.0, 0.0, 0.0});
          dhal_->setBaseAngularVelocity({0.0, 0.0, 0.0});
      }
      dhal_->getState(gc_,gv_);
      gcNoise_.tail(9) = gc_.tail(9);
      /// for check
//      std::cout << "final step joint active: " << gc_(7) << " , " << gc_.tail(2).transpose() << " , ankle passive: " << gc_.segment(8,2).transpose() << " , gv (active) :" << gv_.tail(2).transpose() << std::endl;
//      std::cout << "temp: " << gc_.tail(9).transpose() << std::endl;

      // back to the original setting
      gcNoise_(2) -= 10.0;
      dhal_->setBasePos(gcNoise_.head(3));
      world_->setTimeStep(simulation_dt_);
      dhal_->setPdGains(Eigen::VectorXd::Zero(gvDim_), Eigen::VectorXd::Zero(gvDim_));

  }
  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
//    auto action_conversion = action.cast<double>();
//    pTarget_ << action_conversion(0), action_conversion(1)+action_conversion(2)/2.0, action_conversion(1)-action_conversion(2)/2.0;
    pTarget_ = action.cast<double>();
    pTarget_ = pTarget_.cwiseProduct(actionStd_);
    pTarget_ += actionMean_;                                   /// joint P target

    /// simulation
    double avgReward = 0.0;
    barrierReward_ = 0.0;

    double delayRandomValue = uniDist_(gen_);
    int delayIdx = (delayRandomValue < 0.0) ? 0 : 1;
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      /// compute target torque
      computeTorque();
      dhal_->setGeneralizedForce(genForceTargetHist_[delayIdx]); /// 2ms x (1~2) tick delay (torque command in PC -> actual torque in real robot)
//      dhal_->setGeneralizedForce(genForceTargetHist_[0]); /// 2ms x max tick delay (torque command in PC -> actual torque in real robot)
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
            /// scale down
      avgReward /= 2e1;
      barrierReward_ /= 2e1;

//      std::cout << footContact_ << ", " << footContactPhase_ << std::endl;

    updateHistory();

    return avgReward;
  }

  void computeTorque(){
        genForceTargetHist_.erase(genForceTargetHist_.begin());
        Eigen::VectorXd tempForce(actionDim_);
        tempForce(0) = jointPgain_(0)*(pTarget_(0)-gc_(7))
                       + jointDgain_(0)*(-gv_(6)); // knee
        tempForce.tail(2) = jointPgain_.tail(2).cwiseProduct(pTarget_.tail(2)-gc_.tail(2))
                            + jointDgain_.tail(2).cwiseProduct(-gv_.tail(2)); // ankle input (active)
        /// joint friction (static friction, torque 잡아먹는 효과)
        for (int i = 0; i < actionDim_; i++){
          double jTorque = tempForce(i);
          jTorque = (jTorque>0) ? std::min(jointFrictions_(i), jTorque) : std::max(-jointFrictions_(i), jTorque);
          tempForce(i) -= jTorque;
        }

        /// MOR related
//        getTorqueMOR(tempForce);

        ///
        genForceTarget_.setZero();
        genForceTarget_(6) = tempForce(0);
        genForceTarget_.tail(2) = tempForce.tail(2);
        genForceTargetHist_.push_back(genForceTarget_);
  }

//  void getTorqueMOR(Eigen::VectorXd tempForce){
//      //compute motor input vel & torque
//
//      motor_torque = tempForce
//      gv_motor.block(0,0,3,1) = convJointVelocity2MotorVelocity*gv_.block(6,0,3,1);
//      gv_motor.block(3,0,3,1) = convJointVelocity2MotorVelocity*gv_.block(9,0,3,1);
//      gv_motor.block(6,0,3,1) = convJointVelocity2MotorVelocity*gv_.block(12,0,3,1);
//      gv_motor.block(9,0,3,1) = convJointVelocity2MotorVelocity*gv_.block(15,0,3,1);
//
//      motor_torque.block(0,0,3,1) = convJointTorque2MotorTorque*jointTorque.block(6,0,3,1);
//      motor_torque.block(3,0,3,1) = convJointTorque2MotorTorque*jointTorque.block(9,0,3,1);
//      motor_torque.block(6,0,3,1) = convJointTorque2MotorTorque*jointTorque.block(12,0,3,1);
//      motor_torque.block(9,0,3,1) = convJointTorque2MotorTorque*jointTorque.block(15,0,3,1);
//
//      /// 2nd method
//      double upperBound = 0.0;
//      double lowerBound = 0.0;
//      for (int i_leg = 0; i_leg < 4; i_leg++){
//          for (int jointType = 0; jointType<3; jointType++){
//              double motorVel1, motorVel2, motorVel3, motorVel4, motorVel5;
//              motorVel1 = (tau_max(jointType) + intercept(jointType))/inclination(jointType);
//              motorVel2 = intercept(jointType)/inclination(jointType);
//              motorVel3 = temp_V(jointType);
//              motorVel4 = -motorVel2;
//              motorVel5 = -motorVel1;
//              double motorVel = gv_motor(3*i_leg+jointType);
//              double motorTorque = motor_torque(3*i_leg+jointType);
//              if (motorVel > motorVel1 && motorVel < motorVel2){
//                  if (motorTorque <= 0){
//                      upperBound = 1e-4;
//                      lowerBound = -1e-4;
//                  }
//                  else{
//                      upperBound = tau_max(jointType);
//                      lowerBound = inclination(jointType)*motorVel - intercept(jointType);
////        lowerBound = 0.0;
//                  }
//              }
//              else if (motorVel >= motorVel2 && motorVel < -motorVel3){
//                  upperBound = tau_max(jointType);
//                  lowerBound = inclination(jointType)*motorVel - intercept(jointType);
//              }
//              else if(motorVel >= -motorVel3 && motorVel < motorVel3){
//                  upperBound = tau_max(jointType);
//                  lowerBound = -tau_max(jointType);
//              }
//              else if(motorVel >= motorVel3 && motorVel < motorVel4){
//                  upperBound = inclination(jointType)*motorVel + intercept(jointType);
//                  lowerBound = -tau_max(jointType);
//              }
//              else if(motorVel >= motorVel4 && motorVel < motorVel5){
//                  if (motorTorque >= 0){
//                      upperBound = 1e-4;
//                      lowerBound = -1e-4;
//                  }
//                  else{
//                      upperBound = inclination(jointType)*motorVel + intercept(jointType);
////        upperBound = 0.0;
//                      lowerBound = -tau_max(jointType);
//                  }
//              }
//              else{
////      upperBound = tau_max(jointType);
////      lowerBound = -tau_max(jointType);
//                  upperBound = 1e-4;
//                  lowerBound = -1e-4;
//              }
//              // limit motor torque with boundaries
//              if (motorTorque > upperBound){
//                  motor_torque(3*i_leg+jointType) = upperBound;
//              } else if(motorTorque < lowerBound){
//                  motor_torque(3*i_leg+jointType) = lowerBound;
//              }
//          }
//          finalJointTorque.segment(3*i_leg,3) = convMotorTorque2JointTorque*motor_torque.segment(3*i_leg,3);
//      }
//      MORconstrainedJointTorque.head(6).setZero();
//      MORconstrainedJointTorque.tail(12) = finalJointTorque;
//    }

  float getBarrierReward() final {
            return barrierReward_;
  }

  void updateHistory(){
      prevPrevTarget_ = prevTarget_;
      prevTarget_ = pTarget_;

      Eigen::VectorXd tempVec(3);
      tempVec << gv_(6), gv_.tail(2); // knee, ankle input
      jointVelHist_.erase(jointVelHist_.begin());
      jointVelHist_.push_back(tempVec);

      jointPosErrorHist_.erase(jointPosErrorHist_.begin());
      tempVec << gc_(7), gc_.tail(2); // knee, ankle input
      jointPosErrorHist_.push_back(pTarget_ - tempVec);
  }

  double getReward(){
      standingReward(); /// there is order
      rewards_.record("negSumPos",getNegPosReward());
      return rewards_.getReward("negSumPos");
  }

  void standingReward(){
      /// for standingMode
      if (!standingMode_){
          limitBaseMotion_.row(0) << -1.2,1.2;
          limitBaseMotion_.row(1) << -0.8,0.8;
          standingSmoothness_ = 1.0;
          footPosWeight_ << 0.6,1.0,0.4;
      } else {
          limitBaseMotion_.row(0) << -0.8,0.8;
          limitBaseMotion_.row(1) << -0.4,0.4;
          standingSmoothness_ = 1.0;
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
      rewards_.record("smoothness1", (pTarget_ - prevTarget_).squaredNorm()  * standingSmoothness_);
      rewards_.record("smoothness2", (pTarget_ - 2 * prevTarget_ + prevPrevTarget_).squaredNorm()  * standingSmoothness_);
      rewards_.record("baseMotion", 0.4*pow(bodyLinearVel_(2),2) + 0.2*abs(bodyAngularVel_(0)) + 0.2*abs(bodyAngularVel_(1)));

      /// pos vel acc regulation -> put only for output part (knee, ankle output (passive))
      Eigen::Vector<double,5> tempJoint, tempJointWeight;
      tempJoint << gc_.segment(7,actionDim_) - gcInit_.segment(7,actionDim_) , gc_.tail(2) - gcInit_.tail(2);
      tempJointWeight << 0.5, 1.0, 1.0, 1.0, 1.0; // knee, ankle pitch, ankle roll, ankle input L, ankle input R
      rewards_.record("jointPos", (tempJointWeight.cwiseProduct(tempJoint)).squaredNorm());

      tempJoint << gv_.segment(6,actionDim_), gv_.tail(2);
      tempJointWeight << 0.5, 1.5, 1.5, 1.5, 1.5; // knee, ankle pitch, ankle roll, ankle input L, ankle input R
      rewards_.record("jointVel", (tempJointWeight.cwiseProduct(tempJoint)).squaredNorm());
      rewards_.record("jointAcc", (tempJointWeight.cwiseProduct(tempJoint - preJointVel_)).squaredNorm());

      /// force regulation -> put only for active part (knee, ankle input (active))
      Eigen::Vector3d tempForce, tempForceWeight;
      tempForceWeight << 0.5, 1.0, 1.0; // knee, ankle pitch, ankle roll
      tempForce(0) = genForceTargetHist_[0](6); // knee
      tempForce.tail(2) = genForceTargetHist_[0].tail(2); // knee
      rewards_.record("torque", (tempForceWeight.cwiseProduct(tempForce).squaredNorm()));

      /// body contact reward
      rewards_.record("bodyContact",(double)bodyContact_);
      /// com pos regularization
      rewards_.record("comPos",comToFootLocalFrame_.head(2).squaredNorm()*(double)standingMode_);

      /// sum
      float posReward, negReward;
      posReward = (float)(rewards_.getReward("comAngularVel") + rewards_.getReward("comLinearVel"));
      negReward = (float)(rewards_.getReward("bodyOri") + rewards_.getReward("jointPos") + rewards_.getReward("jointVel") + rewards_.getReward("jointAcc") + rewards_.getReward("torque")
              + rewards_.getReward("footSlip") + rewards_.getReward("smoothness1") + rewards_.getReward("smoothness2") + rewards_.getReward("bodyContact") + rewards_.getReward("baseMotion")
              + rewards_.getReward("comPos"));
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
          double desiredFootZPosition = 0.16;
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
      double barrierJointPos = 0.0, barrierBodyHeight = 0.0, barrierBaseMotion = 0.0, barrierJointVel = 0.0, barrierTargetVel = 0.0, barrierFootContact = 0.0, barrierFootClearance = 0.0, barrierBodyContact = 0.0, barrierCOMpos = 0.0;
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
      relaxedLogBarrier(2.0,limitJointVel_(0,0),limitJointVel_(0,1),gv_(6),tempReward); barrierJointVel += tempReward; // knee
      for (int i=0;i<2;i++){
          relaxedLogBarrier(2.0,limitJointVel_(1,0),limitJointVel_(1,1),gv_(7+i),tempReward); barrierJointVel += tempReward; // ankle passive
          relaxedLogBarrier(2.0,limitJointVel_(1,0),limitJointVel_(1,1),gv_(13+i),tempReward); barrierJointVel += tempReward; // ankle active
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
          relaxedLogBarrier(0.02,limitFootClearance_(0),limitFootClearance_(1),footClearance_(i),tempReward);
          barrierFootClearance += tempReward;
      }
      /// Log Barrier - limit_body_contact
      relaxedLogBarrier(0.5, limitBodyContact_(0), limitBodyContact_(1),-bodyContact_,tempReward);
      barrierBodyContact += tempReward;
      /// Log Barrier - limit_COM_pos
      if (standingMode_){
          relaxedLogBarrier(0.02, limitCOMpos_(0), limitCOMpos_(1), comToFootLocalFrame_(0), tempReward);
          barrierCOMpos += tempReward;
          relaxedLogBarrier(0.02/2.0, limitCOMpos_(0)/2.0, limitCOMpos_(1)/2.0, comToFootLocalFrame_(1), tempReward);
          barrierCOMpos += tempReward;
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
      rewards_.record("barrierBodyContact", barrierBodyContact);
      rewards_.record("barrierCOMpos", barrierCOMpos);


      float logBarReward =  (float)(1e-1*(barrierJointPos + barrierBodyHeight + barrierBaseMotion + barrierJointVel + barrierTargetVel
              + barrierFootContact + barrierFootClearance + barrierBodyContact + barrierCOMpos));
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
    preJointVel_ << gv_.segment(6,actionDim_), gv_.tail(2);
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
    /// to enforce comPos to foot center
    comPos_ = dhal_->getCOM();
    comToFootLocalFrame_ = footOrientation_[0].e().transpose()*(comPos_.e()-footPos_[0].e());
//    std::cout << "comToFootLocalFrame_: " << comToFootLocalFrame_.head(2).transpose() << std::endl;

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
          for (size_t i=0; i<2; i++){
              if(contact.getlocalBodyIndex() == bodyIndices_[i]){
//                  std::cout << "bodyIndices_[i] : " << i << std::endl;
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

      obDouble_ << rot_.e().row(2).transpose(),                                 /// body orientation. 3
          bodyAngularVel_,                                                      /// body angular velocity. 3
          gc_(7)-gcInit_(7),
          gc_.tail(2)-gcInit_.tail(2),                                          /// joint pos 3
          gv_(6)/2e1,
          gv_.tail(2)/2e1,                                                      /// joint velocity 3

          prevTarget_ - actionMean_,                                            /// previous action 3
          prevPrevTarget_ - actionMean_,                                        /// preprevious action 3
          jointPosErrorHist_[0], jointPosErrorHist_[3], jointPosErrorHist_[6],  /// joint History 9 (0.18, 0.12, 0.6)
          jointVelHist_[0]/2e1, jointVelHist_[3]/2e1, jointVelHist_[6]/2e1,     /// joint History 9 (0.18, 0.12, 0.6)
          command_,                                                             /// command 3
          phaseSin_,                                                            /// phase encoding 2
          static_cast<double>(standingMode_);                                   /// standingMode 1

//        std::cout << "obdouble_: " << obDouble_.head(10).transpose() << std::endl;
      double noise = 0.0;
      // for (int i=0; i<obDim_; i++){
      //    if (i<3)       {noise = 0.03;}  /// body orientation
      //    else if(i<6)   {noise = 0.05;}   /// body angular velocity (rad/sec)
      //    else if(i<9)   {noise = 0.05;}  /// joint pos             (rad)
      //    else if(i<12)  {noise = 0.1;}   /// joint vel             (rad/sec)
       //   else           {noise = 0.0;}
      //    obDouble_(i) += uniDist_(gen_) * noise * 0.1;
      //}

      double alpha = 0.5;
      obDoubleLpf_.head(36) = alpha*obDoubleLpf_.head(36) + (1-alpha)*obDouble_.head(36);
      obDoubleLpf_.tail(6) = obDouble_.tail(6);
    /// convert it to float
//    ob = obDouble_.cast<float>();
    ob = obDoubleLpf_.cast<float>();
  }

  void valueObserve(Eigen::Ref<EigenVec> ob) final { /// obs + (true) estimated_state
      if (standingMode_){
          phaseSin_.setZero();
      }
      Eigen::Vector3d temp; /// ref foot to body com
      temp << 0.0, 0.0, -0.70;
      valueObDouble_ << rot_.e().row(2).transpose(),                                /// body orientation. 3
              bodyAngularVel_,                                                      /// body angular velocity. 3
              gc_(7)-gcInit_(7),
              gc_.tail(2)-gcInit_.tail(2),                                          /// joint pos 3
              gv_(6)/2e1,
              gv_.tail(2)/2e1,                                                      /// joint velocity 3

              prevTarget_- actionMean_,                                             /// previous action 3
              prevPrevTarget_- actionMean_,                                         /// preprevious action 3
              jointPosErrorHist_[0], jointPosErrorHist_[3], jointPosErrorHist_[6],  /// joint History 9 (0.18, 0.12, 0.6)
              jointVelHist_[0]/2e1, jointVelHist_[3]/2e1, jointVelHist_[6]/2e1,     /// joint History 9 (0.18, 0.12, 0.6)
              command_,                                                             /// command 3
              phaseSin_,                                                            /// phase encoding 2
              static_cast<double>(standingMode_),                                   /// standingMode 1

              bodyLinearVel_,                                                       /// body linear velocity. 3
              (footToTerrain_(0) + footToTerrain_(2)) * 5e0,
              (footToTerrain_(4) + footToTerrain_(6)) * 5e0,                        /// heel & toe height 2
              static_cast<double>(footContact_)/4.0,                                /// 1 foot contact num
              static_cast<double>(bodyContact_)/4.0,                                /// 1 body contact num
              (rot_.e().transpose() * (footPos_[0].e() - gc_.head(3)) - temp)*2.0,  /// 3 relative foot position with respect to the body COM, expressed in the body frame 3
//              rot_.e().transpose() * ((edgePosWorld_.col(0)+edgePosWorld_.col(1))/2.0 - gc_.head(3)) - temp,
//              rot_.e().transpose() * ((edgePosWorld_.col(2)+edgePosWorld_.col(3))/2.0 - gc_.head(3)) - temp,/// relative edge pos (heel, toe)
              (gc_.segment(8,2)-gcInit_.segment(8,2))*2.0,                          /// 2 ankle output FK
              gv_.segment(7,2)/2e1,                                                 /// 2 ankle output vel
              comToFootLocalFrame_.head(2)*5e0;                                     /// 2 com pos
//              jointFrictions_(0)/2e1,
//              jointFrictions_.tail(2)/4e0;                                           /// 3 joint friction

              /// convert it to float
      ob = valueObDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);
    /// if the contact body is not feet
    for(auto& contact: dhal_->getContacts())
        if ((std::find(footIndices_.begin(), footIndices_.end(), contact.getlocalBodyIndex()) == footIndices_.end())
                and (std::find(exceptionIndices_.begin(), exceptionIndices_.end(), contact.getlocalBodyIndex()) == exceptionIndices_.end())) {
            terminalStack_ += 1;
        }
    if (terminalStack_ > 50){
        prevTerminal_ = true;
        return true;
    }
    terminalReward = -0.f;
    return false;
  }

  void curriculumUpdate() {
      /// for each iteration
      iter_ ++;
      curriculum_ = (double)(iter_)/1000.0; /// 1500 iter -> 1.0
      curriculum_ = (curriculum_ > 0.2) ? 0.2 : curriculum_;

      world_->removeObject(heightMap_);
      heightMap_ = HeightMapSample(world_.get(),iter_%2,curriculum_,gen_,uniDist_);
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
  double terminalRewardCoeff_ = -1e1;
  raisim::ArticulatedSystem* dhal_;

  double pGain_, dGain_;
  Eigen::VectorXd gc_, gv_, genForceTarget_;
  Eigen::Vector<double,16> gcInit_, gcNoise_;
  Eigen::Vector<double,15> gvInit_, gvNoise_, subStepPgain_,subStepDgain_;
  Eigen::Vector<double,3> pTarget_, prevTarget_, prevPrevTarget_, jointFrictions_;
  Eigen::Vector<double,5> preJointVel_; // knee, ankle output, ankle input (1+2+2)
  Eigen::Vector<double,3> jointPgain_, jointDgain_;

  Eigen::Matrix<double,3,3> rotConversion_;
  raisim::Mat<3,3> rot_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_, valueObDouble_, estDouble_, obDoubleLpf_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::vector<size_t> footIndices_, exceptionIndices_;
    /// collision reward
    std::vector<size_t> bodyIndices_;
    /// mass randomization
    std::vector<double> nominalMass_;

  /// additional
  Eigen::Vector3d command_;                     // vx, vy, w
  std::vector<std::string> footJointFrames_;
  std::vector<std::string> hipJointFrames_;
  int footContact_;
  int bodyContact_; // 1
  std::vector<raisim::Vec<3>> footPos_,footVel_, hipJointPos_, refBodyToFoot_;
  std::vector<raisim::Mat<3,3>> footOrientation_;
  raisim::Vec<3> comPos_;
  Eigen::Matrix<double,3,1> comToFootLocalFrame_;

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
  Eigen::Matrix<double,2,2> limitJointVel_;
  Eigen::Matrix<double,1,2> limitTargetVel_;
  Eigen::Matrix<double,1,2> limitFootClearance_;
  Eigen::Matrix<double,1,2> limitFootContact_; // for gait enforcing
  Eigen::Matrix<double,1,2> limitBodyContact_; // for gait enforcing
  Eigen::Matrix<double,1,2> limitCOMpos_; // for gait enforcing
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
  bool prevTerminal_;

  thread_local static std::mt19937 gen_;
  thread_local static std::normal_distribution<double> normDist_;
  thread_local static std::uniform_real_distribution<double> uniDist_;

};
thread_local std::mt19937 raisim::ENVIRONMENT::gen_;
thread_local std::normal_distribution<double> raisim::ENVIRONMENT::normDist_(0.0,1.0);
thread_local std::uniform_real_distribution<double> raisim::ENVIRONMENT::uniDist_(-1.0,1.0);
}

