#include "robotController.h"

using namespace std;
using json = nlohmann::json;
// #define USING_HQP
//=======================================================
// Initialization of classes
//=======================================================
RobotController::RobotController(){
    // Robot Dynamics
    robotDynamics = new RobotDynamics();
    
    // Instantiate task & constraint 
    // AGIROBOT::Task * ptrPelvisPosRpy = new PelvisPosRpy("PelvisPosRpy", 3, nV);
    AGIROBOT::Task * ptrPelvisPosXyz = new PelvisPosXyz("PelvisPosXyz", 3, nV);
    AGIROBOT::Task * ptrTorsoPosRpy = new TorsoPosRpy("TorsoPosRpy", 3, nV);
    // AGIROBOT::Task * ptrTorsoPosXyz = new TorsoPosXyz("TorsoPosXyz", 3, nV);
    AGIROBOT::Task * ptrForce4 = new QuadSoleForce("Force4", NFCC4, nV);
    AGIROBOT::Task * ptrPosition = new QuadSolePosition("Position", NFCC4, nV);
    AGIROBOT::Task * ptrGblVelLimits = new GVLimitation("GVLimitation", 19, nV); 
    AGIROBOT::Constraint * ptrDynamicConsistency = new DynamicConsistency("DynamicConsistency", 6, nV);
    AGIROBOT::Constraint * ptrFrictionCone = new FrictionCone("FrictionCone", 8, nV);
    AGIROBOT::Constraint * ptrJointTorqueSaturation = new JointTorqueSaturation("JointTorqueSaturation", NJ, nV);
    AGIROBOT::Constraint * ptrCenterOfPressure = new CenterOfPressure("CenterOfPressure", 8, nV);
    // AGIROBOT::Task * ptrForce = new QuadSoleForce("Force", NFCC2, nV);
    // AGIROBOT::Task * ptrForceChange = new QuadSoleForceChange("ForceChange", NFCC2, nV);

    ptrFrictionCone->setParameter(std::vector<double>{muStatic, myInfinity});
    ptrJointTorqueSaturation->setParameter(std::vector<double>{jointTauLimit});
    ptrCenterOfPressure->setParameter(std::vector<double>{soleFront, soleBack, 
                                    soleLeft, soleRight, CopFactor, myInfinity});

    // Instantiate the Wbc instance && Add task & constraint to the instance
    // Hqp
#ifdef USING_HQP
    myWbc = new AGIROBOT::HqpWbc(nV, robotDynamics);
    myWbc->addTask(ptrForce4, 0);
    myWbc->addTask(ptrPosition, 0);
    myWbc->addTask(ptrPelvisPosRpy, 0);
    myWbc->addTask(ptrPelvisPosXyz, 0);
    myWbc->addTask(ptrGblVelLimits, 0);
    myWbc->addTask(ptrTorsoPosXyz, 0);
    myWbc->addTask(ptrTorsoPosRpy, 0);
    myWbc->addConstraint(ptrDynamicConsistency, 0);
    myWbc->addConstraint(ptrFrictionCone, 0);
    myWbc->addConstraint(ptrCenterOfPressure, 0);
    myWbc->addConstraint(ptrJointTorqueSaturation, 0);
#else
    myWbc = new AGIROBOT::WqpWbc(nV, robotDynamics);
    // myWbc->addTask(ptrPelvisPosRpy, 0);
    myWbc->addTask(ptrPelvisPosXyz, 0);
    myWbc->addTask(ptrTorsoPosRpy, 0);
    // myWbc->addTask(ptrTorsoPosXyz, 0);
    myWbc->addTask(ptrForce4, 0);
    myWbc->addTask(ptrPosition, 0);
    myWbc->addTask(ptrGblVelLimits, 0);
    myWbc->addConstraint(ptrDynamicConsistency, 0);
    myWbc->addConstraint(ptrFrictionCone, 0);
    myWbc->addConstraint(ptrCenterOfPressure, 0);
    myWbc->addConstraint(ptrJointTorqueSaturation, 0);
#endif
    // Initialize the wbc controller
    myWbc->wbcInit();
    myWbc->displayWbcInformation();
    myWbc->displayResultInformation();
}

RobotController::~RobotController(){
    delete robotDynamics;
    delete myWbc;
    robotDynamics = nullptr;
    myWbc = nullptr;
}

bool RobotController::getValueTauOpt(Eigen::VectorXd &jntTorOpt){
    for (int i = 0; i < nJa; i++){
        jntTorOpt(i) = tauOpt(i);
    }
    return true;
}

bool RobotController::getValueQdd(Eigen::VectorXd &Qdd){
    for (int i = 0; i < nJa; i++){
        Qdd(i) = qDDotOpt(i+6);
    }
    return true;
}

bool RobotController::getValuePosCurrent(Eigen::VectorXd &jntPosCur){
    for (int i = 0; i < nJa; i++){
        jntPosCur(i) = qActuated(i);
    }
    return true;
}
//================================================================
// The main update controller of this frame
//================================================================
bool RobotController::update(double timeCtrlSys, webotsState& robotStateSim){
    time += DT;
    stateEstimation(robotStateSim);
    // motionPlan1();
    // motionPlan2();
    // motionPlan3();
    // motionPlan4();///> Lifting box;
    motionPlan5();///> Lifting box and Large scale;
    taskControl();
    return true;
}

//================================================================
// Use sensor data for state estimation
//================================================================
bool RobotController::stateEstimation(webotsState & robotStateSim){
    // Data from sensor
    qActuated = robotStateSim.jointPosAct;
    qDotActuated = robotStateSim.jointVelAct;
    qGen.tail(nJa) = robotStateSim.jointPosAct;
    qDotGen.tail(nJa) = robotStateSim.jointVelAct;
    qGen.segment(3,3) = robotStateSim.pelvisRpyAct.head(3);
    qDotGen.segment(3,3) = robotStateSim.pelvisDRpyAct.tail(3);

    // Root from forward kinematics; set the stance foot as origin coord.
    Eigen::VectorXd stateTemp = Eigen::VectorXd::Zero(6,1);
    stateTemp = robotDynamics->estPelvisPosVelInWorld(qGen, qDotGen, 0);

#ifdef USING_TWIST
    // Root
    pelvisTwistEst.head(3) = robotStateSim.pelvisRpyAct.head(3);
    pelvisTwistEst.segment(6,3) = robotStateSim.pelvisDRpyAct.tail(3);
    pelvisTwistEst.segment(3,3) = stateTemp.head(3); 
    pelvisTwistEst.tail(3) = stateTemp.tail(3);
    qGen.head(3) = pelvisTwistEst.segment(3,3);
    qDotGen.head(3) = pelvisTwistEst.tail(3);
    // Others
    torsoTwistEst = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 0);
    leftFootTwistEst = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 1);
    rightFootTwistEst = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 2);
    leftArmTwistEst = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 3);
    rightArmTwistEst = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 4);
#else
    rpyPelvisEst = robotStateSim.pelvisRpyAct.head(3);
    rpyDotPelvisEst = robotStateSim.pelvisDRpyAct.tail(3);
    xyzPelvisEst = stateTemp.head(3); 
    xyzDotPelvisEst = stateTemp.tail(3);
    qGen.head(3) = xyzPelvisEst;
    qDotGen.head(3) = xyzDotPelvisEst;

    Eigen::VectorXd poseTemp0 = Eigen::VectorXd::Zero(12,1);
    poseTemp0 = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 0);
    rpyTorsoEst = poseTemp0.head(3);
    xyzTorsoEst = poseTemp0.segment(3,3);
    rpyDotTorsoEst = poseTemp0.segment(6,3);
    xyzDotTorsoEst = poseTemp0.tail(3);

    Eigen::VectorXd footStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd footStateTemp1 = Eigen::VectorXd::Zero(12,1);
    footStateTemp0 = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 1);
    rpyFootEst[0] = footStateTemp0.head(3);
    xyzFootEst[0] = footStateTemp0.segment(3,3);
    rpyDotFootEst[0] = footStateTemp0.segment(6,3);
    xyzDotFootEst[0] = footStateTemp0.tail(3);

    footStateTemp1 = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 2);
    rpyFootEst[1] = footStateTemp1.head(3);
    xyzFootEst[1] = footStateTemp1.segment(3,3);
    rpyDotFootEst[1] = footStateTemp1.segment(6,3);
    xyzDotFootEst[1] = footStateTemp1.tail(3);

    Eigen::VectorXd armStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd armStateTemp1 = Eigen::VectorXd::Zero(12,1);
    armStateTemp0 = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 3);
    rpyArmEst[0] = armStateTemp0.head(3);
    xyzArmEst[0] = armStateTemp0.segment(3,3);
    rpyDotArmEst[0] = armStateTemp0.segment(6,3);
    xyzDotArmEst[0] = armStateTemp0.tail(3);

    armStateTemp1 = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 4);
    rpyArmEst[1] = armStateTemp1.head(3);
    xyzArmEst[1] = armStateTemp1.segment(3,3);
    rpyDotArmEst[1] = armStateTemp1.segment(6,3);
    xyzDotArmEst[1] = armStateTemp1.tail(3);

    robotDynamics->estComPosVel(comPosEst, comVelEst);
    cout << "comPosEst:     " << comPosEst.transpose() << endl;
    cout << "xyzFootEst[0]  " << xyzFootEst[0].transpose() << endl;
    cout << "xyzFootEst[1]  " << xyzFootEst[1].transpose() << endl;
    cout << "xyzPelvisEst   " << xyzPelvisEst.transpose() << endl;
#endif


    // Set initial task reference
    if(flagEstFirst == 0){
        flagEstFirst = 1;  ///< Make sure it only exe. once;

#ifdef USING_TWIST
        pelvisTwistInit = pelvisTwistEst;
        torsoTwistInit = torsoTwistEst;
        leftArmTwistInit =  leftArmTwistEst;
        rightArmTwistInit = rightArmTwistEst;
        leftFootTwistInit = leftFootTwistEst;
        rightFootTwistInit = rightFootTwistEst;
#else
        xyzPelvisInit = xyzPelvisEst;
        rpyPelvisInit = rpyPelvisEst;
        xyzTorsoInit = xyzTorsoEst;
        rpyTorsoInit = rpyTorsoEst;

        rpyFootInit[0] = rpyFootEst[0]; 
        xyzFootInit[0] = xyzFootEst[0]; 
        rpyDotFootInit[0] = rpyDotFootEst[0];
        xyzDotFootInit[0] = xyzDotFootEst[0];

        rpyFootInit[1] = rpyFootEst[1]; 
        xyzFootInit[1] = xyzFootEst[1]; 
        rpyDotFootInit[1] = rpyDotFootEst[1];
        xyzDotFootInit[1] = xyzDotFootEst[1];

        rpyArmInit[0] = rpyArmEst[0];
        xyzArmInit[0] = xyzArmEst[0];
        rpyDotArmInit[0] = rpyDotArmEst[0];
        xyzDotArmInit[0] = xyzDotArmEst[0];

        rpyArmInit[1] = rpyArmEst[1];
        xyzArmInit[1] = xyzArmEst[1];
        rpyDotArmInit[1] = rpyDotArmEst[1];
        xyzDotArmInit[1] = xyzDotArmEst[1];

        comPosInit = comPosEst;
        comVelInit = comVelEst;
#endif
    }

    //===============================================================
    // @todo wrench feedback for interaction @Danny241022
    //===============================================================
    biFootForce6D = robotStateSim.footGrfAct;
    biWristForce6D = robotStateSim.wristItaAct;

    //_____________________________________________________________//
    return true;
}

//================================================================
// Set the reference task values from planning
//================================================================
bool RobotController::motionPlan1(){// @Daniel240523
    
#ifdef USING_TWIST
    if(time <= 1000)
        mPlanPitch = pelvisTwistInit(1)-pms.pitchApt*(sin((pms.motionFrq*time+0.5)*PI)-1);

    pelvisTwistTgt   << 0.0, mPlanPitch, 0.0,
                        pelvisTwistInit(3), pelvisTwistInit(4), pelvisTwistInit(5)+pms.pelvisUpDown*mPlan,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, pms.pelvisUpDown*PI*cos((time+0.5)*PI);

    torsoTwistTgt    << 0.0, mPlanPitch, 0.0,                
                        torsoTwistInit(3), torsoTwistInit(4), torsoTwistInit(5)+pms.pelvisUpDown*mPlan,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, pms.pelvisUpDown*PI*cos((time+0.5)*PI);

    leftFootTwistTgt << leftFootTwistInit.head(3), leftFootTwistInit.segment(3,3),
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    rightFootTwistTgt << rightFootTwistInit.head(3), rightFootTwistInit.segment(3,3),
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    leftArmTwistTgt <<  leftArmTwistInit.head(3),                        
                        leftArmTwistInit(3), leftArmTwistInit(4), leftArmTwistInit(5)+pms.pelvisUpDown*mPlan,
                        Eigen::Vector3d::Zero(),
                        leftArmTwistInit(9), leftArmTwistInit(10), leftArmTwistInit(11)+pms.pelvisUpDown*PI*cos((time+0.5)*PI);

    rightArmTwistTgt << rightArmTwistInit.head(3),
                        rightArmTwistInit(3), rightArmTwistInit(4), rightArmTwistInit(5)+pms.pelvisUpDown*mPlan,
                        Eigen::Vector3d::Zero(),
                        rightArmTwistInit(9), rightArmTwistInit(10), rightArmTwistInit(11)+pms.pelvisUpDown*PI*cos((time+0.5)*PI);
#else
    if(time < 1.0 || time >= 2.0){
        double yyyL{};
        double yyyR{};
        if(time < 1.0){
            mPlan = 0.5*sin((pms.motionFrq*time-0.5)*PI)+0.5;
            mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*time-0.5)*PI);}

        if(time >= 2.0){
            timeS2 = time - 1;
            yyyL = pms.armAside_L;
            yyyR = pms.armAside_R;
            mPlan = 0.5*sin((pms.motionFrq*timeS2-0.5)*PI)+0.5;
            mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS2-0.5)*PI);}

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+pms.pelvisForward*mPlan, 
                        xyzPelvisInit(1),
                        xyzPelvisInit(2)+pms.pelvisUpDown*mPlan;

        xyzDotPelvisTgt << pms.pelvisForward*mPlanDot,
                        0.0,
                        pms.pelvisUpDown*mPlanDot;

        //=============================================================================
        // CoM Adaptator policy @todo
        //=============================================================================
        // Eigen::Vector3d comPosErr = comPosEst - comPosInit;
        // Eigen::Vector3d comVelErr = comVelEst - comVelInit;

        // xyzPelvisTgt = pms.pelvisForward*comPosErr + xyzPelvisInit;
        // xyzPelvisTgt(2) = xyzPelvisInit(2)+pms.pelvisUpDown*mPlan;

        // xyzDotPelvisTgt = pms.pelvisForward*comVelErr;
        // xyzDotPelvisTgt(2) = pms.pelvisUpDown*mPlanDot;
        //=============================================================================

        // Torso
        rpyTorsoTgt <<  0.0, 
                        rpyTorsoInit(1)+pms.pitchApt*mPlan, 
                        0.0;

        rpyDotTorsoTgt <<   0.0, 
                            pms.pitchApt*mPlanDot, 
                            0.0;

        // LeftFoot
        rpyFootTgt[0] = rpyFootInit[0];
        xyzFootTgt[0] = xyzFootInit[0];
        xyzDotFootTgt[0] = Eigen::Vector3d::Zero();
        rpyDotFootTgt[0] = Eigen::Vector3d::Zero();

        // RightFoot
        rpyFootTgt[1] = rpyFootInit[1];
        xyzFootTgt[1] = xyzFootInit[1];
        xyzDotFootTgt[1] = Eigen::Vector3d::Zero();
        rpyDotFootTgt[1] = Eigen::Vector3d::Zero();            

        // LeftArm
        rpyArmTgt[0] = rpyArmInit[0];
        xyzArmTgt[0] << xyzArmInit[0](0)+pms.armForward_L*mPlan,
                        xyzArmInit[0](1)+yyyL*mPlan,
                        xyzArmInit[0](2)+pms.armUpDown_L*mPlan;
        rpyDotArmTgt[0] = Eigen::Vector3d::Zero();
        xyzDotArmTgt[0] << xyzDotArmInit[0](0)+pms.armForward_L*mPlanDot, 
                        xyzDotArmInit[0](1)+yyyL*mPlanDot,
                        xyzDotArmInit[0](2)+pms.armUpDown_L*mPlanDot;

        // RightArm
        rpyArmTgt[1] = rpyArmInit[1];
        xyzArmTgt[1] << xyzArmInit[1](0)+pms.armForward_R*mPlan,
                        xyzArmInit[1](1)+yyyR*mPlan,
                        xyzArmInit[1](2)+pms.armUpDown_R*mPlan;
        rpyDotArmTgt[1] = Eigen::Vector3d::Zero();
        xyzDotArmTgt[1] << xyzDotArmInit[1](0)+pms.armForward_R*mPlanDot, 
                        xyzDotArmInit[1](1)+yyyR*mPlanDot,
                        xyzDotArmInit[1](2)+pms.armUpDown_R*mPlanDot;

    }


    if(time >= 1.0 && time < 2.0){
        timeS1 = time - 1.0;
        mPlan = 0.5*sin((pms.motionFrq*timeS1-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((2*pms.motionFrq*timeS1-0.5)*PI);

        // LeftArm
        rpyArmTgt[0](0) = rpyArmInit[0](0)+pms.armRoll_L*mPlan;
        rpyDotArmTgt[0](0) = rpyDotArmInit[0](0)+pms.armRoll_L*mPlanDot, 
        xyzArmTgt[0](1) = xyzArmInit[0](1)+pms.armAside_L*mPlan;
        xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+pms.armAside_L*mPlanDot;

        // RightArm
        rpyArmTgt[1](0) = rpyArmInit[1](0)+pms.armRoll_R*mPlan;
        rpyDotArmTgt[1](0) = rpyDotArmInit[1](0)+pms.armRoll_R*mPlanDot;
        xyzArmTgt[1](1) = xyzArmInit[1](1)+pms.armAside_R*mPlan;
        xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+pms.armAside_R*mPlanDot;
    }

#endif
    return true;
}

//==================================================================
// The main control function to execute QP calcalation and set solutions
//==================================================================
bool RobotController::taskControl(){ 
    myWbc->updateRobotDynamics(qGen, qDotGen); //# Update Robot Dynamics

#ifdef USING_TWIST //# Calculate References
    Eigen::VectorXd pelvisTwistErr = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd torsoTwistErr = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftFootTwistErr = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightFootTwistErr = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftArmTwistErr = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightArmTwistErr = Eigen::VectorXd::Zero(12);
    pelvisTwistErr = pelvisTwistTgt - pelvisTwistEst;
    torsoTwistErr = torsoTwistTgt - torsoTwistEst;
    leftFootTwistErr = leftFootTwistTgt - leftFootTwistEst;
    rightFootTwistErr = rightFootTwistTgt - rightFootTwistEst;
    leftArmTwistErr = leftArmTwistTgt - leftArmTwistEst;
    rightArmTwistErr = rightArmTwistTgt - rightArmTwistEst;

    pelvisRpyRef = pms.kpPelvisRpyDiag * pelvisTwistErr.segment(0,3)
                + pms.kdPelvisRpyDiag * pelvisTwistErr.segment(6,3);
    pelvisXyzRef = pms.kpPelvisXyzDiag * pelvisTwistErr.segment(3,3)
                + pms.kdPelvisXyzDiag * pelvisTwistErr.segment(9,3);

    torsoRpyRef = pms.kpTorsoRpyDiag * torsoTwistErr.segment(0,3)
                + pms.kdTorsoRpyDiag * torsoTwistErr.segment(6,3);
    torsoXyzRef = pms.kpTorsoXyzDiag * torsoTwistErr.segment(3,3)
                + pms.kdTorsoXyzDiag * torsoTwistErr.segment(9,3);

    footArmPosRef.segment(0,3) = pms.kpFootRpyDiag * leftFootTwistErr.segment(0,3)
                            + pms.kdFootRpyDiag * leftFootTwistErr.segment(6,3);
    footArmPosRef.segment(3,3) = pms.kpFootXyzDiag * leftFootTwistErr.segment(3,3)
                            + pms.kdFootXyzDiag * leftFootTwistErr.segment(9,3);

    footArmPosRef.segment(6,3) = pms.kpFootRpyDiag * rightFootTwistErr.segment(0,3)
                            + pms.kdFootRpyDiag * rightFootTwistErr.segment(6,3);
    footArmPosRef.segment(9,3) = pms.kpFootXyzDiag * rightFootTwistErr.segment(3,3)
                            + pms.kdFootXyzDiag * rightFootTwistErr.segment(9,3);

    footArmPosRef.segment(12,3) = pms.kpArmRpyDiag * leftArmTwistErr.segment(0,3)
                                + pms.kdArmRpyDiag * leftArmTwistErr.segment(6,3);
    footArmPosRef.segment(15,3) = pms.kpArmXyzDiag * leftArmTwistErr.segment(3,3)
                                + pms.kdArmXyzDiag * leftArmTwistErr.segment(9,3);

    footArmPosRef.segment(18,3) = pms.kpArmRpyDiag * rightArmTwistErr.segment(0,3)
                                + pms.kdArmRpyDiag * rightArmTwistErr.segment(6,3);
    footArmPosRef.segment(21,3) = pms.kpArmXyzDiag * rightArmTwistErr.segment(3,3)
                                + pms.kdArmXyzDiag * rightArmTwistErr.segment(9,3);
#else
    pelvisRpyRef = pms.kpPelvisRpyDiag * (rpyPelvisTgt - rpyPelvisEst)
                +  pms.kdPelvisRpyDiag * (rpyDotPelvisTgt - rpyDotPelvisEst);
    pelvisXyzRef = pms.kpPelvisXyzDiag * (xyzPelvisTgt - xyzPelvisEst)
                +  pms.kdPelvisXyzDiag * (xyzDotPelvisTgt - xyzDotPelvisEst);

    torsoRpyRef = pms.kpTorsoRpyDiag * (rpyTorsoTgt - rpyTorsoEst)
                +  pms.kdTorsoRpyDiag * (rpyDotTorsoTgt - rpyDotTorsoEst);
    torsoXyzRef = pms.kpTorsoXyzDiag * (xyzTorsoTgt - xyzTorsoEst) 
                +  pms.kdTorsoXyzDiag * (xyzDotTorsoTgt - xyzDotTorsoEst);    

    footArmPosRef.segment(0,3) = pms.kpFootRpyDiag * (rpyFootTgt[0] - rpyFootEst[0])
                                + pms.kdFootRpyDiag * (rpyDotFootTgt[0] - rpyDotFootEst[0]);
    footArmPosRef.segment(3,3) = pms.kpFootXyzDiag * (xyzFootTgt[0] - xyzFootEst[0])
                                + pms.kdFootXyzDiag * (xyzDotFootTgt[0] - xyzDotFootEst[0]);
    footArmPosRef.segment(6,3) = pms.kpFootRpyDiag * (rpyFootTgt[1] - rpyFootEst[1])
                                + pms.kdFootRpyDiag * (rpyDotFootTgt[1] - rpyDotFootEst[1]);
    footArmPosRef.segment(9,3) = pms.kpFootXyzDiag * (xyzFootTgt[1] - xyzFootEst[1])
                                + pms.kdFootXyzDiag * (xyzDotFootTgt[1] - xyzDotFootEst[1]);
    footArmPosRef.segment(12,3) = pms.kpArmRpyDiag * (rpyArmTgt[0] - rpyArmEst[0])
                                + pms.kdArmRpyDiag * (rpyDotArmTgt[0] - rpyDotArmEst[0]);
    footArmPosRef.segment(15,3) = pms.kpArmXyzDiag * (xyzArmTgt[0] - xyzArmEst[0])
                                + pms.kdArmXyzDiag * (xyzDotArmTgt[0] - xyzDotArmEst[0]);
    footArmPosRef.segment(18,3) = pms.kpArmRpyDiag * (rpyArmTgt[1] - rpyArmEst[1])
                                + pms.kdArmRpyDiag * (rpyDotArmTgt[1] - rpyDotArmEst[1]);
    footArmPosRef.segment(21,3) = pms.kpArmXyzDiag * (xyzArmTgt[1] - xyzArmEst[1])
                                + pms.kdArmXyzDiag * (xyzDotArmTgt[1] - xyzDotArmEst[1]);                                                                                                
#endif
    //============================================================
    // @todo wrench feedback for interaction @Danny241022
    //============================================================
    footArmForceRef = forceOpt; ///< LFtorque, LFforce, RFtorque, RFforce, LWtorque, LWforce, RWtorque, RWforce (8x3)

    footArmForceRef.segment(3,3) = biFootForce6D.segment(0,3);
    footArmForceRef.segment(9,3) = biFootForce6D.segment(3,3);
    footArmForceRef.segment(15,3) = biWristForce6D.segment(0,3);
    footArmForceRef.segment(21,3) = biWristForce6D.segment(3,3);
    // cout << "!!! without force feedback !!!" << endl;

    cout << endl;
    cout << "Data of biWrist force=========================================" << endl;
    cout << biWristForce6D.transpose() << endl;
    // cout << "Data of biFoot force=========================================" << endl;
    // cout << biFootForce6D.transpose() << endl;
    // cout << "==============================================================" << endl;


    //============================================================
    GVLimitationRef = -qDotActuated;
    footArmForceRef = forceOpt;

    // Update task & constraint & bounds        
    // myWbc->updateTask("PelvisPosRpy", pelvisRpyRef, pms.weightPelvisRpy);
    myWbc->updateTask("PelvisPosXyz", pelvisXyzRef, pms.weightPelvisXyz);
    myWbc->updateTask("TorsoPosRpy", torsoRpyRef, pms.weightTorsoRpy);
    // myWbc->updateTask("TorsoPosXyz", torsoXyzRef, pms.weightTorsoXyz);
    myWbc->updateTask("Position", footArmPosRef, pms.weightFootArmPosition);
    myWbc->updateTask("Force4", footArmForceRef, pms.weightFootArmForce);
    myWbc->updateTask("GVLimitation", GVLimitationRef, pms.weightGVLimitation);
    myWbc->updateConstraint("DynamicConsistency");
    myWbc->updateConstraint("FrictionCone");
    myWbc->updateConstraint("CenterOfPressure");
    myWbc->updateConstraint("JointTorqueSaturation");
    lowerbounds(NG+5) = 0.0;
    upperbounds(NG+5) = 1000.0*GRAVITY;
    lowerbounds(NG+11) = 0.0;
    upperbounds(NG+11) = 1000.0*GRAVITY;
    myWbc->updateBound(lowerbounds, upperbounds);
    // WBC solve & progress observation
    myWbc->wbcSolve();
    myWbc->getAuxiliaryDataInt(intData);
    nWsrRes = intData.at(0);
    simpleStatus = intData.at(1);
    //``````````Only for Hqp``````````````````//
    // double Nlevel = myWbc->getNlevel();
    // for (int i = Nlevel; i < Nlevel*2; i++){
    //     simpleStatus += intData.at(i);}
    //________________________________________//
    myWbc->getAuxiliaryDataDouble(doubleData);
    costOpt = doubleData.at(0);
    cpuTimeRes = doubleData.at(1);

    // Get wbc variables output and clac toq 
    if (simpleStatus == 0){
        myWbc->getResultOpt(varOpt);
        qDDotOpt = varOpt.head(nJg);
        forceOpt = varOpt.tail(nFc);
        tauOpt = robotDynamics->eqCstrMatTau * varOpt + robotDynamics->eqCstrMatTauBias;
    }else{
        std::cerr << "QP failed; Exiting the program at time of: " << timeCs 
                    << "  simpleStatus: " << simpleStatus << std::endl; 
        exit(EXIT_FAILURE); 
    }
        // cout << endl 
        // << "tauOpt at timeCs of " << timeCs << "======================================" << endl;
        // akiaPrint1(tauOpt, NJ, 5, 5, 5, 1, 4, 4);
        // cout << "==============================================================" << endl;
    return true;
}

bool RobotController::motionPlan2(){

        mPlan = 0.5*sin((pms.motionFrq*time-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*time-0.5)*PI);

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+pms.pelvisForward*mPlan, 
                        xyzPelvisInit(1)+pms.pelvisAside*mPlan,
                        xyzPelvisInit(2)+pms.pelvisUpDown*mPlan;

        xyzDotPelvisTgt << pms.pelvisForward*mPlanDot,
                        pms.pelvisAside*mPlanDot,
                        pms.pelvisUpDown*mPlanDot;

        // Torso
        rpyTorsoTgt <<  rpyTorsoInit(0)+pms.rollApt*mPlan, 
                        rpyTorsoInit(1)+pms.pitchApt*mPlan, 
                        rpyTorsoInit(2)+pms.yawApt*mPlan;

        rpyDotTorsoTgt <<   pms.rollApt*mPlanDot, 
                            pms.pitchApt*mPlanDot, 
                            pms.yawApt*mPlanDot;

        // LeftFoot
        rpyFootTgt[0] << rpyFootInit[0](0)+pms.footRoll*mPlan,
                        rpyFootInit[0](1)+pms.footPitch*mPlan,
                        rpyFootInit[0](2)+pms.footYaw*mPlan;

        xyzFootTgt[0] << xyzFootInit[0](0)+pms.footForward*mPlan,
                        xyzFootInit[0](1)+pms.footAside*mPlan,
                        xyzFootInit[0](2)+pms.footUpDown*mPlan;

        rpyDotFootTgt[0] << rpyDotFootInit[0](0)+pms.footRoll*mPlanDot, 
                        rpyDotFootInit[0](1)+pms.footPitch*mPlanDot,
                        rpyDotFootInit[0](2)+pms.footYaw*mPlanDot;

        xyzDotFootTgt[0] << xyzDotFootInit[0](0)+pms.footForward*mPlanDot, 
                        xyzDotFootInit[0](1)+pms.footAside*mPlanDot,
                        xyzDotFootInit[0](2)+pms.footUpDown*mPlanDot;

        // RightFoot
        rpyFootTgt[1] << rpyFootInit[1](0)+pms.footRoll*mPlan,
                        rpyFootInit[1](1)+pms.footPitch*mPlan,
                        rpyFootInit[1](2)+pms.footYaw*mPlan;

        xyzFootTgt[1] << xyzFootInit[1](0)+pms.footForward*mPlan,
                        xyzFootInit[1](1)+pms.footAside*mPlan,
                        xyzFootInit[1](2)+pms.footUpDown*mPlan;

        rpyDotFootTgt[1] << rpyDotFootInit[1](0)+pms.footRoll*mPlanDot, 
                        rpyDotFootInit[1](1)+pms.footPitch*mPlanDot,
                        rpyDotFootInit[1](2)+pms.footYaw*mPlanDot;

        xyzDotFootTgt[1] << xyzDotFootInit[1](0)+pms.footForward*mPlanDot, 
                        xyzDotFootInit[1](1)+pms.footAside*mPlanDot,
                        xyzDotFootInit[1](2)+pms.footUpDown*mPlanDot;         

        // LeftArm
        rpyArmTgt[0] << rpyArmInit[0](0)+pms.armRoll_L*mPlan,
                        rpyArmInit[0](1)+pms.armPitch_L*mPlan,
                        rpyArmInit[0](2)+pms.armYaw_L*mPlan;

        xyzArmTgt[0] << xyzArmInit[0](0)+pms.armForward_L*mPlan,
                        xyzArmInit[0](1)+pms.armAside_L*mPlan,
                        xyzArmInit[0](2)+pms.armUpDown_L*mPlan;

        rpyDotArmTgt[0] << rpyDotArmInit[0](0)+pms.armRoll_L*mPlanDot, 
                        rpyDotArmInit[0](1)+pms.armPitch_L*mPlanDot,
                        rpyDotArmInit[0](2)+pms.armYaw_L*mPlanDot;

        xyzDotArmTgt[0] << xyzDotArmInit[0](0)+pms.armForward_L*mPlanDot, 
                        xyzDotArmInit[0](1)+pms.armAside_L*mPlanDot,
                        xyzDotArmInit[0](2)+pms.armUpDown_L*mPlanDot;

        // RightArm
        rpyArmTgt[1] << rpyArmInit[1](0)+pms.armRoll_R*mPlan,
                        rpyArmInit[1](1)+pms.armPitch_R*mPlan,
                        rpyArmInit[1](2)+pms.armYaw_R*mPlan;

        xyzArmTgt[1] << xyzArmInit[1](0)+pms.armForward_R*mPlan,
                        xyzArmInit[1](1)+pms.armAside_R*mPlan,
                        xyzArmInit[1](2)+pms.armUpDown_R*mPlan;

        rpyDotArmTgt[1] << rpyDotArmInit[1](0)+pms.armRoll_R*mPlanDot, 
                        rpyDotArmInit[1](1)+pms.armPitch_R*mPlanDot,
                        rpyDotArmInit[1](2)+pms.armYaw_R*mPlanDot;

        xyzDotArmTgt[1] << xyzDotArmInit[1](0)+pms.armForward_R*mPlanDot, 
                        xyzDotArmInit[1](1)+pms.armAside_R*mPlanDot,
                        xyzDotArmInit[1](2)+pms.armUpDown_R*mPlanDot;

    return true;
}

bool RobotController::motionPlan3(){

        sPlan = sin(pms.sFreq*time*PI);
        sPlanDot = PI*pms.sFreq*cos(pms.sFreq*time*PI);

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+pms.pelvisForward*sPlan, 
                        xyzPelvisInit(1)+pms.pelvisAside*sPlan,
                        xyzPelvisInit(2)+pms.pelvisUpDown*sPlan;

        xyzDotPelvisTgt << pms.pelvisForward*sPlanDot,
                        pms.pelvisAside*sPlanDot,
                        pms.pelvisUpDown*sPlanDot;

        // Torso
        rpyTorsoTgt <<  rpyTorsoInit(0)+pms.rollApt*sPlan, 
                        rpyTorsoInit(1)+pms.pitchApt*sPlan, 
                        rpyTorsoInit(2)+pms.yawApt*sPlan;

        rpyDotTorsoTgt <<   pms.rollApt*sPlanDot, 
                            pms.pitchApt*sPlanDot, 
                            pms.yawApt*sPlanDot;


        // LeftFoot
        rpyFootTgt[0] << rpyFootInit[0](0)+pms.footRoll*sPlan,
                        rpyFootInit[0](1)+pms.footPitch*sPlan,
                        rpyFootInit[0](2)+pms.footYaw*sPlan;

        xyzFootTgt[0] << xyzFootInit[0](0)+pms.footForward*sPlan,
                        xyzFootInit[0](1)+pms.footAside*sPlan,
                        xyzFootInit[0](2)+pms.footUpDown*sPlan;

        rpyDotFootTgt[0] << rpyDotFootInit[0](0)+pms.footRoll*sPlanDot, 
                        rpyDotFootInit[0](1)+pms.footPitch*sPlanDot,
                        rpyDotFootInit[0](2)+pms.footYaw*sPlanDot;

        xyzDotFootTgt[0] << xyzDotFootInit[0](0)+pms.footForward*sPlanDot, 
                        xyzDotFootInit[0](1)+pms.footAside*sPlanDot,
                        xyzDotFootInit[0](2)+pms.footUpDown*sPlanDot;

        // RightFoot
        rpyFootTgt[1] << rpyFootInit[1](0)+pms.footRoll*sPlan,
                        rpyFootInit[1](1)+pms.footPitch*sPlan,
                        rpyFootInit[1](2)+pms.footYaw*sPlan;

        xyzFootTgt[1] << xyzFootInit[1](0)+pms.footForward*sPlan,
                        xyzFootInit[1](1)+pms.footAside*sPlan,
                        xyzFootInit[1](2)+pms.footUpDown*sPlan;

        rpyDotFootTgt[1] << rpyDotFootInit[1](0)+pms.footRoll*sPlanDot, 
                        rpyDotFootInit[1](1)+pms.footPitch*sPlanDot,
                        rpyDotFootInit[1](2)+pms.footYaw*sPlanDot;

        xyzDotFootTgt[1] << xyzDotFootInit[1](0)+pms.footForward*sPlanDot, 
                        xyzDotFootInit[1](1)+pms.footAside*sPlanDot,
                        xyzDotFootInit[1](2)+pms.footUpDown*sPlanDot;           

        // LeftArm
        rpyArmTgt[0] << rpyArmInit[0](0)+pms.armRoll_L*sPlan,
                        rpyArmInit[0](1)+pms.armPitch_L*sPlan,
                        rpyArmInit[0](2)+pms.armYaw_L*sPlan;

        xyzArmTgt[0] << xyzArmInit[0](0)+pms.armForward_L*sPlan,
                        xyzArmInit[0](1)+pms.armAside_L*sPlan,
                        xyzArmInit[0](2)+pms.armUpDown_L*sPlan;

        rpyDotArmTgt[0] << rpyDotArmInit[0](0)+pms.armRoll_L*sPlanDot, 
                        rpyDotArmInit[0](1)+pms.armPitch_L*sPlanDot,
                        rpyDotArmInit[0](2)+pms.armYaw_L*sPlanDot;

        xyzDotArmTgt[0] << xyzDotArmInit[0](0)+pms.armForward_L*sPlanDot, 
                        xyzDotArmInit[0](1)+pms.armAside_L*sPlanDot,
                        xyzDotArmInit[0](2)+pms.armUpDown_L*sPlanDot;

        // RightArm
        rpyArmTgt[1] << rpyArmInit[1](0)+pms.armRoll_R*sPlan,
                        rpyArmInit[1](1)+pms.armPitch_R*sPlan,
                        rpyArmInit[1](2)+pms.armYaw_R*sPlan;

        xyzArmTgt[1] << xyzArmInit[1](0)+pms.armForward_R*sPlan,
                        xyzArmInit[1](1)+pms.armAside_R*sPlan,
                        xyzArmInit[1](2)+pms.armUpDown_R*sPlan;

        rpyDotArmTgt[1] << rpyDotArmInit[1](0)+pms.armRoll_R*sPlanDot, 
                        rpyDotArmInit[1](1)+pms.armPitch_R*sPlanDot,
                        rpyDotArmInit[1](2)+pms.armYaw_R*sPlanDot;

        xyzDotArmTgt[1] << xyzDotArmInit[1](0)+pms.armForward_R*sPlanDot, 
                        xyzDotArmInit[1](1)+pms.armAside_R*sPlanDot,
                        xyzDotArmInit[1](2)+pms.armUpDown_R*sPlanDot;

    return true;
}

bool RobotController::motionPlan4(){

    if(time < 1.0 || (time >= 2.0 && time < 4.0) || time >= 5.0){
        double yyyL{};
        double yyyR{};
        if(time < 1.0){
            yyyL = pms.armAside_L;
            yyyR = pms.armAside_R;
            mPlan = 0.5*sin((pms.motionFrq*time-0.5)*PI)+0.5;
            mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*time-0.5)*PI);
        }

        if(time >= 2.0 && time < 4.0){
            timeS2 = time - 1;
            yyyL = 0;
            yyyR = 0;            
            mPlan = 0.5*sin((pms.motionFrq*timeS2-0.5)*PI)+0.5;
            mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS2-0.5)*PI);
        }

        if(time >= 5.0){
            timeS2 = time-2;         
            mPlan = 0.5*sin((pms.motionFrq*timeS2-0.5)*PI)+0.5;
            mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS2-0.5)*PI);
        }

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+pms.pelvisForward*mPlan, 
                        xyzPelvisInit(1),
                        xyzPelvisInit(2)+pms.pelvisUpDown*mPlan;

        xyzDotPelvisTgt <<  pms.pelvisForward*mPlanDot,
                            0.0,
                            pms.pelvisUpDown*mPlanDot;

        // Torso
        rpyTorsoTgt <<  0.0, 
                        rpyTorsoInit(1)+pms.pitchApt*mPlan, 
                        0.0;

        rpyDotTorsoTgt <<   0.0, 
                            pms.pitchApt*mPlanDot, 
                            0.0;

        // LeftFoot
        rpyFootTgt[0] = rpyFootInit[0];
        xyzFootTgt[0] = xyzFootInit[0];
        xyzDotFootTgt[0] = Eigen::Vector3d::Zero();
        rpyDotFootTgt[0] = Eigen::Vector3d::Zero();

        // RightFoot
        rpyFootTgt[1] = rpyFootInit[1];
        xyzFootTgt[1] = xyzFootInit[1];
        xyzDotFootTgt[1] = Eigen::Vector3d::Zero();
        rpyDotFootTgt[1] = Eigen::Vector3d::Zero();            

        if (time >= 5){
            // LeftArm
            rpyArmTgt[0] = rpyArmInit[0];
            xyzArmTgt[0](0) = xyzArmInit[0](0)+pms.armForward_L*mPlan;
            xyzArmTgt[0](2) = xyzArmInit[0](2)+pms.armUpDown_L*mPlan;

            rpyDotArmTgt[0] = Eigen::Vector3d::Zero();

            xyzDotArmTgt[0](0) = xyzDotArmInit[0](0)+pms.armForward_L*mPlanDot;
            xyzDotArmTgt[0](2) = xyzDotArmInit[0](2)+pms.armUpDown_L*mPlanDot;

            // RightArm
            rpyArmTgt[1] = rpyArmInit[1];

            xyzArmTgt[1](0) = xyzArmInit[1](0)+pms.armForward_R*mPlan;
            xyzArmTgt[1](2) = xyzArmInit[1](2)+pms.armUpDown_R*mPlan;

            rpyDotArmTgt[1] = Eigen::Vector3d::Zero();

            xyzDotArmTgt[1](0) = xyzDotArmInit[1](0)+pms.armForward_R*mPlanDot;
            xyzDotArmTgt[1](2) = xyzDotArmInit[1](2)+pms.armUpDown_R*mPlanDot;

        }else{
            // LeftArm
            rpyArmTgt[0] = rpyArmInit[0];
            xyzArmTgt[0](0) = xyzArmInit[0](0)+pms.armForward_L*mPlan;
            xyzArmTgt[0](1) = xyzArmInit[0](1)+yyyL*mPlan;
            xyzArmTgt[0](2) = xyzArmInit[0](2)+pms.armUpDown_L*mPlan;

            rpyDotArmTgt[0] = Eigen::Vector3d::Zero();

            xyzDotArmTgt[0](0) = xyzDotArmInit[0](0)+pms.armForward_L*mPlanDot;
            xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+yyyL*mPlanDot;
            xyzDotArmTgt[0](2) = xyzDotArmInit[0](2)+pms.armUpDown_L*mPlanDot;

            // RightArm
            rpyArmTgt[1] = rpyArmInit[1];

            xyzArmTgt[1](0) = xyzArmInit[1](0)+pms.armForward_R*mPlan;
            xyzArmTgt[1](1) = xyzArmInit[1](1)+yyyR*mPlan;
            xyzArmTgt[1](2) = xyzArmInit[1](2)+pms.armUpDown_R*mPlan;

            rpyDotArmTgt[1] = Eigen::Vector3d::Zero();

            xyzDotArmTgt[1](0) = xyzDotArmInit[1](0)+pms.armForward_R*mPlanDot;
            xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+yyyR*mPlanDot;
            xyzDotArmTgt[1](2) = xyzDotArmInit[1](2)+pms.armUpDown_R*mPlanDot;
        }
    }

    if(time >= 1.0 && time < 2.0){
        timeS1 = time;
        mPlan = 0.5*sin((pms.motionFrq*timeS1-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((2*pms.motionFrq*timeS1-0.5)*PI);

        // LeftArm
        rpyArmTgt[0](0) = rpyArmInit[0](0)+pms.armRoll_L*mPlan;
        rpyDotArmTgt[0](0) = rpyDotArmInit[0](0)+pms.armRoll_L*mPlanDot, 
        xyzArmTgt[0](1) = xyzArmInit[0](1)+pms.armAside_L*mPlan;
        xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+pms.armAside_L*mPlanDot;

        // RightArm
        rpyArmTgt[1](0) = rpyArmInit[1](0)+pms.armRoll_R*mPlan;
        rpyDotArmTgt[1](0) = rpyDotArmInit[1](0)+pms.armRoll_R*mPlanDot;
        xyzArmTgt[1](1) = xyzArmInit[1](1)+pms.armAside_R*mPlan;
        xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+pms.armAside_R*mPlanDot;
    }

    if(time >= 4.0 && time < 5.0){
        timeS1 = time;
        double yFactor = 1.0;
        mPlan = 0.5*sin((pms.motionFrq*timeS1-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((2*pms.motionFrq*timeS1-0.5)*PI);

        // LeftArm
        rpyArmTgt[0](0) = rpyArmInit[0](0)+pms.armRoll_L*mPlan;
        rpyDotArmTgt[0](0) = rpyDotArmInit[0](0)+pms.armRoll_L*mPlanDot, 
        xyzArmTgt[0](1) = xyzArmInit[0](1)+yFactor*pms.armAside_L*mPlan;
        xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+yFactor*pms.armAside_L*mPlanDot;

        // RightArm
        rpyArmTgt[1](0) = rpyArmInit[1](0)+pms.armRoll_R*mPlan;
        rpyDotArmTgt[1](0) = rpyDotArmInit[1](0)+pms.armRoll_R*mPlanDot;
        xyzArmTgt[1](1) = xyzArmInit[1](1)+yFactor*pms.armAside_R*mPlan;
        xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+yFactor*pms.armAside_R*mPlanDot;
    }

    return true;
}

bool RobotController::motionPlan5(){

    double timeS12, timeS23, timeS34, timeS45, timeS56;

    // Foot static;
    rpyFootTgt[0] = rpyFootInit[0];
    xyzFootTgt[0] = xyzFootInit[0];
    rpyFootTgt[1] = rpyFootInit[1];
    xyzFootTgt[1] = xyzFootInit[1];    
    xyzDotFootTgt[0] = Eigen::Vector3d::Zero();
    rpyDotFootTgt[0] = Eigen::Vector3d::Zero();
    xyzDotFootTgt[1] = Eigen::Vector3d::Zero();
    rpyDotFootTgt[1] = Eigen::Vector3d::Zero();   

    if(time < 1.0){
        mPlan = 0.5*sin((pms.motionFrq*time-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*time-0.5)*PI);
    
        // Pelvis&Torso
        xyzPelvisTgt(0) = xyzPelvisInit(0)+pms.pelvisForward*mPlan;
        xyzPelvisTgt(1) = xyzPelvisInit(1);
        xyzPelvisTgt(2) = xyzPelvisInit(2)+pms.pelvisUpDown*mPlan;
        xyzDotPelvisTgt(0) = pms.pelvisForward*mPlanDot;
        xyzDotPelvisTgt(1) = 0.0;
        xyzDotPelvisTgt(2) = pms.pelvisUpDown*mPlanDot;
        rpyTorsoTgt(0) = rpyTorsoInit(0);
        rpyTorsoTgt(1) = rpyTorsoInit(1)+pms.pitchApt*mPlan;
        rpyTorsoTgt(2) = rpyTorsoInit(2);
        rpyDotTorsoTgt(0) = 0.0;
        rpyDotTorsoTgt(1) = pms.pitchApt*mPlanDot;
        rpyDotTorsoTgt(2) = 0.0;
        // LeftArm
        rpyArmTgt[0] = rpyArmInit[0];
        rpyDotArmTgt[0] = Eigen::Vector3d::Zero();
        xyzArmTgt[0](0) = xyzArmInit[0](0)+pms.armForward_L*mPlan;
        xyzArmTgt[0](1) = xyzArmInit[0](1)+pms.armAside_L*mPlan;
        xyzArmTgt[0](2) = xyzArmInit[0](2)+pms.armUpDown_L*mPlan;
        xyzDotArmTgt[0](0) = xyzDotArmInit[0](0)+pms.armForward_L*mPlanDot;
        xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+pms.armAside_L*mPlanDot;
        xyzDotArmTgt[0](2) = xyzDotArmInit[0](2)+pms.armUpDown_L*mPlanDot;
        // RightArm
        rpyArmTgt[1] = rpyArmInit[1];
        rpyDotArmTgt[1] = Eigen::Vector3d::Zero();
        xyzArmTgt[1](0) = xyzArmInit[1](0)+pms.armForward_R*mPlan;
        xyzArmTgt[1](1) = xyzArmInit[1](1)+pms.armAside_R*mPlan;
        xyzArmTgt[1](2) = xyzArmInit[1](2)+pms.armUpDown_R*mPlan;
        xyzDotArmTgt[1](0) = xyzDotArmInit[1](0)+pms.armForward_R*mPlanDot;
        xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+pms.armUpDown_R*mPlanDot;        
        xyzDotArmTgt[1](2) = xyzDotArmInit[1](2)+pms.armUpDown_R*mPlanDot;

    }
    else if(time >= 1.0 && time < 2.0){
        timeS12 = time;
        mPlan = 0.5*sin((pms.motionFrq*timeS12-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS12-0.5)*PI);

        // LeftArm
        rpyArmTgt[0](0) = rpyArmInit[0](0)+pms.armRoll_L*mPlan;
        rpyDotArmTgt[0](0) = rpyDotArmInit[0](0)+pms.armRoll_L*mPlanDot, 
        xyzArmTgt[0](1) = xyzArmInit[0](1)+pms.armAside_L*mPlan;
        xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+pms.armAside_L*mPlanDot;

        // RightArm
        rpyArmTgt[1](0) = rpyArmInit[1](0)+pms.armRoll_R*mPlan;
        rpyDotArmTgt[1](0) = rpyDotArmInit[1](0)+pms.armRoll_R*mPlanDot;
        xyzArmTgt[1](1) = xyzArmInit[1](1)+pms.armAside_R*mPlan;
        xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+pms.armAside_R*mPlanDot;        
    }
    else if (time >= 2.0 && time < 3.0){
        timeS23 = time-1;
        mPlan = 0.5*sin((pms.motionFrq*timeS23-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS23-0.5)*PI);
    
        // Pelvis&Torso
        xyzPelvisTgt(0) = xyzPelvisInit(0)+pms.pelvisForward*mPlan;
        xyzPelvisTgt(2) = xyzPelvisInit(2)+pms.pelvisUpDown*mPlan;
        xyzDotPelvisTgt(0) = pms.pelvisForward*mPlanDot;
        xyzDotPelvisTgt(2) = pms.pelvisUpDown*mPlanDot;
        rpyTorsoTgt(1) = rpyTorsoInit(1)+pms.pitchApt*mPlan;
        rpyDotTorsoTgt(1) = pms.pitchApt*mPlanDot;
        // LeftArm
        xyzArmTgt[0](0) = xyzArmInit[0](0)+pms.armForward_L*mPlan;
        xyzArmTgt[0](2) = xyzArmInit[0](2)+pms.armUpDown_L*mPlan;
        xyzDotArmTgt[0](0) = xyzDotArmInit[0](0)+pms.armForward_L*mPlanDot;
        xyzDotArmTgt[0](2) = xyzDotArmInit[0](2)+pms.armUpDown_L*mPlanDot;
        // RightArm
        xyzArmTgt[1](0) = xyzArmInit[1](0)+pms.armForward_R*mPlan;
        xyzArmTgt[1](2) = xyzArmInit[1](2)+pms.armUpDown_R*mPlan;
        xyzDotArmTgt[1](0) = xyzDotArmInit[1](0)+pms.armForward_R*mPlanDot;
        xyzDotArmTgt[1](2) = xyzDotArmInit[1](2)+pms.armUpDown_R*mPlanDot;        
    }
    else if (time >= 3.0 && time < 4.0){
        timeS34 = time - 3.0;
        mPlan = 0.5*sin((pms.motionFrq*timeS34-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS34-0.5)*PI);
    
        // LeftArm
        xyzArmTgt[0](2) = xyzArmInit[0](2)+pms.armUpDown_L_Lift*mPlan;
        xyzDotArmTgt[0](2) = xyzDotArmInit[0](2)+pms.armUpDown_L_Lift*mPlanDot;
        // RightArm
        xyzArmTgt[1](2) = xyzArmInit[1](2)+pms.armUpDown_R_Lift*mPlan;
        xyzDotArmTgt[1](2) = xyzDotArmInit[1](2)+pms.armUpDown_R_Lift*mPlanDot;    
    }
    else if (time >= 4.0 && time < 5.0){
        timeS34 = time - 4.0;
        mPlan = 0.5*sin((pms.motionFrq*timeS34-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS34-0.5)*PI);
    
        // LeftArm
        xyzArmTgt[0](0) = xyzArmInit[0](0)+pms.armForward_L_Lift*mPlan;
        xyzDotArmTgt[0](0) = xyzDotArmInit[0](0)+pms.armForward_L_Lift*mPlanDot;
        // RightArm
        xyzArmTgt[1](0) = xyzArmInit[1](0)+pms.armForward_R_Lift*mPlan;
        xyzDotArmTgt[1](0) = xyzDotArmInit[1](0)+pms.armForward_R_Lift*mPlanDot;
        
        // Pelvis&Torso
        xyzPelvisTgt(0) = xyzPelvisInit(0)+pms.pelvisForward_Lift*mPlan;
        xyzDotPelvisTgt(0) = pms.pelvisForward_Lift*mPlanDot;
        rpyTorsoTgt(1) = rpyTorsoInit(1)+pms.pitchApt_Lift*mPlan;
        rpyDotTorsoTgt(1) = pms.pitchApt_Lift*mPlanDot;
    }
    else if(time >= 5.0 && time < 6.0){
        if(!flag1){
            xyzArmInit[0] = xyzArmTgt[0];
            xyzDotArmInit[0] = xyzDotArmTgt[0];
            xyzArmInit[1] = xyzArmTgt[1];
            xyzDotArmInit[1] = xyzDotArmTgt[1];
            flag1 = true;
        }
        timeS34 = time - 5.0;
        mPlan = 0.5*sin((pms.motionFrq*timeS34-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS34-0.5)*PI);
    
        // LeftArm
        xyzArmTgt[0](2) = xyzArmInit[0](2)+pms.pelvisAside_Lift*mPlan;
        xyzDotArmTgt[0](2) = xyzDotArmInit[0](2)+pms.pelvisAside_Lift*mPlanDot;
        // RightArm
        xyzArmTgt[1](2) = xyzArmInit[1](2)+pms.pelvisAside_Lift*mPlan;
        xyzDotArmTgt[1](2) = xyzDotArmInit[1](2)+pms.pelvisAside_Lift*mPlanDot;  
    }
    else if(time >= 6.0 && time < 7.0){
        timeS45 = time - 6.0;
        mPlan = 0.5*sin((pms.motionFrq*timeS45-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS45-0.5)*PI);

        // LeftArm
        rpyArmTgt[0](0) = rpyArmInit[0](0)+pms.armRoll_L_Lift*mPlan;
        rpyDotArmTgt[0](0) = rpyDotArmInit[0](0)+pms.armRoll_L_Lift*mPlanDot, 
        xyzArmTgt[0](1) = xyzArmInit[0](1)+pms.armAside_L_Lift*mPlan;
        xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+pms.armAside_L_Lift*mPlanDot;

        // RightArm
        rpyArmTgt[1](0) = rpyArmInit[1](0)+pms.armRoll_R_Lift*mPlan;
        rpyDotArmTgt[1](0) = rpyDotArmInit[1](0)+pms.armRoll_R_Lift*mPlanDot;
        xyzArmTgt[1](1) = xyzArmInit[1](1)+pms.armAside_R_Lift*mPlan;
        xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+pms.armAside_R_Lift*mPlanDot;
    }
    else if(time >= 7.0 && time < 8.0){
        timeS56 = time-6.0;
        mPlan = 0.5*sin((pms.motionFrq*timeS56-0.5)*PI)+0.5;
        mPlanDot = 0.5*pms.motionFrq*cos((pms.motionFrq*timeS56-0.5)*PI);
    
        // LeftArm
        xyzArmTgt[0](0) = xyzArmInit[0](0)+pms.armForward_L_Lift*mPlan;
        xyzArmTgt[0](1) = xyzArmInit[0](1)+pms.armAside_L_Lift*mPlan;
        xyzArmTgt[0](2) = xyzArmInit[0](2)+pms.armUpDown_L_Lift*mPlan;
        xyzDotArmTgt[0](0) = xyzDotArmInit[0](0)+pms.armForward_L_Lift*mPlanDot;
        xyzDotArmTgt[0](1) = xyzDotArmInit[0](1)+pms.armAside_L_Lift*mPlanDot;
        xyzDotArmTgt[0](2) = xyzDotArmInit[0](2)+pms.armUpDown_L_Lift*mPlanDot;
        // RightArm
        xyzArmTgt[1](0) = xyzArmInit[1](0)+pms.armForward_R_Lift*mPlan;
        xyzArmTgt[1](1) = xyzArmInit[1](1)+pms.armAside_R_Lift*mPlan;
        xyzArmTgt[1](2) = xyzArmInit[1](2)+pms.armUpDown_R_Lift*mPlan;
        xyzDotArmTgt[1](0) = xyzDotArmInit[1](0)+pms.armForward_R_Lift*mPlanDot;
        xyzDotArmTgt[1](1) = xyzDotArmInit[1](1)+pms.armAside_R_Lift*mPlanDot;
        xyzDotArmTgt[1](2) = xyzDotArmInit[1](2)+pms.armUpDown_R_Lift*mPlanDot;
    }else{
        
    }

    return true;
}