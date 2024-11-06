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
    motionPlan4();
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
        mPlanPitch = pelvisTwistInit(1)-configParams.pitchApt*(sin((configParams.motionFrq*time+0.5)*PI)-1);

    pelvisTwistTgt   << 0.0, mPlanPitch, 0.0,
                        pelvisTwistInit(3), pelvisTwistInit(4), pelvisTwistInit(5)+configParams.pelvisUpDown*mPlan,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, configParams.pelvisUpDown*PI*cos((time+0.5)*PI);

    torsoTwistTgt    << 0.0, mPlanPitch, 0.0,                
                        torsoTwistInit(3), torsoTwistInit(4), torsoTwistInit(5)+configParams.pelvisUpDown*mPlan,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, configParams.pelvisUpDown*PI*cos((time+0.5)*PI);

    leftFootTwistTgt << leftFootTwistInit.head(3), leftFootTwistInit.segment(3,3),
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    rightFootTwistTgt << rightFootTwistInit.head(3), rightFootTwistInit.segment(3,3),
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    leftArmTwistTgt <<  leftArmTwistInit.head(3),                        
                        leftArmTwistInit(3), leftArmTwistInit(4), leftArmTwistInit(5)+configParams.pelvisUpDown*mPlan,
                        Eigen::Vector3d::Zero(),
                        leftArmTwistInit(9), leftArmTwistInit(10), leftArmTwistInit(11)+configParams.pelvisUpDown*PI*cos((time+0.5)*PI);

    rightArmTwistTgt << rightArmTwistInit.head(3),
                        rightArmTwistInit(3), rightArmTwistInit(4), rightArmTwistInit(5)+configParams.pelvisUpDown*mPlan,
                        Eigen::Vector3d::Zero(),
                        rightArmTwistInit(9), rightArmTwistInit(10), rightArmTwistInit(11)+configParams.pelvisUpDown*PI*cos((time+0.5)*PI);
#else
    if(time < 1.0 || time >= 2.0){
        double yyyL{};
        double yyyR{};
        if(time < 1.0){
            mPlan = 0.5*sin((configParams.motionFrq*time-0.5)*PI)+0.5;
            mPlanDot = 0.5*configParams.motionFrq*cos((configParams.motionFrq*time-0.5)*PI);}

        if(time >= 2.0){
            timeS2 = time - 1;
            yyyL = configParams.armAside_L;
            yyyR = configParams.armAside_R;
            mPlan = 0.5*sin((configParams.motionFrq*timeS2-0.5)*PI)+0.5;
            mPlanDot = 0.5*configParams.motionFrq*cos((configParams.motionFrq*timeS2-0.5)*PI);}

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+configParams.pelvisForward*mPlan, 
                        xyzPelvisInit(1),
                        xyzPelvisInit(2)+configParams.pelvisUpDown*mPlan;

        xyzDotPelvisTgt << configParams.pelvisForward*mPlanDot,
                        0.0,
                        configParams.pelvisUpDown*mPlanDot;

        //=============================================================================
        // CoM Adaptator policy @todo
        //=============================================================================
        // Eigen::Vector3d comPosErr = comPosEst - comPosInit;
        // Eigen::Vector3d comVelErr = comVelEst - comVelInit;

        // xyzPelvisTgt = configParams.pelvisForward*comPosErr + xyzPelvisInit;
        // xyzPelvisTgt(2) = xyzPelvisInit(2)+configParams.pelvisUpDown*mPlan;

        // xyzDotPelvisTgt = configParams.pelvisForward*comVelErr;
        // xyzDotPelvisTgt(2) = configParams.pelvisUpDown*mPlanDot;
        //=============================================================================

        // Torso
        rpyTorsoTgt <<  0.0, 
                        rpyTorsoInit(1)+configParams.pitchApt*mPlan, 
                        0.0;

        rpyDotTorsoTgt <<   0.0, 
                            configParams.pitchApt*mPlanDot, 
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
        xyzArmTgt[0] << xyzArmInit[0].x()+configParams.armForward_L*mPlan,
                        xyzArmInit[0].y()+yyyL*mPlan,
                        xyzArmInit[0].z()+configParams.armUpDown_L*mPlan;
        rpyDotArmTgt[0] = Eigen::Vector3d::Zero();
        xyzDotArmTgt[0] << xyzDotArmInit[0].x()+configParams.armForward_L*mPlanDot, 
                        xyzDotArmInit[0].y()+yyyL*mPlanDot,
                        xyzDotArmInit[0].z()+configParams.armUpDown_L*mPlanDot;

        // RightArm
        rpyArmTgt[1] = rpyArmInit[1];
        xyzArmTgt[1] << xyzArmInit[1].x()+configParams.armForward_R*mPlan,
                        xyzArmInit[1].y()+yyyR*mPlan,
                        xyzArmInit[1].z()+configParams.armUpDown_R*mPlan;
        rpyDotArmTgt[1] = Eigen::Vector3d::Zero();
        xyzDotArmTgt[1] << xyzDotArmInit[1].x()+configParams.armForward_R*mPlanDot, 
                        xyzDotArmInit[1].y()+yyyR*mPlanDot,
                        xyzDotArmInit[1].z()+configParams.armUpDown_R*mPlanDot;

    }


    if(time >= 1.0 && time < 2.0){
        timeS1 = time - 1.0;
        mPlan = 0.5*sin((configParams.motionFrq*timeS1-0.5)*PI)+0.5;
        mPlanDot = 0.5*configParams.motionFrq*cos((2*configParams.motionFrq*timeS1-0.5)*PI);

        // LeftArm
        rpyArmTgt[0].x() = rpyArmInit[0].x()+configParams.armRoll_L*mPlan;
        rpyDotArmTgt[0].x() = rpyDotArmInit[0].x()+configParams.armRoll_L*mPlanDot, 
        xyzArmTgt[0].y() = xyzArmInit[0].y()+configParams.armAside_L*mPlan;
        xyzDotArmTgt[0].y() = xyzDotArmInit[0].y()+configParams.armAside_L*mPlanDot;

        // RightArm
        rpyArmTgt[1].x() = rpyArmInit[1].x()+configParams.armRoll_R*mPlan;
        rpyDotArmTgt[1].x() = rpyDotArmInit[1].x()+configParams.armRoll_R*mPlanDot;
        xyzArmTgt[1].y() = xyzArmInit[1].y()+configParams.armAside_R*mPlan;
        xyzDotArmTgt[1].y() = xyzDotArmInit[1].y()+configParams.armAside_R*mPlanDot;
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

    pelvisRpyRef = configParams.kpPelvisRpyDiag * pelvisTwistErr.segment(0,3)
                + configParams.kdPelvisRpyDiag * pelvisTwistErr.segment(6,3);
    pelvisXyzRef = configParams.kpPelvisXyzDiag * pelvisTwistErr.segment(3,3)
                + configParams.kdPelvisXyzDiag * pelvisTwistErr.segment(9,3);

    torsoRpyRef = configParams.kpTorsoRpyDiag * torsoTwistErr.segment(0,3)
                + configParams.kdTorsoRpyDiag * torsoTwistErr.segment(6,3);
    torsoXyzRef = configParams.kpTorsoXyzDiag * torsoTwistErr.segment(3,3)
                + configParams.kdTorsoXyzDiag * torsoTwistErr.segment(9,3);

    footArmPosRef.segment(0,3) = configParams.kpFootRpyDiag * leftFootTwistErr.segment(0,3)
                            + configParams.kdFootRpyDiag * leftFootTwistErr.segment(6,3);
    footArmPosRef.segment(3,3) = configParams.kpFootXyzDiag * leftFootTwistErr.segment(3,3)
                            + configParams.kdFootXyzDiag * leftFootTwistErr.segment(9,3);

    footArmPosRef.segment(6,3) = configParams.kpFootRpyDiag * rightFootTwistErr.segment(0,3)
                            + configParams.kdFootRpyDiag * rightFootTwistErr.segment(6,3);
    footArmPosRef.segment(9,3) = configParams.kpFootXyzDiag * rightFootTwistErr.segment(3,3)
                            + configParams.kdFootXyzDiag * rightFootTwistErr.segment(9,3);

    footArmPosRef.segment(12,3) = configParams.kpArmRpyDiag * leftArmTwistErr.segment(0,3)
                                + configParams.kdArmRpyDiag * leftArmTwistErr.segment(6,3);
    footArmPosRef.segment(15,3) = configParams.kpArmXyzDiag * leftArmTwistErr.segment(3,3)
                                + configParams.kdArmXyzDiag * leftArmTwistErr.segment(9,3);

    footArmPosRef.segment(18,3) = configParams.kpArmRpyDiag * rightArmTwistErr.segment(0,3)
                                + configParams.kdArmRpyDiag * rightArmTwistErr.segment(6,3);
    footArmPosRef.segment(21,3) = configParams.kpArmXyzDiag * rightArmTwistErr.segment(3,3)
                                + configParams.kdArmXyzDiag * rightArmTwistErr.segment(9,3);
#else
    pelvisRpyRef = configParams.kpPelvisRpyDiag * (rpyPelvisTgt - rpyPelvisEst)
                +  configParams.kdPelvisRpyDiag * (rpyDotPelvisTgt - rpyDotPelvisEst);
    pelvisXyzRef = configParams.kpPelvisXyzDiag * (xyzPelvisTgt - xyzPelvisEst)
                +  configParams.kdPelvisXyzDiag * (xyzDotPelvisTgt - xyzDotPelvisEst);

    torsoRpyRef = configParams.kpTorsoRpyDiag * (rpyTorsoTgt - rpyTorsoEst)
                +  configParams.kdTorsoRpyDiag * (rpyDotTorsoTgt - rpyDotTorsoEst);
    torsoXyzRef = configParams.kpTorsoXyzDiag * (xyzTorsoTgt - xyzTorsoEst) 
                +  configParams.kdTorsoXyzDiag * (xyzDotTorsoTgt - xyzDotTorsoEst);    

    footArmPosRef.segment(0,3) = configParams.kpFootRpyDiag * (rpyFootTgt[0] - rpyFootEst[0])
                                + configParams.kdFootRpyDiag * (rpyDotFootTgt[0] - rpyDotFootEst[0]);
    footArmPosRef.segment(3,3) = configParams.kpFootXyzDiag * (xyzFootTgt[0] - xyzFootEst[0])
                                + configParams.kdFootXyzDiag * (xyzDotFootTgt[0] - xyzDotFootEst[0]);
    footArmPosRef.segment(6,3) = configParams.kpFootRpyDiag * (rpyFootTgt[1] - rpyFootEst[1])
                                + configParams.kdFootRpyDiag * (rpyDotFootTgt[1] - rpyDotFootEst[1]);
    footArmPosRef.segment(9,3) = configParams.kpFootXyzDiag * (xyzFootTgt[1] - xyzFootEst[1])
                                + configParams.kdFootXyzDiag * (xyzDotFootTgt[1] - xyzDotFootEst[1]);
    footArmPosRef.segment(12,3) = configParams.kpArmRpyDiag * (rpyArmTgt[0] - rpyArmEst[0])
                                + configParams.kdArmRpyDiag * (rpyDotArmTgt[0] - rpyDotArmEst[0]);
    footArmPosRef.segment(15,3) = configParams.kpArmXyzDiag * (xyzArmTgt[0] - xyzArmEst[0])
                                + configParams.kdArmXyzDiag * (xyzDotArmTgt[0] - xyzDotArmEst[0]);
    footArmPosRef.segment(18,3) = configParams.kpArmRpyDiag * (rpyArmTgt[1] - rpyArmEst[1])
                                + configParams.kdArmRpyDiag * (rpyDotArmTgt[1] - rpyDotArmEst[1]);
    footArmPosRef.segment(21,3) = configParams.kpArmXyzDiag * (xyzArmTgt[1] - xyzArmEst[1])
                                + configParams.kdArmXyzDiag * (xyzDotArmTgt[1] - xyzDotArmEst[1]);                                                                                                
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
    // myWbc->updateTask("PelvisPosRpy", pelvisRpyRef, configParams.weightPelvisRpy);
    myWbc->updateTask("PelvisPosXyz", pelvisXyzRef, configParams.weightPelvisXyz);
    myWbc->updateTask("TorsoPosRpy", torsoRpyRef, configParams.weightTorsoRpy);
    // myWbc->updateTask("TorsoPosXyz", torsoXyzRef, configParams.weightTorsoXyz);
    myWbc->updateTask("Position", footArmPosRef, configParams.weightFootArmPosition);
    myWbc->updateTask("Force4", footArmForceRef, configParams.weightFootArmForce);
    myWbc->updateTask("GVLimitation", GVLimitationRef, configParams.weightGVLimitation);
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

        mPlan = 0.5*sin((configParams.motionFrq*time-0.5)*PI)+0.5;
        mPlanDot = 0.5*configParams.motionFrq*cos((configParams.motionFrq*time-0.5)*PI);

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+configParams.pelvisForward*mPlan, 
                        xyzPelvisInit(1)+configParams.pelvisAside*mPlan,
                        xyzPelvisInit(2)+configParams.pelvisUpDown*mPlan;

        xyzDotPelvisTgt << configParams.pelvisForward*mPlanDot,
                        configParams.pelvisAside*mPlanDot,
                        configParams.pelvisUpDown*mPlanDot;

        // Torso
        rpyTorsoTgt <<  rpyTorsoInit(0)+configParams.rollApt*mPlan, 
                        rpyTorsoInit(1)+configParams.pitchApt*mPlan, 
                        rpyTorsoInit(2)+configParams.yawApt*mPlan;

        rpyDotTorsoTgt <<   configParams.rollApt*mPlanDot, 
                            configParams.pitchApt*mPlanDot, 
                            configParams.yawApt*mPlanDot;

        // LeftFoot
        rpyFootTgt[0] << rpyFootInit[0].x()+configParams.footRoll*mPlan,
                        rpyFootInit[0].y()+configParams.footPitch*mPlan,
                        rpyFootInit[0].z()+configParams.footYaw*mPlan;

        xyzFootTgt[0] << xyzFootInit[0].x()+configParams.footForward*mPlan,
                        xyzFootInit[0].y()+configParams.footAside*mPlan,
                        xyzFootInit[0].z()+configParams.footUpDown*mPlan;

        rpyDotFootTgt[0] << rpyDotFootInit[0].x()+configParams.footRoll*mPlanDot, 
                        rpyDotFootInit[0].y()+configParams.footPitch*mPlanDot,
                        rpyDotFootInit[0].z()+configParams.footYaw*mPlanDot;

        xyzDotFootTgt[0] << xyzDotFootInit[0].x()+configParams.footForward*mPlanDot, 
                        xyzDotFootInit[0].y()+configParams.footAside*mPlanDot,
                        xyzDotFootInit[0].z()+configParams.footUpDown*mPlanDot;

        // RightFoot
        rpyFootTgt[1] << rpyFootInit[1].x()+configParams.footRoll*mPlan,
                        rpyFootInit[1].y()+configParams.footPitch*mPlan,
                        rpyFootInit[1].z()+configParams.footYaw*mPlan;

        xyzFootTgt[1] << xyzFootInit[1].x()+configParams.footForward*mPlan,
                        xyzFootInit[1].y()+configParams.footAside*mPlan,
                        xyzFootInit[1].z()+configParams.footUpDown*mPlan;

        rpyDotFootTgt[1] << rpyDotFootInit[1].x()+configParams.footRoll*mPlanDot, 
                        rpyDotFootInit[1].y()+configParams.footPitch*mPlanDot,
                        rpyDotFootInit[1].z()+configParams.footYaw*mPlanDot;

        xyzDotFootTgt[1] << xyzDotFootInit[1].x()+configParams.footForward*mPlanDot, 
                        xyzDotFootInit[1].y()+configParams.footAside*mPlanDot,
                        xyzDotFootInit[1].z()+configParams.footUpDown*mPlanDot;         

        // LeftArm
        rpyArmTgt[0] << rpyArmInit[0].x()+configParams.armRoll_L*mPlan,
                        rpyArmInit[0].y()+configParams.armPitch_L*mPlan,
                        rpyArmInit[0].z()+configParams.armYaw_L*mPlan;

        xyzArmTgt[0] << xyzArmInit[0].x()+configParams.armForward_L*mPlan,
                        xyzArmInit[0].y()+configParams.armAside_L*mPlan,
                        xyzArmInit[0].z()+configParams.armUpDown_L*mPlan;

        rpyDotArmTgt[0] << rpyDotArmInit[0].x()+configParams.armRoll_L*mPlanDot, 
                        rpyDotArmInit[0].y()+configParams.armPitch_L*mPlanDot,
                        rpyDotArmInit[0].z()+configParams.armYaw_L*mPlanDot;

        xyzDotArmTgt[0] << xyzDotArmInit[0].x()+configParams.armForward_L*mPlanDot, 
                        xyzDotArmInit[0].y()+configParams.armAside_L*mPlanDot,
                        xyzDotArmInit[0].z()+configParams.armUpDown_L*mPlanDot;

        // RightArm
        rpyArmTgt[1] << rpyArmInit[1].x()+configParams.armRoll_R*mPlan,
                        rpyArmInit[1].y()+configParams.armPitch_R*mPlan,
                        rpyArmInit[1].z()+configParams.armYaw_R*mPlan;

        xyzArmTgt[1] << xyzArmInit[1].x()+configParams.armForward_R*mPlan,
                        xyzArmInit[1].y()+configParams.armAside_R*mPlan,
                        xyzArmInit[1].z()+configParams.armUpDown_R*mPlan;

        rpyDotArmTgt[1] << rpyDotArmInit[1].x()+configParams.armRoll_R*mPlanDot, 
                        rpyDotArmInit[1].y()+configParams.armPitch_R*mPlanDot,
                        rpyDotArmInit[1].z()+configParams.armYaw_R*mPlanDot;

        xyzDotArmTgt[1] << xyzDotArmInit[1].x()+configParams.armForward_R*mPlanDot, 
                        xyzDotArmInit[1].y()+configParams.armAside_R*mPlanDot,
                        xyzDotArmInit[1].z()+configParams.armUpDown_R*mPlanDot;

    return true;
}


bool RobotController::motionPlan3(){

        sPlan = sin(configParams.sFreq*time*PI);
        sPlanDot = PI*configParams.sFreq*cos(configParams.sFreq*time*PI);

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+configParams.pelvisForward*sPlan, 
                        xyzPelvisInit(1)+configParams.pelvisAside*sPlan,
                        xyzPelvisInit(2)+configParams.pelvisUpDown*sPlan;

        xyzDotPelvisTgt << configParams.pelvisForward*sPlanDot,
                        configParams.pelvisAside*sPlanDot,
                        configParams.pelvisUpDown*sPlanDot;

        // Torso
        rpyTorsoTgt <<  rpyTorsoInit(0)+configParams.rollApt*sPlan, 
                        rpyTorsoInit(1)+configParams.pitchApt*sPlan, 
                        rpyTorsoInit(2)+configParams.yawApt*sPlan;

        rpyDotTorsoTgt <<   configParams.rollApt*sPlanDot, 
                            configParams.pitchApt*sPlanDot, 
                            configParams.yawApt*sPlanDot;


        // LeftFoot
        rpyFootTgt[0] << rpyFootInit[0].x()+configParams.footRoll*sPlan,
                        rpyFootInit[0].y()+configParams.footPitch*sPlan,
                        rpyFootInit[0].z()+configParams.footYaw*sPlan;

        xyzFootTgt[0] << xyzFootInit[0].x()+configParams.footForward*sPlan,
                        xyzFootInit[0].y()+configParams.footAside*sPlan,
                        xyzFootInit[0].z()+configParams.footUpDown*sPlan;

        rpyDotFootTgt[0] << rpyDotFootInit[0].x()+configParams.footRoll*sPlanDot, 
                        rpyDotFootInit[0].y()+configParams.footPitch*sPlanDot,
                        rpyDotFootInit[0].z()+configParams.footYaw*sPlanDot;

        xyzDotFootTgt[0] << xyzDotFootInit[0].x()+configParams.footForward*sPlanDot, 
                        xyzDotFootInit[0].y()+configParams.footAside*sPlanDot,
                        xyzDotFootInit[0].z()+configParams.footUpDown*sPlanDot;

        // RightFoot
        rpyFootTgt[1] << rpyFootInit[1].x()+configParams.footRoll*sPlan,
                        rpyFootInit[1].y()+configParams.footPitch*sPlan,
                        rpyFootInit[1].z()+configParams.footYaw*sPlan;

        xyzFootTgt[1] << xyzFootInit[1].x()+configParams.footForward*sPlan,
                        xyzFootInit[1].y()+configParams.footAside*sPlan,
                        xyzFootInit[1].z()+configParams.footUpDown*sPlan;

        rpyDotFootTgt[1] << rpyDotFootInit[1].x()+configParams.footRoll*sPlanDot, 
                        rpyDotFootInit[1].y()+configParams.footPitch*sPlanDot,
                        rpyDotFootInit[1].z()+configParams.footYaw*sPlanDot;

        xyzDotFootTgt[1] << xyzDotFootInit[1].x()+configParams.footForward*sPlanDot, 
                        xyzDotFootInit[1].y()+configParams.footAside*sPlanDot,
                        xyzDotFootInit[1].z()+configParams.footUpDown*sPlanDot;           

        // LeftArm
        rpyArmTgt[0] << rpyArmInit[0].x()+configParams.armRoll_L*sPlan,
                        rpyArmInit[0].y()+configParams.armPitch_L*sPlan,
                        rpyArmInit[0].z()+configParams.armYaw_L*sPlan;

        xyzArmTgt[0] << xyzArmInit[0].x()+configParams.armForward_L*sPlan,
                        xyzArmInit[0].y()+configParams.armAside_L*sPlan,
                        xyzArmInit[0].z()+configParams.armUpDown_L*sPlan;

        rpyDotArmTgt[0] << rpyDotArmInit[0].x()+configParams.armRoll_L*sPlanDot, 
                        rpyDotArmInit[0].y()+configParams.armPitch_L*sPlanDot,
                        rpyDotArmInit[0].z()+configParams.armYaw_L*sPlanDot;

        xyzDotArmTgt[0] << xyzDotArmInit[0].x()+configParams.armForward_L*sPlanDot, 
                        xyzDotArmInit[0].y()+configParams.armAside_L*sPlanDot,
                        xyzDotArmInit[0].z()+configParams.armUpDown_L*sPlanDot;

        // RightArm
        rpyArmTgt[1] << rpyArmInit[1].x()+configParams.armRoll_R*sPlan,
                        rpyArmInit[1].y()+configParams.armPitch_R*sPlan,
                        rpyArmInit[1].z()+configParams.armYaw_R*sPlan;

        xyzArmTgt[1] << xyzArmInit[1].x()+configParams.armForward_R*sPlan,
                        xyzArmInit[1].y()+configParams.armAside_R*sPlan,
                        xyzArmInit[1].z()+configParams.armUpDown_R*sPlan;

        rpyDotArmTgt[1] << rpyDotArmInit[1].x()+configParams.armRoll_R*sPlanDot, 
                        rpyDotArmInit[1].y()+configParams.armPitch_R*sPlanDot,
                        rpyDotArmInit[1].z()+configParams.armYaw_R*sPlanDot;

        xyzDotArmTgt[1] << xyzDotArmInit[1].x()+configParams.armForward_R*sPlanDot, 
                        xyzDotArmInit[1].y()+configParams.armAside_R*sPlanDot,
                        xyzDotArmInit[1].z()+configParams.armUpDown_R*sPlanDot;

    return true;
}

bool RobotController::motionPlan4(){

    if(time < 1.0 || time >= 2.0){
        double yyyL{};
        double yyyR{};
        if(time < 1.0){
            yyyL = configParams.armAside_L;
            yyyR = configParams.armAside_R;
            mPlan = 0.5*sin((configParams.motionFrq*time-0.5)*PI)+0.5;
            mPlanDot = 0.5*configParams.motionFrq*cos((configParams.motionFrq*time-0.5)*PI);}

        if(time >= 2.0){
            timeS2 = time - 1;
            yyyL = 0;
            yyyR = 0;            
            mPlan = 0.5*sin((configParams.motionFrq*timeS2-0.5)*PI)+0.5;
            mPlanDot = 0.5*configParams.motionFrq*cos((configParams.motionFrq*timeS2-0.5)*PI);}

        // Pelvis
        xyzPelvisTgt << xyzPelvisInit(0)+configParams.pelvisForward*mPlan, 
                        xyzPelvisInit(1),
                        xyzPelvisInit(2)+configParams.pelvisUpDown*mPlan;

        xyzDotPelvisTgt <<  configParams.pelvisForward*mPlanDot,
                            0.0,
                            configParams.pelvisUpDown*mPlanDot;

        // Torso
        rpyTorsoTgt <<  0.0, 
                        rpyTorsoInit(1)+configParams.pitchApt*mPlan, 
                        0.0;

        rpyDotTorsoTgt <<   0.0, 
                            configParams.pitchApt*mPlanDot, 
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
        xyzArmTgt[0] << xyzArmInit[0].x()+configParams.armForward_L*mPlan,
                        xyzArmInit[0].y()+yyyL*mPlan,
                        xyzArmInit[0].z()+configParams.armUpDown_L*mPlan;
        rpyDotArmTgt[0] = Eigen::Vector3d::Zero();
        xyzDotArmTgt[0] << xyzDotArmInit[0].x()+configParams.armForward_L*mPlanDot, 
                        xyzDotArmInit[0].y()+yyyL*mPlanDot,
                        xyzDotArmInit[0].z()+configParams.armUpDown_L*mPlanDot;

        // RightArm
        rpyArmTgt[1] = rpyArmInit[1];
        xyzArmTgt[1] << xyzArmInit[1].x()+configParams.armForward_R*mPlan,
                        xyzArmInit[1].y()+yyyR*mPlan,
                        xyzArmInit[1].z()+configParams.armUpDown_R*mPlan;
        rpyDotArmTgt[1] = Eigen::Vector3d::Zero();
        xyzDotArmTgt[1] << xyzDotArmInit[1].x()+configParams.armForward_R*mPlanDot, 
                        xyzDotArmInit[1].y()+yyyR*mPlanDot,
                        xyzDotArmInit[1].z()+configParams.armUpDown_R*mPlanDot;

    }


    if(time >= 1.0 && time < 2.0){
        timeS1 = time;
        mPlan = 0.5*sin((configParams.motionFrq*timeS1-0.5)*PI)+0.5;
        mPlanDot = 0.5*configParams.motionFrq*cos((2*configParams.motionFrq*timeS1-0.5)*PI);

        // LeftArm
        rpyArmTgt[0].x() = rpyArmInit[0].x()+configParams.armRoll_L*mPlan;
        rpyDotArmTgt[0].x() = rpyDotArmInit[0].x()+configParams.armRoll_L*mPlanDot, 
        xyzArmTgt[0].y() = xyzArmInit[0].y()+configParams.armAside_L*mPlan;
        xyzDotArmTgt[0].y() = xyzDotArmInit[0].y()+configParams.armAside_L*mPlanDot;

        // RightArm
        rpyArmTgt[1].x() = rpyArmInit[1].x()+configParams.armRoll_R*mPlan;
        rpyDotArmTgt[1].x() = rpyDotArmInit[1].x()+configParams.armRoll_R*mPlanDot;
        xyzArmTgt[1].y() = xyzArmInit[1].y()+configParams.armAside_R*mPlan;
        xyzDotArmTgt[1].y() = xyzDotArmInit[1].y()+configParams.armAside_R*mPlanDot;
    }
    return true;
}