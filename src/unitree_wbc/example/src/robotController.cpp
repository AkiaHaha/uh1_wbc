#include "robotController.h"

using namespace std;
using json = nlohmann::json;
#define USING_HQP
//=======================================================
// Initialization of classes
//=======================================================
RobotController::RobotController(){
    // Robot Dynamics
    robotDynamics = std::make_unique<RobotDynamics>();    
    
    // Instantiate task & constraint 
    auto ptrPelvisRpy = std::make_unique<PelvisRpy>("PelvisRpy", 3, nV);
    auto ptrPelvisXyz = std::make_unique<PelvisXyz>("PelvisXyz", 3, nV);
    auto ptrTorsoRpy = std::make_unique<TorsoRpy>("TorsoRpy", 3, nV);
    auto ptrTorsoXyz  = std::make_unique<TorsoXyz>("TorsoXyz", 3, nV);
    auto ptrFootRpy = std::make_unique<FootRpy>("FootRpy", 3, nV);
    auto ptrFootXyz  = std::make_unique<FootXyz>("FootXyz", 3, nV);
    auto ptrArmRpy = std::make_unique<FootPosZ>("ArmRpy", 1, nV);
    auto ptrArmXyz  = std::make_unique<FootPosZ>("ArmXyz", 1, nV);
    auto ptrGblVelLimits = std::make_unique<FootPosZ>("GVLimitation", 19, nV);

    AGIROBOT::Constraint * ptrDynamicConsistency = new DynamicConsistency("DynamicConsistency", 6, nV);
    AGIROBOT::Constraint * ptrFrictionCone = new FrictionCone("FrictionCone", 8, nV);
    AGIROBOT::Constraint * ptrJointTorqueSaturation = new JointTorqueSaturation("JointTorqueSaturation", NJ19, nV);
    AGIROBOT::Constraint * ptrCenterOfPressure = new CenterOfPressure("CenterOfPressure", 8, nV);

    ptrFrictionCone->setParameter(std::vector<double>{muStatic, myInfinity});
    ptrJointTorqueSaturation->setParameter(std::vector<double>{jointTauLimit});
    ptrCenterOfPressure->setParameter(std::vector<double>{soleFront, soleBack, 
                                    soleLeft, soleRight, CopFactor, myInfinity});

    // Instantiate the Wbc instance && Add task & constraint to the instance
    // Hqp
    myWbc = std::make_unique<AGIROBOT::HqpWbc>(nV, robotDynamics.get());
    myWbc->addTask(ptrPelvisPosXyz.get(), 0);
    myWbc->addTask(ptrGblVelLimits.get(), 0);
    myWbc->addTask(ptrTorsoXyz.get(), 0);
    myWbc->addTask(ptrTorsoRpy.get(), 0);
    myWbc->addTask(ptrPelvisRpy.get(), 0);
    myWbc->addTask(ptrPelvisXyz.get(), 0);
    myWbc->addTask(ptrFootRpy.get(), 0);
    myWbc->addTask(ptrFootXyz.get(), 0);
    myWbc->addTask(ptrArmRpy.get(), 0);
    myWbc->addTask(ptrArmXyz.get(), 0);
    myWbc->addConstraint(ptrDynamicConsistency, 0);
    myWbc->addConstraint(ptrFrictionCone, 0);
    myWbc->addConstraint(ptrCenterOfPressure, 0);
    myWbc->addConstraint(ptrJointTorqueSaturation, 0);

    // Initialize the wbc controller
    myWbc->wbcInit();
    myWbc->displayWbcInformation();
    myWbc->displayResultInformation();
}

RobotController::~RobotController(){
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
    time = timeCtrlSys;
    stateEstimation(robotStateSim);
    motionPlan();
    calcRef();
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
    xyzArmEst[0] = armStateTemp0.head(3);
    xyzArmEst[0] = armStateTemp0.segment(3,3);
    rpyDotArmEst[0] = armStateTemp0.segment(6,3);
    xyzDotArmEst[0] = armStateTemp0.tail(3);

    armStateTemp1 = robotDynamics->estBodyTwistInWorld(qGen, qDotGen, 4);
    rpyArmEst[1] = armStateTemp1.head(3);
    xyzArmEst[1] = armStateTemp1.segment(3,3);
    rpyDotArmEst[1] = armStateTemp1.segment(6,3);
    xyzDotArmEst[1] = armStateTemp1.tail(3);
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
bool RobotController::motionPlan(){// @Daniel240523
    
#ifdef USING_TWIST
    dsp = pelvisTwistInit(1)-configParams.pitchApt*(sin((configParams.pitchFrq*time+0.5)*PI)-1);

    pelvisTwistTgt   << 0.0, dsp, 0.0,
                        pelvisTwistInit(3), pelvisTwistInit(4), pelvisTwistInit(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        0.0, 0.0, 0.0,
                        0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);

    torsoTwistTgt    << 0.0, dsp, 0.0,                
                        torsoTwistInit(3), torsoTwistInit(4), torsoTwistInit(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        0.0, 0.0, 0.0,
                        0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);

    leftFootTwistTgt << leftFootTwistInit.head(3), leftFootTwistInit.segment(3,3),
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    rightFootTwistTgt << rightFootTwistInit.head(3), rightFootTwistInit.segment(3,3),
                        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

    leftArmTwistTgt <<  leftArmTwistInit.head(3),                        
                        leftArmTwistInit(3), leftArmTwistInit(4), leftArmTwistInit(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        Eigen::Vector3d::Zero(),
                        leftArmTwistInit(9), leftArmTwistInit(10), leftArmTwistInit(11)+configParams.height*PI*cos((time+0.5)*PI);

    rightArmTwistTgt << rightArmTwistInit.head(3),
                        rightArmTwistInit(3), rightArmTwistInit(4), rightArmTwistInit(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        Eigen::Vector3d::Zero(),
                        rightArmTwistInit(9), rightArmTwistInit(10), rightArmTwistInit(11)+configParams.height*PI*cos((time+0.5)*PI);
#else
    dsp = rpyPelvisInit(1)-configParams.pitchApt*(sin((configParams.pitchFrq*time+0.5)*PI)-1);

    xyzPelvisTgt << xyzPelvisInit(0), xyzPelvisInit(1), xyzPelvisInit(2)+configParams.height*(sin((time+0.5)*PI)-1);
    xyzDotPelvisTgt <<  0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);
    rpyPelvisTgt << 0.0, dsp, 0.0;
    rpyDotPelvisTgt << 0.0, 0.0, 0.0;

    xyzTorsoTgt << xyzTorsoInit(0), xyzTorsoInit(1), xyzTorsoInit(2)+configParams.height*(sin((time+0.5)*PI)-1);
    xyzDotTorsoTgt << 0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);
    rpyTorsoTgt << 0.0, dsp, 0.0;
    rpyDotTorsoTgt << 0.0, 0.0, 0.0;

    rpyFootTgt[0] = rpyFootInit[0];
    xyzFootTgt[0] = xyzFootInit[0];
    xyzDotFootTgt[0] = Eigen::Vector3d::Zero();
    rpyDotFootTgt[0] = Eigen::Vector3d::Zero();

    rpyFootTgt[1] = rpyFootInit[1];
    xyzFootTgt[1] = xyzFootInit[1];
    xyzDotFootTgt[1] = Eigen::Vector3d::Zero();
    rpyDotFootTgt[1] = Eigen::Vector3d::Zero();

    rpyArmTgt[0] = rpyArmInit[0];
    xyzArmTgt[0] << xyzArmInit[0].x(), xyzArmInit[0].y(), xyzArmInit[0].z()+configParams.height*(sin((time+0.5)*PI)-1);
    rpyDotArmTgt[0] = Eigen::Vector3d::Zero();
    xyzDotArmTgt[0] << xyzDotArmInit[0].x(), xyzDotArmInit[0].y(), xyzDotArmInit[0].z()+configParams.height*PI*cos((time+0.5)*PI);

    xyzArmTgt[1] << xyzArmInit[1].x(), xyzArmInit[1].y(), xyzArmInit[1].z()+configParams.height*(sin((time+0.5)*PI)-1);
    xyzDotArmTgt[1] << xyzDotArmInit[1].x(), xyzDotArmInit[1].y(), xyzDotArmInit[1].z()+configParams.height*PI*cos((time+0.5)*PI);
    rpyArmTgt[1] = rpyArmInit[1];
    rpyDotArmTgt[1] = Eigen::Vector3d::Zero();
#endif
    return true;
}

bool RobotController::calcRef(){

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
}

//==================================================================
// The main control function to execute QP calcalation and set solutions
//==================================================================
bool RobotController::taskControl(){ 

    // Update task & constraint & bounds        
    myWbc->updateTask("PelvisRpy", pelvisRpyRef, configParams.weightPelvisRpy);
    myWbc->updateTask("PelvisXyz", pelvisXyzRef, configParams.weightPelvisXyz);
    myWbc->updateTask("TorsoRpy", torsoRpyRef, configParams.weightTorsoRpy);
    myWbc->updateTask("TorsoXyz", torsoXyzRef, configParams.weightTorsoXyz);
    myWbc->updateTask("GVLimitation", GVLimitationRef, configParams.weightGVLimitation);
    myWbc->updateConstraint("DynamicConsistency");
    myWbc->updateConstraint("FrictionCone");
    myWbc->updateConstraint("CenterOfPressure");
    myWbc->updateConstraint("JointTorqueSaturation");
    lowerbounds(NG25+5) = 0.0;
    upperbounds(NG25+5) = 1000.0*GRAVITY;
    lowerbounds(NG25+11) = 0.0;
    upperbounds(NG25+11) = 1000.0*GRAVITY;
    myWbc->updateBound(lowerbounds, upperbounds);

    // WBC solve & progress observation
    myWbc->wbcSolve();
    myWbc->getAuxiliaryDataInt(intData);
    nWsrRes = intData.at(0);
    simpleStatus = intData.at(1);
    //``````````Only for Hqp``````````````````//
    double Nlevel = myWbc->getNlevel();
    for (int i = Nlevel; i < Nlevel*2; i++){
        simpleStatus += intData.at(i);}
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
        // akiaPrint1(tauOpt, NJ19, 5, 5, 5, 1, 4, 4);
        // cout << "==============================================================" << endl;
    return true;
}