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
    AGIROBOT::Task * ptrPelvisPosRpy = new PelvisPosRpy("PelvisPosRpy", 3, nV);
    AGIROBOT::Task * ptrPelvisPosXyz = new PelvisPosXyz("PelvisPosXyz", 3, nV);
    AGIROBOT::Task * ptrTorsoPosRpy = new TorsoPosRpy("TorsoPosRpy", 3, nV);
    AGIROBOT::Task * ptrTorsoPosXyz = new TorsoPosXyz("TorsoPosXyz", 3, nV);
    AGIROBOT::Task * ptrForce4 = new QuadSoleForce("Force4", NFCC4, nV);
    AGIROBOT::Task * ptrForceChange4 = new QuadSoleForceChange("ForceChange4", NFCC4, nV);
    AGIROBOT::Task * ptrPosition = new QuadSolePosition("Position", NFCC4, nV);
    AGIROBOT::Task * ptrDynamic = new FloatingBaseDynamics("Dynamics", 6, nV);
    AGIROBOT::Task * ptrGblVelLimits = new GlobalVelocityLimitation("GlobalVelocityLimitation", 19, nV); 
    // AGIROBOT::Task * ptrForce = new QuadSoleForce("Force", NFCC2, nV);
    // AGIROBOT::Task * ptrForceChange = new QuadSoleForceChange("ForceChange", NFCC2, nV);
    AGIROBOT::Constraint * ptrDynamicConsistency = new DynamicConsistency("DynamicConsistency", 6, nV);
    AGIROBOT::Constraint * ptrFrictionCone = new FrictionCone("FrictionCone", 8, nV);
    AGIROBOT::Constraint * ptrJointTorqueSaturation = new JointTorqueSaturation("JointTorqueSaturation", NJ, nV);
    AGIROBOT::Constraint * ptrCenterOfPressure = new CenterOfPressure("CenterOfPressure", 8, nV);

    ptrFrictionCone->setParameter(std::vector<double>{muStatic, myInfinity});
    ptrJointTorqueSaturation->setParameter(std::vector<double>{jointTauLimit});
    ptrCenterOfPressure->setParameter(std::vector<double>{soleFront, soleBack, soleLeft, soleRight, CopFactor, myInfinity});

    // Instantiate the Wbc instance && Add task & constraint to the instance
    // Hqp
#ifdef USING_HQP
    myWbc = new AGIROBOT::HqpWbc(nV, robotDynamics);
    // myWbc->addTask(ptrGblVelLimits, 2);
    myWbc->addTask(ptrForce4, 2);
    // myWbc->addTask(ptrTorsoPosRpy, 1);
    // myWbc->addTask(ptrTorsoPosXyz, 1);
    myWbc->addTask(ptrPosition, 1);
    myWbc->addTask(ptrPelvisPosRpy, 0);
    myWbc->addTask(ptrPelvisPosXyz, 0);
    // myWbc->addTask(ptrDynamic, 0);
    myWbc->addConstraint(ptrDynamicConsistency, 0);
    myWbc->addConstraint(ptrFrictionCone, 0);
    myWbc->addConstraint(ptrCenterOfPressure, 0);
    myWbc->addConstraint(ptrJointTorqueSaturation, 0);
#else
    // Wqp
    myWbc = new AGIROBOT::WqpWbc(nV, robotDynamics);
    myWbc->addTask(ptrPelvisPosRpy, 0);
    myWbc->addTask(ptrPelvisPosXyz, 0);
    myWbc->addTask(ptrTorsoPosRpy, 0);
    myWbc->addTask(ptrTorsoPosXyz, 0);
    myWbc->addTask(ptrForce4, 0);
    myWbc->addTask(ptrForceChange4, 0);
    myWbc->addTask(ptrPosition, 0);
    myWbc->addTask(ptrDynamic, 0);
    myWbc->addTask(ptrGblVelLimits, 0);
    myWbc->addConstraint(ptrDynamicConsistency, 0);
    myWbc->addConstraint(ptrFrictionCone, 0);
    myWbc->addConstraint(ptrCenterOfPressure, 0);
    myWbc->addConstraint(ptrJointTorqueSaturation, 0);
#endif
    // Initialize the wbc controller
    myWbc->wbcInit();
    myWbc->displayWbcInformation(); // optional
    myWbc->displayResultInformation();// show qp parameter informations 
}

RobotController::~RobotController(){
    delete robotDynamics;
    delete myWbc;
    robotDynamics = nullptr;
    myWbc = nullptr;
}

Eigen::MatrixXd RobotController::diag(const std::vector<double>& diagElement){
    int dim = static_cast<int>(diagElement.size());
    Eigen::MatrixXd diagM = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i != dim; i++){
        diagM(i, i) = diagElement.at(i);
    }
    return diagM;
}


//================================================================
// The main update controller of this frame
//================================================================
bool RobotController::update(double timeCtrlSys, webotsState& robotStateSim){
    // update Time
    timeCs = timeCtrlSys;
    if ( tick > 0){
        time += DT;
    } else {// tick<=0
        tick = 0;
        time = 0.0;
    }

    stateEstimation(robotStateSim);
    motionPlan();
    taskControl();

    // update tick-tack
    tick++;
    if (tick >= std::numeric_limits<int>::max()-1000){
        tick = 1000;    // avoid tick out-of range
    }

    return true;
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
// Use sensor data for state estimation
//================================================================
bool RobotController::stateEstimation(webotsState & robotStateSim){
    // Data from sensor
    rpyPelvisEst = robotStateSim.pelvisRpyAct.head(3);
    rpyDotPelvisEst = robotStateSim.pelvisDRpyAct.tail(3);
    qActuated = robotStateSim.jointPosAct;
    qDotActuated = robotStateSim.jointVelAct;
    groundReactiveForce = robotStateSim.footGrfAct;

    qGen.tail(nJa) = qActuated;
    qDotGen.tail(nJa) = qDotActuated;
    qGen.segment(3,3) = rpyPelvisEst;
    qDotGen.segment(3,3) = rpyDotPelvisEst;

    // Pelvis xyz pos&vel
    Eigen::VectorXd trosoStateTemp = Eigen::VectorXd::Zero(6,1);
    trosoStateTemp = robotDynamics->estPelvisPosVelInWorld(qGen, qDotGen, 0);
    xyzPelvisEst = trosoStateTemp.head(3); 
    xyzDotPelvisEst = trosoStateTemp.tail(3);
    qGen.head(3) = xyzPelvisEst;
    qDotGen.head(3) = xyzDotPelvisEst;

    // Torso pose
    Eigen::VectorXd poseTemp0 = Eigen::VectorXd::Zero(12,1);
    poseTemp0 = robotDynamics->estFootArmPosVelInWorld(qGen, qDotGen, 0);

    rpyTorsoEst = poseTemp0.head(3);
    xyzTorsoEst = poseTemp0.segment(3,3);
    rpyDotTorsoEst = poseTemp0.segment(6,3);
    xyzDotTorsoEst = poseTemp0.tail(3);

    // Foot EE pos&vel
    Eigen::VectorXd footStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd footStateTemp1 = Eigen::VectorXd::Zero(12,1);

    footStateTemp0 = robotDynamics->estFootArmPosVelInWorld(qGen, qDotGen, 1);
    rpyFootEst[0] = footStateTemp0.head(3);
    xyzFootEst[0] = footStateTemp0.segment(3,3);
    rpyDotFootEst[0] = footStateTemp0.segment(6,3);
    xyzDotFootEst[0] = footStateTemp0.tail(3);

    footStateTemp1 = robotDynamics->estFootArmPosVelInWorld(qGen, qDotGen, 2);
    rpyFootEst[1] = footStateTemp1.head(3);
    xyzFootEst[1] = footStateTemp1.segment(3,3);
    rpyDotFootEst[1] = footStateTemp1.segment(6,3);
    xyzDotFootEst[1] = footStateTemp1.tail(3);

    // Arm EE pos&vel
    Eigen::VectorXd armStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd armStateTemp1 = Eigen::VectorXd::Zero(12,1);

    armStateTemp0 = robotDynamics->estFootArmPosVelInWorld(qGen, qDotGen, 3);
    xyzArmEst[0] = armStateTemp0.head(3);
    xyzArmEst[0] = armStateTemp0.segment(3,3);
    rpyDotArmEst[0] = armStateTemp0.segment(6,3);
    xyzDotArmEst[0] = armStateTemp0.tail(3);

    armStateTemp1 = robotDynamics->estFootArmPosVelInWorld(qGen, qDotGen, 4);
    rpyArmEst[1] = armStateTemp1.head(3);
    xyzArmEst[1] = armStateTemp1.segment(3,3);
    rpyDotArmEst[1] = armStateTemp1.segment(6,3);
    xyzDotArmEst[1] = armStateTemp1.tail(3);

    // Set initial task reference
    if(flagEstFirst == 0){
        flagEstFirst = 1;

        xyzPelvisInit = xyzPelvisEst;
        xyzTorsoInit = xyzTorsoEst;

        rpyPelvisInit = rpyPelvisEst;
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
    }

    return true;
}

//================================================================
// Set the reference task values from planning
//================================================================
bool RobotController::motionPlan(){// @Daniel240523

        // Pelvis
        if(time <= 1000){
            dsp = rpyPelvisInit(1)-configParams.pitchApt*(sin((configParams.pitchFrq*time+0.5)*PI)-1);
        }
        xyzPelvisTgt << xyzPelvisInit(0), xyzPelvisInit(1), xyzPelvisInit(2)+configParams.height*(sin((time+0.5)*PI)-1);
        xyzDotPelvisTgt <<  0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);
        rpyPelvisTgt << 0.0, dsp, 0.0;
        rpyDotPelvisTgt << 0.0, 0.0, 0.0;

        // Torso
        xyzTorsoTgt << xyzTorsoInit(0), xyzTorsoInit(1), xyzTorsoInit(2)+configParams.height*(sin((time+0.5)*PI)-1);
        xyzDotTorsoTgt << 0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);
        rpyTorsoTgt << 0.0, dsp, 0.0;
        rpyDotTorsoTgt << 0.0, 0.0, 0.0;
        
        // Foot
        xyzFootTgt[0] = xyzFootInit[0];
        xyzDotFootTgt[0] = Eigen::Vector3d::Zero();
        rpyFootTgt[0] = rpyFootInit[0];
        rpyDotFootTgt[0] = Eigen::Vector3d::Zero();

        xyzFootTgt[1] = xyzFootInit[1];
        xyzDotFootTgt[1] = Eigen::Vector3d::Zero();
        rpyFootTgt[1] = rpyFootInit[1];
        rpyDotFootTgt[1] = Eigen::Vector3d::Zero();

        xyzArmTgt[0] << xyzArmInit[0].x(), xyzArmInit[0].y(), xyzArmInit[0].z()+configParams.height*(sin((time+0.5)*PI)-1);
        xyzDotArmTgt[0] << xyzDotArmInit[0].x(), xyzDotArmInit[0].y(), xyzDotArmInit[0].z()+configParams.height*PI*cos((time+0.5)*PI);
        rpyArmTgt[0] = rpyArmInit[0];
        rpyDotArmTgt[0] = Eigen::Vector3d::Zero();

        xyzArmTgt[1] << xyzArmInit[1].x(), xyzArmInit[1].y(), xyzArmInit[1].z()+configParams.height*(sin((time+0.5)*PI)-1);
        xyzDotArmTgt[1] << xyzDotArmInit[1].x(), xyzDotArmInit[1].y(), xyzDotArmInit[1].z()+configParams.height*PI*cos((time+0.5)*PI);
        rpyArmTgt[1] = rpyArmInit[1];
        rpyDotArmTgt[1] = Eigen::Vector3d::Zero();

    return true;
}

//==================================================================
// The main control function to execute QP calcalation and set solutions
//==================================================================
bool RobotController::taskControl(){
    // Update Robot Dynamics 
    myWbc->updateRobotDynamics(qGen, qDotGen);

    // Calculate References //

    // Pelvis
    pelvisRpyRef = diag(configParams.kpPelvisRpy)*(rpyPelvisTgt - rpyPelvisEst) + diag(configParams.kdPelvisRpy)*(rpyDotPelvisTgt - rpyDotPelvisEst);
    pelvisXyzRef = diag(configParams.kpPelvisXyz)*(xyzPelvisTgt - xyzPelvisEst) + diag(configParams.kdPelvisXyz)*(xyzDotPelvisTgt - xyzDotPelvisEst);
    // Torso
    torsoRpyRef = diag(configParams.kpTorsoRpy)*(rpyTorsoTgt - rpyTorsoEst) + diag(configParams.kdTorsoRpy)*(rpyDotTorsoTgt - rpyDotTorsoEst);
    torsoXyzRef = diag(configParams.kpTorsoXyz)*(xyzTorsoTgt - xyzTorsoEst) + diag(configParams.kdTorsoXyz)*(xyzDotTorsoTgt - xyzDotTorsoEst);
    // left foot
    footArmPosRef.segment(0,3) = diag(configParams.kpFootRpy)*(rpyFootTgt[0] - rpyFootEst[0]) + diag(configParams.kdFootRpy)*(rpyDotFootTgt[0] - rpyDotFootEst[0]);
    footArmPosRef.segment(3,3) = diag(configParams.kpFootXyz)*(xyzFootTgt[0] - xyzFootEst[0]) + diag(configParams.kdFootXyz)*(xyzDotFootTgt[0] - xyzDotFootEst[0]); 
    //right foot 
    footArmPosRef.segment(6,3) = diag(configParams.kpFootRpy)*(rpyFootTgt[1] - rpyFootEst[1]) + diag(configParams.kdFootRpy)*(rpyDotFootTgt[1] - rpyDotFootEst[1]);
    footArmPosRef.segment(9,3) = diag(configParams.kpFootXyz)*(xyzFootTgt[1] - xyzFootEst[1]) + diag(configParams.kdFootXyz)*(xyzDotFootTgt[1] - xyzDotFootEst[1]); 
    // left arm
    footArmPosRef.segment(12,3) = diag(configParams.kpArmRpy)*(rpyArmTgt[0] - rpyArmEst[0]) + diag(configParams.kdArmRpy)*(rpyDotArmTgt[0] - rpyDotArmEst[0]);
    footArmPosRef.segment(15,3) = diag(configParams.kpArmXyz)*(xyzArmTgt[0] - xyzArmEst[0]) + diag(configParams.kdArmXyz)*(xyzDotArmTgt[0] - xyzDotArmEst[0]); 
    //right arm 
    footArmPosRef.segment(18,3) = diag(configParams.kpArmRpy)*(rpyArmTgt[1] - rpyArmEst[1]) + diag(configParams.kdArmRpy)*(rpyDotArmTgt[1] - rpyDotArmEst[1]);
    footArmPosRef.segment(21,3) = diag(configParams.kpArmXyz)*(xyzArmTgt[1] - xyzArmEst[1]) + diag(configParams.kdArmXyz)*(xyzDotArmTgt[1] - xyzDotArmEst[1]);  
    
    GlobalVelocityLimitationRef = -qDotActuated;
    footArmForceRef = forceOpt;
    // footArmForceRef.tail(12) = Eigen::VectorXd::Zero(12);


    // Update task & constraint             
    myWbc->updateTask("PelvisPosRpy", pelvisRpyRef, configParams.weightPelvisOrientation);
    myWbc->updateTask("PelvisPosXyz", pelvisXyzRef, configParams.weightPelvisPosition);
    myWbc->updateTask("TorsoPosRpy", torsoRpyRef, configParams.weightTorsoOrientation);
    myWbc->updateTask("TorsoPosXyz", torsoXyzRef, configParams.weightTorsoPosition);
    myWbc->updateTask("Position", footArmPosRef, configParams.weightFootArmPosition);
    myWbc->updateTask("Force4", footArmForceRef, configParams.weightFootArmForce);
    myWbc->updateTask("GlobalVelocityLimitation", GlobalVelocityLimitationRef, configParams.weightGlobalVelLimitation);
    myWbc->updateTask("Dynamics", floatBaseDynamicRef, configParams.weightFloatBaseDynamic);

    myWbc->updateConstraint("DynamicConsistency");
    myWbc->updateConstraint("FrictionCone");
    myWbc->updateConstraint("CenterOfPressure");
    myWbc->updateConstraint("JointTorqueSaturation");
    // myWbc->updateTask("ForceChange4", footArmForceChangeRef, configParams.weightFootArmForceChange);
    // myWbc->updateTask("ForceChange4", footArmForceChangeRef, weightFootArmForceChange);
    // myWbc->updateTask("ForceChange", footForceChangeRef, weightFootForceChange);
    // myWbc->updateTask("Force", footForceRef, weightFootForce);

    // Update bounds
    lowerbounds(NG+5) = 0.0;
    upperbounds(NG+5) = 1000.0*GRAVITY;
    lowerbounds(NG+11) = 0.0;
    upperbounds(NG+11) = 1000.0*GRAVITY;
    myWbc->updateBound(lowerbounds, upperbounds);

    // WBC solve
    myWbc->wbcSolve();

    // Get some data from solved wbc
    myWbc->getAuxiliaryDataInt(intData);
    nWsrRes = intData.at(0);
    double Nlevel = myWbc->getNlevel();
    for (int i = Nlevel; i < Nlevel*2; i++){
        simpleStatus += intData.at(i);
    }
    myWbc->getAuxiliaryDataDouble(doubleData);
    costOpt = doubleData.at(0);
    cpuTimeRes = doubleData.at(1);

    // Get wbc variables output and clac toq //
    if (simpleStatus == 0){
        myWbc->getResultOpt(varOpt);
        qDDotOpt = varOpt.head(nJg);
        forceOpt = varOpt.tail(nFc);
        tauOpt = robotDynamics->eqCstrMatTau * varOpt + robotDynamics->eqCstrMatTauBias;
    }else{
        std::cerr << "QP failed; Exiting the program at time of: " << timeCs << "  simpleStatus: " << simpleStatus << std::endl;
        exit(EXIT_FAILURE); 
    }
        cout << endl << "tauOpt at timeCs of " << timeCs << endl;
        akiaPrint1(tauOpt, NJ, 5, 5, 5, 1, 4, 4);

    return true;
}