#include <iostream>
#include <fstream>

#include "robotController.h"
#include "robotDynamics.h"
#include "robotDynamicsBiped.h"
#include "webotsInterface.h"

#include "json.hpp"

using namespace std;
using json = nlohmann::json;
// #define USING_HQP

//=======================================================
// Initialization of classes
//=======================================================
RobotController::RobotController(){
    // Robot Dynamics
    robotDynamics = new RobotDynamicsBiped();
    
    // Instantiate task & constraint 
    AGIROBOT::Task * ptrBipedPelvisRpy = new BipedPelvisRpy("BipedPelvisRpy", 3, nV);
    AGIROBOT::Task * ptrBipedPelvisXyz = new BipedPelvisXyz("BipedPelvisXyz", 3, nV);
    AGIROBOT::Task * ptrBipedTorsoRpy = new BipedTorsoRpy("BipedTorsoRpy", 3, nV);
    AGIROBOT::Task * ptrBipedTorsoXyz = new BipedTorsoXyz("BipedTorsoXyz", 3, nV);
    AGIROBOT::Task * ptrForce4 = new QuadSoleForce("Force4", NFCC4, nV);
    AGIROBOT::Task * ptrForceChange4 = new QuadSoleForceChange("ForceChange4", NFCC4, nV);
    AGIROBOT::Task * ptrPosition = new QuadSolePosition("Position", NFCC4, nV);
    AGIROBOT::Task * ptrDynamic = new BipedFloatingBaseDynamics("Dynamics", 6, nV);
    AGIROBOT::Task * ptrGblVelLimits = new GlobalVelocityLimitation("GlobalVelocityLimitation", 19, nV); 
    // AGIROBOT::Task * ptrForce = new QuadSoleForce("Force", NFCC2, nV);
    // AGIROBOT::Task * ptrForceChange = new QuadSoleForceChange("ForceChange", NFCC2, nV);
    AGIROBOT::Constraint * ptrBipedDynamicConsistency = new BipedDynamicConsistency("BipedDynamicConsistency", 6, nV);
    AGIROBOT::Constraint * ptrBipedFrictionCone = new BipedFrictionCone("BipedFrictionCone", 8, nV);
    AGIROBOT::Constraint * ptrBipedJointTorqueSaturation = new BipedJointTorqueSaturation("BipedJointTorqueSaturation", NJ, nV);
    AGIROBOT::Constraint * ptrBipedCenterOfPressure = new BipedCenterOfPressure("BipedCenterOfPressure", 8, nV);

    ptrBipedFrictionCone->setParameter(std::vector<double>{muStatic, myInfinity});
    ptrBipedJointTorqueSaturation->setParameter(std::vector<double>{jointTauLimit});
    ptrBipedCenterOfPressure->setParameter(std::vector<double>{soleFront, soleBack, soleLeft, soleRight, CopFactor, myInfinity});

    // Instantiate the Wbc instance && Add task & constraint to the instance
    // Hqp
#ifdef USING_HQP
    myWbc = new AGIROBOT::HqpWbc(nV, robotDynamics);
    // myWbc->addTask(ptrGblVelLimits, 2);
    myWbc->addTask(ptrForce4, 2);
    // myWbc->addTask(ptrBipedTorsoRpy, 1);
    // myWbc->addTask(ptrBipedTorsoXyz, 1);
    myWbc->addTask(ptrPosition, 1);
    myWbc->addTask(ptrBipedPelvisRpy, 0);
    myWbc->addTask(ptrBipedPelvisXyz, 0);
    // myWbc->addTask(ptrDynamic, 0);
    myWbc->addConstraint(ptrBipedDynamicConsistency, 0);
    myWbc->addConstraint(ptrBipedFrictionCone, 0);
    myWbc->addConstraint(ptrBipedCenterOfPressure, 0);
    myWbc->addConstraint(ptrBipedJointTorqueSaturation, 0);
#else
    // Wqp
    myWbc = new AGIROBOT::WqpWbc(nV, robotDynamics);
    myWbc->addTask(ptrBipedPelvisRpy, 0);
    myWbc->addTask(ptrBipedPelvisXyz, 0);
    myWbc->addTask(ptrBipedTorsoRpy, 0);
    myWbc->addTask(ptrBipedTorsoXyz, 0);
    myWbc->addTask(ptrForce4, 0);
    myWbc->addTask(ptrForceChange4, 0);
    myWbc->addTask(ptrPosition, 0);
    myWbc->addTask(ptrDynamic, 0);
    myWbc->addTask(ptrGblVelLimits, 0);
    myWbc->addConstraint(ptrBipedDynamicConsistency, 0);
    myWbc->addConstraint(ptrBipedFrictionCone, 0);
    myWbc->addConstraint(ptrBipedCenterOfPressure, 0);
    myWbc->addConstraint(ptrBipedJointTorqueSaturation, 0);
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
    qActuated = robotStateSim.jointPosAct;
    qDotActuated = robotStateSim.jointVelAct;
    groundReactiveForce = robotStateSim.footGrfAct;
    qGen.tail(nJa) = qActuated;
    qDotGen.tail(nJa) = qDotActuated;
    qGen.segment(3,3) = robotStateSim.pelvisRpyAct;
    qDotGen.segment(3,3) = robotStateSim.pelvisDRpyAct;

    // Pelvis 
    Eigen::VectorXd stateTemp = Eigen::VectorXd::Zero(6,1);
    stateTemp = robotDynamics->estPelvisPosVelInWorld(qGen, qDotGen, 0);
    twistEstPelvis << robotStateSim.pelvisRpyAct, 
                    stateTemp.head(3), 
                    robotStateSim.pelvisDRpyAct, 
                    stateTemp.tail(3);

    // Update general joint params.
    qGen.head(3) = twistEstPelvis.segment(3,3);
    qDotGen.head(3) = twistEstPelvis.tail(3);

    // Torso, foot, arm;
    twistEstTorso = robotDynamics->estPointTwistWorld(qGen, qDotGen, 0);
    twistEstLeftFoot = robotDynamics->estPointTwistWorld(qGen, qDotGen, 1);
    twistEstRightFoot = robotDynamics->estPointTwistWorld(qGen, qDotGen, 2);
    twistEstLeftArm = robotDynamics->estPointTwistWorld(qGen, qDotGen, 3);
    twistEstRightArm = robotDynamics->estPointTwistWorld(qGen, qDotGen, 4);

    // Set initial task reference
    if(flagEstFirst == 0){
        flagEstFirst = 1;
        twistInitPelvis = twistEstPelvis;
        twistInitTorso = twistEstTorso;
        twistInitLeftFoot = twistEstLeftFoot;
        twistInitRightFoot = twistEstRightFoot;
        twistTgtLeftArm = twistEstLeftArm;
        twistTgtRightArm = twistEstRightArm;
    }

    return true;
}

//================================================================
// Set the reference task values from planning
//================================================================

bool RobotController::motionPlan(){// @Daniel240523
        // The displacement to generate motion using sin() function;
        if(time <= 1000){
            dsp = twistInitPelvis(1)-configParams.pitchApt*(sin((configParams.pitchFrq*time+0.5)*PI)-1);
        }

        // Set targets
        twistTgtPelvis << 0.0, dsp, 0.0,
                        twistInitPelvis(3), twistInitPelvis(4), twistInitPelvis(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        0.0, 0.0, 0.0,
                        0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);
        twistTgtTorso << 0.0, dsp, 0.0,
                        twistInitTorso(3), twistInitTorso(4), twistInitTorso(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        0.0, 0.0, 0.0,
                        0.0, 0.0, configParams.height*PI*cos((time+0.5)*PI);

        twistTgtLeftFoot << twistInitLeftFoot.head(6),
                        Eigen::VectorXd::Zero(6);

        twistTgtRightFoot << twistInitRightFoot.head(6),
                        Eigen::VectorXd::Zero(6);

        twistTgtLeftArm << twistInitLeftArm.head(5),
                        twistInitLeftArm(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        Eigen::Vector3d::Zero(),
                        twistInitLeftArm.segment(9,2),
                        twistInitLeftArm(11)+configParams.height*PI*cos((time+0.5)*PI);

        twistTgtRightArm << twistInitRightArm.head(5),
                        twistInitRightArm(5)+configParams.height*(sin((time+0.5)*PI)-1),
                        Eigen::Vector3d::Zero(),
                        twistInitRightArm.segment(9,2), 
                        twistInitRightArm(11)+configParams.height*PI*cos((time+0.5)*PI);
    return true;
}

//==================================================================
// The main control function to execute QP calcalation and set solutions
//==================================================================
bool RobotController::taskControl(){
    // Update Robot Dynamics 
    myWbc->updateRobotDynamics(qGen, qDotGen);

    // Calculate References 

    //  Pelvis
    pelvisRpyRef = diag(configParams.kpPelvisRpy)*(twistTgtPelvis.head(3) - twistEstPelvis.head(3)) 
                    + diag(configParams.kdPelvisRpy)*(twistTgtPelvis.segment(6,3) - twistEstPelvis.segment(6,3));
    pelvisXyzRef = diag(configParams.kpPelvisXyz)*(twistTgtPelvis.segment(3,3) - twistEstPelvis.segment(3,3)) 
                    + diag(configParams.kdPelvisXyz)*(twistTgtPelvis.tail(3) - twistEstPelvis.tail(3));
    //  Torso
    torsoRpyRef = diag(configParams.kpTorsoRpy)*(twistTgtTorso.head(3) - twistEstTorso.head(3)) 
                    + diag(configParams.kdTorsoRpy)*(twistTgtTorso.segment(6,3) - twistEstTorso.segment(6,3));
    torsoXyzRef = diag(configParams.kpTorsoXyz)*(twistTgtTorso.segment(3,3) - twistEstTorso.segment(3,3)) 
                    + diag(configParams.kdTorsoXyz)*(twistTgtTorso.tail(3) - twistEstTorso.tail(3));

    //  left foot
    footArmPoseRef.segment(0,3) = diag(configParams.kpFootRpy)*(twistTgtLeftFoot.head(3) - twistEstLeftFoot.head(3)) 
                                + diag(configParams.kdFootRpy)*(twistTgtLeftFoot.segment(6,3) - twistEstLeftFoot.segment(6,3));
    footArmPoseRef.segment(3,3) = diag(configParams.kpFootXyz)*(twistTgtLeftFoot.segment(3,3) - twistEstLeftFoot.segment(3,3)) 
                                + diag(configParams.kdFootXyz)*(twistTgtLeftFoot.tail(3) - twistEstLeftFoot.tail(3)); 
    //  right foot 
    footArmPoseRef.segment(6,3) = diag(configParams.kpFootRpy)*(twistTgtRightFoot.head(3) - twistEstRightFoot.head(3)) 
                                + diag(configParams.kdFootRpy)*(twistTgtRightFoot.segment(6,3) - twistEstRightFoot.segment(6,3));
    footArmPoseRef.segment(9,3) = diag(configParams.kpFootXyz)*(twistTgtRightFoot.segment(3,3) - twistEstRightFoot.segment(3,3)) 
                                + diag(configParams.kdFootXyz)*(twistTgtRightFoot.tail(3) - twistEstRightFoot.tail(3)); 

    //  left arm
    footArmPoseRef.segment(12,3) = diag(configParams.kpArmRpy)*(twistTgtLeftArm.head(3) - twistEstLeftArm.head(3)) 
                                + diag(configParams.kdArmRpy)*(twistTgtLeftArm.segment(6,3) - twistEstLeftArm.segment(6,3));
    footArmPoseRef.segment(15,3) = diag(configParams.kpArmXyz)*(twistTgtLeftArm.segment(3,3) - twistEstLeftArm.segment(3,3)) 
                                + diag(configParams.kdArmXyz)*(twistTgtLeftArm.tail(3) - twistEstLeftArm.tail(3)); 
    //  right arm 
    footArmPoseRef.segment(18,3) = diag(configParams.kpArmRpy)*(twistTgtRightArm.head(3) - twistEstRightArm.head(3)) 
                                + diag(configParams.kdArmRpy)*(twistTgtRightArm.segment(6,3) - twistEstRightArm.segment(6,3));
    footArmPoseRef.segment(21,3) = diag(configParams.kpArmXyz)*(twistTgtRightArm.segment(3,3) - twistEstRightArm.segment(3,3)) 
                                + diag(configParams.kdArmXyz)*(twistTgtRightArm.tail(3) - twistEstRightArm.tail(3)); 
    //  Force and limit
    footArmForceRef = forceOpt;
    GlobalVelocityLimitationRef = -qDotActuated;


    // Update task & constraint             
    myWbc->updateTask("BipedPelvisRpy", pelvisRpyRef, configParams.weightPelvisOrientation);
    myWbc->updateTask("BipedPelvisXyz", pelvisXyzRef, configParams.weightPelvisPosition);
    myWbc->updateTask("BipedTorsoRpy", torsoRpyRef, configParams.weightTorsoOrientation);
    myWbc->updateTask("BipedTorsoXyz", torsoXyzRef, configParams.weightTorsoPosition);
    myWbc->updateTask("Position", footArmPoseRef, configParams.weightFootArmPosition);
    myWbc->updateTask("Force4", footArmForceRef, configParams.weightFootArmForce);
    myWbc->updateTask("GlobalVelocityLimitation", GlobalVelocityLimitationRef, configParams.weightGlobalVelLimitation);
    myWbc->updateTask("Dynamics", floatBaseDynamicRef, configParams.weightFloatBaseDynamic);

    // myWbc->updateTask("ForceChange4", footArmForceChangeRef, configParams.weightFootArmForceChange);
    // myWbc->updateTask("ForceChange4", footArmForceChangeRef, weightFootArmForceChange);
    // myWbc->updateTask("ForceChange", footForceChangeRef, weightFootForceChange);
    // myWbc->updateTask("Force", footForceRef, weightFootForce);
    myWbc->updateConstraint("BipedDynamicConsistency");
    myWbc->updateConstraint("BipedFrictionCone");
    myWbc->updateConstraint("BipedCenterOfPressure");
    myWbc->updateConstraint("BipedJointTorqueSaturation");

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