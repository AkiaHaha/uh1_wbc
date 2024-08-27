#include "bipedController.h"
#include "robotDynamics.h"
#include "robotDynamicsBiped.h"
#include "nlohmann/json.hpp"
#include <iostream>
#include <fstream>


using namespace std;
using json = nlohmann::json;
#define USING_HQP
#define USING_UNIT_WEI

BipedController::BipedController(){
    // -------------------- robot dynamics ------------------
    biped = new RobotDynamicsBiped();
    // -------------------- task control --------------------
    // Instantiate task & constraint (Create Task Library )
    HUMANOID::Task * ptrBipedTorsoPosRpy = new BipedTorsoRpy("BipedTorsoRpy", 3, nV);
    HUMANOID::Task * ptrBipedTorsoPosXyz = new BipedTorsoXyz("BipedTorsoXyz", 3, nV);
    HUMANOID::Task * ptrBipedComRpy = new BipedTorsoRpy("BipedComRpy", 3, nV);
    HUMANOID::Task * ptrBipedComXyz = new BipedTorsoXyz("BipedComXyz", 3, nV);
    HUMANOID::Task * ptrBipedUpTorsoPosRpy = new BipedUpTorsoRpy("BipedUpTorsoRpy", 3, nV);
    HUMANOID::Task * ptrBipedUpTorsoPosXyz = new BipedUpTorsoXyz("BipedUpTorsoXyz", 3, nV);
    HUMANOID::Task * ptrPosition = new QuadSolePosition("Position", NFCC4, nV);
    HUMANOID::Task * ptrForce4 = new QuadSoleForce("Force4", NFCC4, nV);
    HUMANOID::Task * ptrForceFoot = new BipedFootForce("FootForce", 12, nV);
    HUMANOID::Task * ptrForceArm = new BipedArmForce("ArmForce", 12, nV);
    HUMANOID::Task * ptrPoseFoot = new BipedFootPose("FootPose", 12, nV);
    HUMANOID::Task * ptrPoseArm = new BipedArmPose("ArmPose", 12, nV);
    HUMANOID::Task * ptrPoseArmStatic = new BipedArmPoseStatic("ArmPoseStatic", 8, nV);
    HUMANOID::Task * ptrDynamic = new BipedFloatingBaseDynamics("Dynamics", 6, nV);
    HUMANOID::Task * ptrGblVelLimits = new GlobalVelocityLimitation("GlobalVelocityLimitation", 19, nV); 
    
    //---------------------------------------------------------
    // HUMANOID::Task * ptrForce = new QuadSoleForce("Force", NFCC2, nV);
    // HUMANOID::Task * ptrForceChange = new QuadSoleForceChange("ForceChange", NFCC2, nV);
    HUMANOID::Constraint * ptrBipedDynamicConsistency = new BipedDynamicConsistency("BipedDynamicConsistency", 6, nV);
    HUMANOID::Constraint * ptrBipedFrictionCone = new BipedFrictionCone("BipedFrictionCone", 8, nV);
    HUMANOID::Constraint * ptrBipedJointTorqueSaturation = new BipedJointTorqueSaturation("BipedJointTorqueSaturation", NJ, nV);
    HUMANOID::Constraint * ptrBipedCenterOfPressure = new BipedCenterOfPressure("BipedCenterOfPressure", 8, nV);

    ptrBipedFrictionCone->setParameter(std::vector<double>{muStatic, myInfinity});
    ptrBipedJointTorqueSaturation->setParameter(std::vector<double>{jointTauLimit});
    ptrBipedCenterOfPressure->setParameter(std::vector<double>{soleFront, soleBack, soleLeft, soleRight, CopFactor, myInfinity});

    // Instantiate the Wbc instance && Add task & constraint to the instance
    // ---Hqp
#ifdef USING_HQP
    std::ifstream inputFile("/home/ukia/wwwws_uh1/src/unitree_wbc/config/controller.json");
    json jsonData;
    inputFile >> jsonData;
    int ForceFoot = jsonData["ForceFoot"];
    int ForceArm = jsonData["ForceArm"];
    int PoseFoot = jsonData["PoseFoot"];
    int PoseArm = jsonData["PoseArm"];
    int UpTorsoRPY = jsonData["UpTorsoRPY"];
    int UpTorsoXYZ = jsonData["UpTorsoXYZ"];
    int TorsoRPY = jsonData["TorsoRPY"];
    int TorsoXYZ = jsonData["TorsoXYZ"];
    int ComRPY = jsonData["ComRPY"];
    int ComXYZ = jsonData["ComXYZ"];
    int Dynamic = jsonData["Dynamic"];
    int GVL = jsonData["GVL"];

    myWbc = new HUMANOID::WqpWbc(nV, biped);
    // myWbc->addTask(ptrGblVelLimits, GVL);
    // myWbc->addTask(ptrForceFoot, ForceFoot);
    // myWbc->addTask(ptrForceArm, ForceArm);
    // myWbc->addTask(ptrBipedComRpy, ComRPY);
    // myWbc->addTask(ptrBipedComXyz, ComXYZ);
    // myWbc->addTask(ptrPoseArm, PoseArm);
    // myWbc->addTask(ptrBipedUpTorsoPosRpy, UpTorsoRPY);
    // myWbc->addTask(ptrBipedUpTorsoPosXyz,UpTorsoXYZ);
    myWbc->addTask(ptrPoseFoot, PoseFoot);
    myWbc->addTask(ptrPoseArmStatic, PoseArm);
    myWbc->addTask(ptrBipedTorsoPosRpy, TorsoRPY);
    myWbc->addTask(ptrBipedTorsoPosXyz, TorsoXYZ);
    myWbc->addTask(ptrDynamic, Dynamic);
    myWbc->addConstraint(ptrBipedFrictionCone, 0);
    myWbc->addConstraint(ptrBipedCenterOfPressure, 0);
    myWbc->addConstraint(ptrBipedJointTorqueSaturation, 0);
    // myWbc->addConstraint(ptrBipedDynamicConsistency, 0);
#else
    // ---Wqp
    myWbc = new HUMANOID::WqpWbc(nV, biped);
    myWbc->addTask(ptrBipedTorsoPosRpy, 0);
    myWbc->addTask(ptrBipedTorsoPosXyz, 0);
    myWbc->addTask(ptrBipedUpTorsoPosRpy, 0);
    myWbc->addTask(ptrBipedUpTorsoPosXyz, 0);
    // myWbc->addTask(ptrForce4, 0);
    myWbc->addTask(ptrPoseArmStatic, 0);
    // myWbc->addTask(ptrBipedComRpy, 0);
    // myWbc->addTask(ptrBipedComXyz, 0);
    myWbc->addTask(ptrPoseFoot, 0);
    myWbc->addTask(ptrDynamic, 0);
    myWbc->addTask(ptrGblVelLimits, 0);
    myWbc->addConstraint(ptrBipedDynamicConsistency, 0);
    myWbc->addConstraint(ptrBipedFrictionCone, 0);
    myWbc->addConstraint(ptrBipedCenterOfPressure, 0);
    myWbc->addConstraint(ptrBipedJointTorqueSaturation, 0);
#endif
    // Initialize the instance
    myWbc->wbcInit();
    myWbc->displayWbcInformation(); // optional
    myWbc->displayResultInformation();// show qp parameter informations //
}

BipedController::~BipedController(){
    delete biped;
    delete myWbc;
    biped = nullptr;
    myWbc = nullptr;
}

bool BipedController::update(double timeCtrlSys, const Eigen::VectorXd & imuData,
                const Eigen::VectorXd & jntPos, const Eigen::VectorXd & jntVel,
                const Eigen::VectorXd & forceSensorData, 
                const Eigen::VectorXd & LeftSoleXyzRpyAct, const Eigen::VectorXd & RightSoleXyzRpyAct, 
                const Eigen::VectorXd & LeftArmHandXyzRpyAct, const Eigen::VectorXd & RightArmHandXyzRpyAct){
    // update Time
    timeCs = timeCtrlSys;
    if ( tick > 0){
        time += DT;
    } else {//tick<=0
        tick = 0;
        time = 0.0;
    }
    stateEstimation(imuData, jntPos, jntVel, forceSensorData, LeftSoleXyzRpyAct, RightSoleXyzRpyAct, LeftArmHandXyzRpyAct, RightArmHandXyzRpyAct);
    motionPlan();
    taskControl();
    // update tick-tack
    tick++;
    if (tick >= std::numeric_limits<int>::max()-1000){
        tick = 1000;    // avoid tick out-of range
    }
    return true;
}

bool BipedController::getValueTauOpt(Eigen::VectorXd &jntTorOpt){
    for (int i = 0; i < nJa; i++){
        jntTorOpt(i) = tauOpt(i);
    }
    return true;
}

bool BipedController::getValueQdd(Eigen::VectorXd &Qdd){
    for (int i = 0; i < nJa; i++){
        Qdd(i) = qDDotOpt(i+6);
    }
    return true;
}

bool BipedController::getValuePosCurrent(Eigen::VectorXd &jntPosCur){
    for (int i = 0; i < nJa; i++){
        jntPosCur(i) = qActuated(i);
    }
    return true;
}

bool BipedController::stateEstimation(const Eigen::VectorXd & imuData,
                                      const Eigen::VectorXd & jntPos, const Eigen::VectorXd & jntVel,
                                      const Eigen::VectorXd & forceSensorData, 
                                      const Eigen::VectorXd & LeftSoleXyzRpyAct, const Eigen::VectorXd & RightSoleXyzRpyAct,
                                      const Eigen::VectorXd & LeftArmHandXyzRpyAct, const Eigen::VectorXd & RightArmHandXyzRpyAct){
    //<<<data from sensor//
    rpyTorsoEst = imuData.head(3);
    rpyDotTorsoEst = imuData.tail(3);
    qActuated = jntPos;
    qDotActuated = jntVel;
    groundReactiveForce = forceSensorData;

    qGen.tail(nJa) = qActuated;
    qDotGen.tail(nJa) = qDotActuated;
    qGen.segment(3,3) = rpyTorsoEst;
    qDotGen.segment(3,3) = rpyDotTorsoEst;

    //<<<torso xyz posVel//
    Eigen::VectorXd trosoStateTemp = Eigen::VectorXd::Zero(6,1);
    trosoStateTemp = biped->estWaistPosVelInWorld(qGen, qDotGen, 0);
    xyzTorsoEst = trosoStateTemp.head(3); 
    xyzDotTorsoEst = trosoStateTemp.tail(3);
    qGen.head(3) = xyzTorsoEst;
    qDotGen.head(3) = xyzDotTorsoEst;//>>>
    // cout << "******************** Torso ****************************" << endl;
    // cout << rpyTorsoEst.transpose() << endl;
    // cout << xyzTorsoEst.transpose() << endl;

    //<<<upTorso pose//
    Eigen::VectorXd poseTemp0 = Eigen::VectorXd::Zero(12,1);
    poseTemp0 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 0);

    rpyUpTorsoEst = poseTemp0.head(3);
    xyzUpTorsoEst = poseTemp0.segment(3,3);
    rpyDotUpTorsoEst = poseTemp0.segment(6,3);
    xyzDotUpTorsoEst = poseTemp0.tail(3);

    // cout << "******************** Up Torso ****************************" << endl;
    // cout << akiaPrint2(footStateTemp0, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    // cout << rpyUpTorsoEst.transpose() << endl;
    // cout << xyzUpTorsoEst.transpose() << endl;

    //<<<foot ee posVel//
    Eigen::VectorXd footStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd footStateTemp1 = Eigen::VectorXd::Zero(12,1);

    footStateTemp0 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 1);
    rpyFootEst[0] = footStateTemp0.head(3);
    xyzFootEst[0] = footStateTemp0.segment(3,3);
    rpyDotFootEst[0] = footStateTemp0.segment(6,3);
    xyzDotFootEst[0] = footStateTemp0.tail(3);
    // cout << "Left foot" << "-------------------------------" << endl;
    // cout << akiaPrint2(footStateTemp0, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    // cout << footStateTemp0.segment(3,3).transpose() << endl;

    footStateTemp1 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 2);
    rpyFootEst[1] = footStateTemp1.head(3);
    xyzFootEst[1] = footStateTemp1.segment(3,3);
    rpyDotFootEst[1] = footStateTemp1.segment(6,3);
    xyzDotFootEst[1] = footStateTemp1.tail(3);
    // cout << "Right foot" << "-------------------------------" << endl;
    // cout << akiaPrint2(footStateTemp1, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    // cout << footStateTemp1.segment(3,3).transpose() << endl;

    // <<<arm ee posVel//
    Eigen::VectorXd armStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd armStateTemp1 = Eigen::VectorXd::Zero(12,1);

    armStateTemp0 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 3);
    xyzArmEst[0] = armStateTemp0.head(3);
    xyzArmEst[0] = armStateTemp0.segment(3,3);
    rpyDotArmEst[0] = armStateTemp0.segment(6,3);
    xyzDotArmEst[0] = armStateTemp0.tail(3);
    // cout << "Left arm" << "-------------------------------" << endl;
    // cout << akiaPrint2(armStateTemp0, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    // cout << armStateTemp0.segment(3,3).transpose() << endl;

    armStateTemp1 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 4);
    rpyArmEst[1] = armStateTemp1.head(3);
    xyzArmEst[1] = armStateTemp1.segment(3,3);
    rpyDotArmEst[1] = armStateTemp1.segment(6,3);
    xyzDotArmEst[1] = armStateTemp1.tail(3);
    // cout << "Right arm" << "-------------------------------" << endl;
    // cout << akiaPrint2(armStateTemp1, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    // cout << armStateTemp1.segment(3,3).transpose() << endl;

    //<<<arm ee xyzRpy from supervisor//
    // cout << "arm ee xyzRpy from supervisor" << "-----------------------------------" << endl;
        // << "Left: " << LeftArmHandXyzRpyAct.head(3).transpose() << endl;
        // << "Right: " << RightArmHandXyzRpyAct.head(3).transpose() << endl<<endl;   


    // com pose estimation
    xyzComEst = biped->comPos2World;
    xyzDotComEst = biped->comVel2World;

    if(flagEstFirst == 0){
        flagEstFirst = 1;
        
        armPoseStaticInit = jntPos.tail(8);

        xyzTorsoInit = xyzTorsoEst;
        rpyTorsoInit = rpyTorsoEst;

        xyzComInit = xyzComEst;
        xyzDotComInit = xyzDotComEst;
        rpyComInit = rpyTorsoEst;

        xyzUpTorsoInit = xyzUpTorsoEst;
        rpyUpTorsoInit = rpyUpTorsoEst;

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

    armPoseStaticRef = 20000000 * (armPoseStaticInit - jntPos.tail(8));
    



    return true;
}

bool BipedController::motionPlan(){//Daniel 5.23
        std::ifstream inputFile("/home/ukia/wwwws_uh1/src/unitree_wbc/config/controller.json");
        json jsonData;
        inputFile >> jsonData;
        double height = jsonData["height"];
        double pitchApt = jsonData["pitchApt"];
        double pitchFrq = jsonData["pitchFrq"];

        // if (flagTimeSetZero == 0){
        //     time = 0.0;
        //     flagTimeSetZero = 1;
        //     cout << "welcome to plan stage 2 " << endl<<endl;
        // }
        // // TORSO XYZ
        // xyzTorsoTgt << xyzTorsoInit(0), xyzTorsoInit(1), -0.005*sin(time/1*PI)+xyzTorsoInit(2);
        // xyzDotTorsoTgt << 0.0, 0.0, -0.005*PI*cos(time/1*PI);
        // std::cout << "plan stage 2"  << std::endl;

        //torso
        if(time <= 1000){
            dsp = rpyTorsoInit(1)-pitchApt*(sin((pitchFrq*time+0.5)*PI)-1);
        }
        xyzTorsoTgt = xyzTorsoInit;
        xyzDotTorsoTgt << 0.0, 0.0, 0.0;
        // xyzTorsoTgt << xyzTorsoInit(0), xyzTorsoInit(1), xyzTorsoInit(2)+height*(sin((time+0.5)*PI)-1);
        // xyzTorsoTgt << comx, comy, xyzTorsoInit(2)+height*(sin((time+0.5)*PI)-1);
        // xyzTorsoTgt << CoM(0), CoM(1), xyzTorsoInit(2)+height*(sin((time+0.5)*PI)-1);
        // xyzDotTorsoTgt <<  0.0, 0.0, height*PI*cos((time+0.5)*PI);

        rpyTorsoTgt << 0.0, dsp, 0.0;
        rpyDotTorsoTgt << 0.0, 0.0, 0.0;

        // com
        xyzComTgt = xyzComInit;
        xyzDotComTgt = xyzDotComInit;
        rpyComTgt = rpyTorsoInit;

        // Up torso
        xyzUpTorsoTgt = xyzUpTorsoInit;
        xyzDotUpTorsoTgt << 0.0, 0.0, 0.0;
        // xyzUpTorsoTgt << xyzUpTorsoInit(0), xyzUpTorsoInit(1), xyzUpTorsoInit(2)+height*(sin((time+0.5)*PI)-1);
        // xyzUpTorsoTgt << comx, comy, xyzUpTorsoInit(2)+height*(sin((time+0.5)*PI)-1);
        // xyzUpTorsoTgt << CoM(0), CoM(1), xyzUpTorsoInit(2)+height*(sin((time+0.5)*PI)-1);
        // xyzDotUpTorsoTgt << 0.0, 0.0, height*PI*cos((time+0.5)*PI);
        rpyUpTorsoTgt << 0.0, dsp, 0.0;
        rpyDotUpTorsoTgt << 0.0, 0.0, 0.0;
        
        //foot
        xyzFootTgt[0] = xyzFootInit[0];
        xyzDotFootTgt[0] = Eigen::Vector3d::Zero();
        rpyFootTgt[0] = rpyFootInit[0];
        rpyDotFootTgt[0] = Eigen::Vector3d::Zero();

        xyzFootTgt[1] = xyzFootInit[1];
        xyzDotFootTgt[1] = Eigen::Vector3d::Zero();
        rpyFootTgt[1] = rpyFootInit[1];
        rpyDotFootTgt[1] = Eigen::Vector3d::Zero();

        // arm
        xyzArmTgt[0] = xyzArmInit[0];
        xyzDotArmTgt[0] = Eigen::Vector3d::Zero();
        // xyzArmTgt[0] << xyzArmInit[0].x(), xyzArmInit[0].y(), xyzArmInit[0].z()+height*(sin((time+0.5)*PI)-1);
        // xyzDotArmTgt[0] << xyzDotArmInit[0].x(), xyzDotArmInit[0].y(), xyzDotArmInit[0].z()+height*PI*cos((time+0.5)*PI);
        rpyArmTgt[0] = rpyArmInit[0];
        rpyDotArmTgt[0] = Eigen::Vector3d::Zero();


        xyzArmTgt[1] = xyzArmInit[1];
        xyzDotArmTgt[1] = Eigen::Vector3d::Zero();
        // xyzArmTgt[1] << xyzArmInit[1].x(), xyzArmInit[1].y(), xyzArmInit[1].z()+height*(sin((time+0.5)*PI)-1);
        // xyzDotArmTgt[1] << xyzDotArmInit[1].x(), xyzDotArmInit[1].y(), xyzDotArmInit[1].z()+height*PI*cos((time+0.5)*PI);
        rpyArmTgt[1] = rpyArmInit[1];
        rpyDotArmTgt[1] = Eigen::Vector3d::Zero();
    return true;
}

bool BipedController::taskControl(){

    // ------------------------------ Update Robot Dynamics --------------------------- //
    myWbc->updateRobotDynamics(qGen, qDotGen);
    std::ifstream inputFile("/home/ukia/wwwws_uh1/src/unitree_wbc/config/controller.json");
    json jsonData;
    inputFile >> jsonData;

    double kpTorsoR = jsonData["kpTorsoR"];
    double kpTorsoP = jsonData["kpTorsoP"];
    double kpTorsoY = jsonData["kpTorsoY"];
    double kdTorsoR = jsonData["kdTorsoR"];
    double kdTorsoP = jsonData["kdTorsoP"];
    double kdTorsoY = jsonData["kdTorsoY"];

    double kpTorsoX = jsonData["kpTorsoX"];
    double kpTorsoYY = jsonData["kpTorsoYY"];
    double kpTorsoZ = jsonData["kpTorsoZ"];
    double kdTorsoX = jsonData["kdTorsoX"];
    double kdTorsoYY = jsonData["kdTorsoYY"];
    double kdTorsoZ = jsonData["kdTorsoZ"];

    double kpUpTorsoR = jsonData["kpUpTorsoR"];
    double kpUpTorsoP = jsonData["kpUpTorsoP"];
    double kpUpTorsoY = jsonData["kpUpTorsoY"];
    double kdUpTorsoR = jsonData["kdUpTorsoR"];
    double kdUpTorsoP = jsonData["kdUpTorsoP"];
    double kdUpTorsoY = jsonData["kdUpTorsoY"];

    double kpUpTorsoX = jsonData["kpUpTorsoX"];
    double kpUpTorsoYY = jsonData["kpUpTorsoYY"];
    double kpUpTorsoZ = jsonData["kpUpTorsoZ"];
    double kdUpTorsoX = jsonData["kdUpTorsoX"];
    double kdUpTorsoYY = jsonData["kdUpTorsoYY"];
    double kdUpTorsoZ = jsonData["kdUpTorsoZ"];

    double kpFootR = jsonData["kpFootR"];
    double kpFootP = jsonData["kpFootP"];
    double kpFootY = jsonData["kpFootY"];
    double kdFootR = jsonData["kdFootR"];
    double kdFootP = jsonData["kdFootP"];
    double kdFootY = jsonData["kdFootY"];

    double kpFootX = jsonData["kpFootX"];
    double kpFootYY = jsonData["kpFootYY"];
    double kpFootZ = jsonData["kpFootZ"];
    double kdFootX = jsonData["kdFootX"];
    double kdFootYY = jsonData["kdFootY"];
    double kdFootZ = jsonData["kdFootZ"];

    double kpArmR = jsonData["kpArmR"];
    double kpArmP = jsonData["kpArmP"];
    double kpArmY = jsonData["kpArmY"];
    double kdArmR = jsonData["kdArmR"];
    double kdArmP = jsonData["kdArmP"];
    double kdArmY = jsonData["kdArmY"];

    double kpArmX = jsonData["kpArmX"];
    double kpArmYY = jsonData["kpArmYY"];
    double kpArmZ = jsonData["kpArmZ"];
    double kdArmX = jsonData["kdArmX"];
    double kdArmYY = jsonData["kdArmY"];
    double kdArmZ = jsonData["kdArmZ"];


    // ------------------------------ set weights --------------------------------------
#ifdef USING_HQP
    double hweightFootForceR = jsonData["hweightFootForceR"];
    double hweightFootForceP = jsonData["hweightFootForceP"];
    double hweightFootForceYaw = jsonData["hweightFootForceYaw"];
    double hweightFootForceX = jsonData["hweightFootForceX"];
    double hweightFootForceY = jsonData["hweightFootForceY"];
    double hweightFootForceZ = jsonData["hweightFootForceZ"];

    double hweightArmForceR = jsonData["hweightArmForceR"];
    double hweightArmForceP = jsonData["hweightArmForceP"];
    double hweightArmForceYaw = jsonData["hweightArmForceYaw"];
    double hweightArmForceX = jsonData["hweightArmForceX"];
    double hweightArmForceY = jsonData["hweightArmForceY"];
    double hweightArmForceZ = jsonData["hweightArmForceZ"];

    double hweightFootPoseR = jsonData["hweightFootPoseR"];
    double hweightFootPoseP = jsonData["hweightFootPoseP"];
    double hweightFootPoseYaw = jsonData["hweightFootPoseYaw"];
    double hweightFootPoseX = jsonData["hweightFootPoseX"];
    double hweightFootPoseY = jsonData["hweightFootPoseY"];
    double hweightFootPoseZ = jsonData["hweightFootPoseZ"];

    double hweightArmPoseR = jsonData["hweightArmPoseR"];
    double hweightArmPoseP = jsonData["hweightArmPoseP"];
    double hweightArmPoseYaw = jsonData["hweightArmPoseYaw"];
    double hweightArmPoseX = jsonData["hweightArmPoseX"];
    double hweightArmPoseY = jsonData["hweightArmPoseY"];
    double hweightArmPoseZ = jsonData["hweightArmPoseZ"];

    double hweightTorsoPos = jsonData["hweightTorsoPos"];
    double hweightTorsoRPY = jsonData["hweightTorsoRPY"];

    double weightFBD = jsonData["weightFBD"];

    weightFootArmForce <<   hweightFootForceR, hweightFootForceP, hweightFootForceYaw, 
                            hweightFootForceX, hweightFootForceY, hweightFootForceZ, 
                            hweightFootForceR, hweightFootForceP, hweightFootForceYaw, 
                            hweightFootForceX, hweightFootForceY, hweightFootForceZ,
                            hweightArmForceR, hweightArmForceP, hweightArmForceYaw, 
                            hweightArmForceX, hweightArmForceY, hweightArmForceZ, 
                            hweightArmForceR, hweightArmForceP, hweightArmForceYaw, 
                            hweightArmForceX, hweightArmForceY, hweightArmForceZ;

    weightUpTorsoPosition = fillVector3(hweightTorsoPos);                                        
    weightUpTorsoOrientation = fillVector3(hweightTorsoRPY);
    weightTorsoPosition = fillVector3(hweightTorsoPos);                                           
    weightTorsoOrientation = fillVector3(hweightTorsoRPY); 
    weightFloatBaseDynamic = Eigen::VectorXd::Constant(6,weightFBD);
    weightGlobalVelLimitation = Eigen::VectorXd::Constant(19,1);

    weightArmForce <<   hweightArmForceR, hweightArmForceP, hweightArmForceYaw, 
                        hweightArmForceX, hweightArmForceY, hweightArmForceZ, 
                        hweightArmForceR, hweightArmForceP, hweightArmForceYaw, 
                        hweightArmForceX, hweightArmForceY, hweightArmForceZ;

    weightFootForce<<   hweightFootForceR, hweightFootForceP, hweightFootForceYaw, 
                        hweightFootForceX, hweightFootForceY, hweightFootForceZ, 
                        hweightFootForceR, hweightFootForceP, hweightFootForceYaw, 
                        hweightFootForceX, hweightFootForceY, hweightFootForceZ;

    weightArmPose <<    hweightArmPoseR, hweightArmPoseP, hweightArmPoseYaw, 
                        hweightArmPoseX, hweightArmPoseY, hweightArmPoseZ, 
                        hweightArmPoseR, hweightArmPoseP, hweightArmPoseYaw, 
                        hweightArmPoseX, hweightArmPoseY, hweightArmPoseZ;

    weightFootPose <<   hweightFootPoseR, hweightFootPoseP, hweightFootPoseYaw, 
                        hweightFootPoseX, hweightFootPoseY, hweightFootPoseZ, 
                        hweightFootPoseR, hweightFootPoseP, hweightFootPoseYaw, 
                        hweightFootPoseX, hweightFootPoseY, hweightArmPoseZ;


#else
    double weightGVL = jsonData["weightGVL"];
    double weightGVL10 = jsonData["weightGVL10"];
    double weightGVLKnee = jsonData["weightGVLKnee"];
    double weightGVLAnkle = jsonData["weightGVLAnkle"];

    double weightFBD = jsonData["weightFBD"];

    double weightFootForceR = jsonData["weightFootForceR"];
    double weightFootForceP = jsonData["weightFootForceP"];
    double weightFootForceYaw = jsonData["weightFootForceYaw"];
    double weightFootForceX = jsonData["weightFootForceX"];
    double weightFootForceY = jsonData["weightFootForceY"];
    double weightFootForceZ = jsonData["weightFootForceZ"];

    double weightArmForceR = jsonData["weightArmForceR"];
    double weightArmForceP = jsonData["weightArmForceP"];
    double weightArmForceYaw = jsonData["weightArmForceYaw"];
    double weightArmForceX = jsonData["weightArmForceX"];
    double weightArmForceY = jsonData["weightArmForceY"];
    double weightArmForceZ = jsonData["weightArmForceZ"];


    double weightFootPos = jsonData["weightFootPos"];
    double weightFootYaw = jsonData["weightFootYaw"];
    double weightArmPos = jsonData["weightArmPos"];
    double weightArmRPY = jsonData["weightArmRPY"];
    double weightArmZ = jsonData["weightArmZ"];

    double weightTorsoPos = jsonData["weightTorsoPos"];
    double weightTorsoRPY = jsonData["weightTorsoRPY"];

    weightTorsoPosition = fillVector3(600000);                                           
    weightTorsoOrientation = fillVector3(600000); 
    weightUpTorsoPosition = fillVector3(300000);                                        
    weightUpTorsoOrientation << 300000, 300000, 3000000;

    weightFootArmPosition = fillVector(weightFootPos, weightArmPos);   
    weightFootArmPosition(2) = weightFootYaw;
    weightFootArmPosition(8) = weightFootYaw;
    weightFootArmPosition(17) = weightArmZ;
    weightFootArmPosition(23) = weightArmZ;
    weightFootArmPosition.segment(12,3) = Eigen::VectorXd::Constant(3,weightArmRPY);
    weightFootArmPosition.segment(18,3) = Eigen::VectorXd::Constant(3,weightArmRPY);
    
    weightFootArmForce <<   weightFootForceR, weightFootForceP, weightFootForceYaw, 
                            weightFootForceX, weightFootForceY, weightFootForceZ, 
                            weightFootForceR, weightFootForceP, weightFootForceYaw, 
                            weightFootForceX, weightFootForceY, weightFootForceZ,
                            weightArmForceR, weightArmForceP, weightArmForceYaw, 
                            weightArmForceX, weightArmForceY, weightArmForceZ, 
                            weightArmForceR, weightArmForceP, weightArmForceYaw, 
                            weightArmForceX, weightArmForceY, weightArmForceZ;
                            // 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 
                            // 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;



    weightFloatBaseDynamic = Eigen::VectorXd::Constant(6,weightFBD);

    weightGlobalVelLimitation = Eigen::VectorXd::Constant(19,weightGVL);
    weightGlobalVelLimitation(10) = weightGVL10;
    weightGlobalVelLimitation(3) = weightGVLKnee;
    weightGlobalVelLimitation(8) = weightGVLKnee;
    weightGlobalVelLimitation(4) = weightGVLAnkle;
    weightGlobalVelLimitation(9) = weightGVLAnkle;
#endif

    // ------------------------------ Set PD gains ------------------------------------ //
    kpTorsoRpy = {kpTorsoR, kpTorsoP, kpTorsoY};
    kdTorsoRpy = {kdTorsoR, kdTorsoP, kdTorsoY};
    kpTorsoXyz = {kpTorsoX, kpTorsoYY, kpTorsoZ};
    kdTorsoXyz = {kdTorsoX, kdTorsoYY, kdTorsoZ};

    kpUpTorsoRpy = {kpUpTorsoR, kpUpTorsoP, kpUpTorsoY};
    kdUpTorsoRpy = {kdUpTorsoR, kdUpTorsoP, kdUpTorsoY};
    kpUpTorsoXyz = {kpUpTorsoX, kpUpTorsoYY, kpUpTorsoZ};
    kdUpTorsoXyz = {kdUpTorsoX, kdUpTorsoYY, kdUpTorsoZ};


    kpFootXyz = {kpFootX, kpFootYY, kpFootZ};
    kdFootXyz = {kdFootX, kdFootYY, kdFootZ};
    kpFootRpy = {kpFootR, kpFootP, kpFootY};
    kdFootRpy = {kdFootR, kdFootP, kdFootY};

    kpArmXyz = {kpArmX, kpArmYY, kpArmZ};
    kdArmXyz = {kdArmX, kdArmYY, kdArmZ};
    kpArmRpy = {kpArmR, kpArmP, kpArmY};
    kdArmRpy = {kdArmR, kdArmP, kdArmY};
    // ------------------------------ Calculate Reference ------------------------------ //
    // GVL && ForceRef
    GlobalVelocityLimitationRef = -qDotActuated;
    // GlobalVelocityLimitationRef = Eigen::VectorXd::Zero(19);
    footArmForceRef = forceOpt;
    footForceRef = forceOpt.head(12);
    armForceRef = forceOpt.tail(12);

    // torso
    torsoRpyRef = diag(kpTorsoRpy)*(rpyTorsoTgt - rpyTorsoEst) + diag(kdTorsoRpy)*(rpyDotTorsoTgt - rpyDotTorsoEst);
    torsoXyzRef = diag(kpTorsoXyz)*(xyzTorsoTgt - xyzTorsoEst) + diag(kdTorsoXyz)*(xyzDotTorsoTgt - xyzDotTorsoEst);
    // up torso
    upTorsoRpyRef = diag(kpUpTorsoRpy)*(rpyUpTorsoTgt - rpyUpTorsoEst) + diag(kdUpTorsoRpy)*(rpyDotUpTorsoTgt - rpyDotUpTorsoEst);
    upTorsoXyzRef = diag(kpUpTorsoXyz)*(xyzUpTorsoTgt - xyzUpTorsoEst) + diag(kdUpTorsoXyz)*(xyzDotUpTorsoTgt - xyzDotUpTorsoEst);
    // com
    comRpyRef = diag(kpTorsoRpy)*(rpyComTgt - rpyComEst) + diag(kdTorsoRpy)*(rpyDotComTgt - rpyDotComEst); 
    comXyzRef = diag(kpTorsoXyz)*(xyzComTgt - xyzComEst) + diag(kdTorsoXyz)*(xyzDotComTgt - xyzDotComEst); 

    // // left foot
    // footArmPosRef.segment(0,3) = diag(kpFootRpy)*(rpyFootTgt[0] - rpyFootEst[0]) + diag(kdFootRpy)*(rpyDotFootTgt[0] - rpyDotFootEst[0]);
    // footArmPosRef.segment(3,3) = diag(kpFootXyz)*(xyzFootTgt[0] - xyzFootEst[0]) + diag(kdFootXyz)*(xyzDotFootTgt[0] - xyzDotFootEst[0]); 
    // //right foot 
    // footArmPosRef.segment(6,3) = diag(kpFootRpy)*(rpyFootTgt[1] - rpyFootEst[1]) + diag(kdFootRpy)*(rpyDotFootTgt[1] - rpyDotFootEst[1]);
    // footArmPosRef.segment(9,3) = diag(kpFootXyz)*(xyzFootTgt[1] - xyzFootEst[1]) + diag(kdFootXyz)*(xyzDotFootTgt[1] - xyzDotFootEst[1]); 
    // // left arm
    // footArmPosRef.segment(12,3) = diag(kpArmRpy)*(rpyArmTgt[0] - rpyArmEst[0]) + diag(kdArmRpy)*(rpyDotArmTgt[0] - rpyDotArmEst[0]);
    // footArmPosRef.segment(15,3) = diag(kpArmXyz)*(xyzArmTgt[0] - xyzArmEst[0]) + diag(kdArmXyz)*(xyzDotArmTgt[0] - xyzDotArmEst[0]); 
    // //right arm 
    // footArmPosRef.segment(18,3) = diag(kpArmRpy)*(rpyArmTgt[1] - rpyArmEst[1]) + diag(kdArmRpy)*(rpyDotArmTgt[1] - rpyDotArmEst[1]);
    // footArmPosRef.segment(21,3) = diag(kpArmXyz)*(xyzArmTgt[1] - xyzArmEst[1]) + diag(kdArmXyz)*(xyzDotArmTgt[1] - xyzDotArmEst[1]);  
    
    // left foot
    footPoseRef.segment(0,3) = diag(kpFootRpy)*(rpyFootTgt[0] - rpyFootEst[0]) + diag(kdFootRpy)*(rpyDotFootTgt[0] - rpyDotFootEst[0]);
    footPoseRef.segment(3,3) = diag(kpFootXyz)*(xyzFootTgt[0] - xyzFootEst[0]) + diag(kdFootXyz)*(xyzDotFootTgt[0] - xyzDotFootEst[0]); 
    //right foot 
    footPoseRef.segment(6,3) = diag(kpFootRpy)*(rpyFootTgt[1] - rpyFootEst[1]) + diag(kdFootRpy)*(rpyDotFootTgt[1] - rpyDotFootEst[1]);
    footPoseRef.segment(9,3) = diag(kpFootXyz)*(xyzFootTgt[1] - xyzFootEst[1]) + diag(kdFootXyz)*(xyzDotFootTgt[1] - xyzDotFootEst[1]); 
    // left arm
    armPoseRef.segment(0,3) = diag(kpArmRpy)*(rpyArmTgt[0] - rpyArmEst[0]) + diag(kdArmRpy)*(rpyDotArmTgt[0] - rpyDotArmEst[0]);
    armPoseRef.segment(3,3) = diag(kpArmXyz)*(xyzArmTgt[0] - xyzArmEst[0]) + diag(kdArmXyz)*(xyzDotArmTgt[0] - xyzDotArmEst[0]); 
    //right arm 
    armPoseRef.segment(6,3) = diag(kpArmRpy)*(rpyArmTgt[1] - rpyArmEst[1]) + diag(kdArmRpy)*(rpyDotArmTgt[1] - rpyDotArmEst[1]);
    armPoseRef.segment(9,3) = diag(kpArmXyz)*(xyzArmTgt[1] - xyzArmEst[1]) + diag(kdArmXyz)*(xyzDotArmTgt[1] - xyzDotArmEst[1]);  


    // ------------------------------ Update task & constraint ------------------------- //                 
    myWbc->updateTask("BipedTorsoRpy", torsoRpyRef, weightTorsoOrientation);
    myWbc->updateTask("BipedTorsoXyz", torsoXyzRef, weightTorsoPosition);
    // myWbc->updateTask("BipedComRpy", comRpyRef, weightComOrientation);
    // myWbc->updateTask("BipedComXyz", comXyzRef, weightComPosition);
    myWbc->updateTask("BipedUpTorsoRpy", upTorsoRpyRef, weightUpTorsoOrientation);
    myWbc->updateTask("BipedUpTorsoXyz", upTorsoXyzRef, weightUpTorsoPosition);
    // myWbc->updateTask("Position", footArmPosRef, weightFootArmPosition);
    myWbc->updateTask("FootPose", footPoseRef, weightFootPose);
    // myWbc->updateTask("ArmPose", armPoseRef, weightArmPose);
    myWbc->updateTask("ArmPoseStatic", armPoseStaticRef, weightArmPose);
    // myWbc->updateTask("Force4", footArmForceRef, weightFootArmForce);
    // myWbc->updateTask("FootForce", footForceRef, weightFootForce);
    // myWbc->updateTask("ArmForce", armForceRef, weightArmForce);
    myWbc->updateTask("GlobalVelocityLimitation", GlobalVelocityLimitationRef, weightGlobalVelLimitation);
    myWbc->updateTask("Dynamics", floatBaseDynamicRef, weightFloatBaseDynamic);

    // myWbc->updateTask("ForceChange4", footArmForceChangeRef, weightFootArmForceChange);
    // myWbc->updateTask("ForceChange", footForceChangeRef, weightFootForceChange);
    // myWbc->updateTask("Force", footForceRef, weightFootForce);
    myWbc->updateConstraint("BipedDynamicConsistency");
    myWbc->updateConstraint("BipedFrictionCone");
    myWbc->updateConstraint("BipedCenterOfPressure");
    myWbc->updateConstraint("BipedJointTorqueSaturation");

    // ------------------------------ Update bounds ------------------------------------- //
    lowerbounds(NG+5) = 0.0;
    upperbounds(NG+5) = 1000.0*GRAVITY;
    lowerbounds(NG+11) = 0.0;
    upperbounds(NG+11) = 1000.0*GRAVITY;

    // cout << lowerbounds.transpose() << endl;
    // cout << upperbounds.transpose() << endl;
    myWbc->updateBound(lowerbounds, upperbounds);

    // ------------------------------ WBC solve ------------------------------------------ //
    // exe wbc slove
    myWbc->wbcSolve();

    // get some data from solved wbc //
    myWbc->getAuxiliaryDataInt(intData);
    nWsrRes = intData.at(0);
    double Nlevel = myWbc->getNlevel();
    for (int i = Nlevel; i < Nlevel*2; i++){
        simpleStatus += intData.at(i);
    }
    myWbc->getAuxiliaryDataDouble(doubleData);
    costOpt = doubleData.at(0);
    cpuTimeRes = doubleData.at(1);

    // get wbc variables output and clac toq //
    // here shows the nV = G + Fc //
    if (simpleStatus == 0){
        myWbc->getResultOpt(varOpt);
        qDDotOpt = varOpt.head(nJg);
        forceOpt = varOpt.tail(nFc);
        tauOpt = biped->eqCstrMatTau * varOpt + biped->eqCstrMatTauBias;
    }else{
        std::cerr << "QP failed; Exiting the program at time of: " << timeCs << "  simpleStatus: " << simpleStatus << std::endl;
        exit(EXIT_FAILURE); 
    }
        // cout << endl << "output*** " << timeCs << endl;
        // cout << endl << "qDDotOpt-----------------" << endl;
        // akiaPrint1(qDDotOpt, NJ, 5, 5, 5, 1, 4, 4);
        // cout << endl << "forceOpt-----------------" << endl;
        // akiaPrint1(forceOpt, NFCC2, 2, 6, 6);
        cout << endl << "tauOpt at timeCs of " << timeCs << endl;
        akiaPrint1(tauOpt, NJ, 5, 5, 5, 1, 4, 4);
        // cout << endl << "varOpt-----------------" << endl;
        // akiaPrint1(varOpt, NV, 5, 6, 11, 8, 6, 6);
    return true;
}

Eigen::MatrixXd BipedController::diag(const std::vector<double>& diagElement){
    int dim = static_cast<int>(diagElement.size());
    Eigen::MatrixXd diagM = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i != dim; i++){
        diagM(i, i) = diagElement.at(i);
    }
    return diagM;
}
