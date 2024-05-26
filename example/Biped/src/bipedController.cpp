/**
 *	This file is part of TAICHI.
 *
 *	TAICHI -- Task Arrangement In Control HIerarchy.
 *	Copyright (C) 2015-2021 Beijing Research Institute of UBTECH Robotics.
 *	All rights reserved.
 *
 *	Licensed under the Apache License 2.0. See LICENSE for more details.
 */

/**
 * @file bipedController.cpp
 * @brief Function implementation part of the class BipedController.
 * @author Jiajun Wang, Yijie Guo
 * @date 2021
 * @version alpha
 */

#include "bipedController.h"
using namespace std;

BipedController::BipedController(){
    // -------------------- robot dynamics ------------------
    std::cout<<"12000"<<std::endl;
    biped = new RobotDynamicsBiped();
    std::cout<<"12000"<<std::endl;
    // -------------------- task control --------------------
    // Instantiate task & constraint (Create Task Library )
    TAICHI::Task * ptrBipedTorsoPosRpy = new BipedTorsoPosRpy("BipedTorsoPosRpy", 3, nV);
    TAICHI::Task * ptrBipedTorsoPosXyz = new BipedTorsoPosXyz("BipedTorsoPosXyz", 3, nV);
    // TAICHI::Task * ptrBipedFootForce = new BipedFootForce("BipedFootForce", 12, nV);//Daniel 5.22
    TAICHI::Task * ptrBipedFootForce = new BipedFootForce("BipedFootForce", nFc, nV);
    // TAICHI::Task * ptrBipedFootForceChange = new BipedFootForceChange("BipedFootForceChange", 12, nV);
    TAICHI::Task * ptrBipedFootForceChange = new BipedFootForceChange("BipedFootForceChange", nFc, nV);
    TAICHI::Task * ptrBipedFootPosition = new BipedFootPosition("BipedFootPosition", 12, nV);

    TAICHI::Constraint * ptrBipedDynamicConsistency = new BipedDynamicConsistency("BipedDynamicConsistency", 6, nV);
    TAICHI::Constraint * ptrBipedFrictionCone = new BipedFrictionCone("BipedFrictionCone", 8, nV);
    TAICHI::Constraint * ptrBipedJointTorqueSaturation = new BipedJointTorqueSaturation("BipedJointTorqueSaturation", 19, nV);
    TAICHI::Constraint * ptrBipedCenterOfPressure = new BipedCenterOfPressure("BipedCenterOfPressure", 8, nV);

    ptrBipedFrictionCone->setParameter(std::vector<double>{muStatic, myInfinity});
    ptrBipedJointTorqueSaturation->setParameter(std::vector<double>{jointTauLimit});
    ptrBipedCenterOfPressure->setParameter(std::vector<double>{soleFront, soleBack, soleLeft, soleRight, CopFactor, myInfinity});

    std::cout<<"12000"<<std::endl;
    // Instantiate the Wbc instance
    myWbc = new TAICHI::WqpWbc(nV, biped);
    std::cout<<"12300"<<std::endl;

    // Add task & constraint to the instance
    myWbc->addTask(ptrBipedTorsoPosRpy, 0);
    myWbc->addTask(ptrBipedTorsoPosXyz, 0);
    myWbc->addTask(ptrBipedFootForce, 0);
    myWbc->addTask(ptrBipedFootForceChange, 0);
    myWbc->addTask(ptrBipedFootPosition, 0);
    myWbc->addConstraint(ptrBipedDynamicConsistency, 0);
    myWbc->addConstraint(ptrBipedFrictionCone, 0);
    myWbc->addConstraint(ptrBipedCenterOfPressure, 0);
    myWbc->addConstraint(ptrBipedJointTorqueSaturation, 0);

    std::cout<<"12340"<<std::endl;
    // Initialize the instance
    myWbc->wbcInit();
    std::cout<<"12345"<<std::endl;
    
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
                const Eigen::VectorXd & forceSensorData, const Eigen::VectorXd & LeftSoleXyzRpyAct,
                const Eigen::VectorXd & RightSoleXyzRpyAct){
    // update Time
    timeCs = timeCtrlSys;
    if ( tick > 0){
        time += DT;
    } else {//tick<=0
        tick = 0;
        time = 0.0;
    }
    // std::cout << "imuData" <<imuData<< std::endl;
    // std::cout << "jntPos" <<jntPos<< std::endl;
    // estimate --> plan --> control
    stateEstimation(imuData, jntPos, jntVel, forceSensorData, LeftSoleXyzRpyAct, RightSoleXyzRpyAct);
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

bool BipedController::getValuePosCurrent(Eigen::VectorXd &jntPosCur){
    for (int i = 0; i < nJa; i++){
        jntPosCur(i) = qActuated(i);
    }
    return true;
}

bool BipedController::stateEstimation(const Eigen::VectorXd & imuData,
                                      const Eigen::VectorXd & jntPos, const Eigen::VectorXd & jntVel,
                                      const Eigen::VectorXd & forceSensorData, const Eigen::VectorXd & LeftSoleXyzRpyAct,
                                      const Eigen::VectorXd & RightSoleXyzRpyAct){
    // ------------------------------------- read sensors -------------------------------------
    rpyTorsoEst = imuData.head(3);//robotStateSim.imu9DAct << robotStateSim.waistRpyAct, robotStateSim.waistXyzAccAct, robotStateSim.waistRpyVelAct; {not used of imu data, only supervisor of waist and accelerometer}
    rpyDotTorsoEst = imuData.tail(3);
    
    qActuated = jntPos;
    qDotActuated = jntVel;
    groundReactiveForce = forceSensorData;

    qGen.segment(3,3) = rpyTorsoEst;
    qGen.tail(nJa) = qActuated;//Daniel
    qDotGen.segment(3,3) = rpyDotTorsoEst;
    qDotGen.tail(nJa) = qDotActuated;

    // ------------------------------------- read sensors -------------------------------------
    // ------------------------------------- update Kinematics --------------------------------
    Eigen::VectorXd trosoStateTemp = Eigen::VectorXd::Zero(6,1);
    trosoStateTemp = biped->estWaistPosVelInWorld(qGen, qDotGen, stanceLeg);
    xyzTorsoEst = trosoStateTemp.head(3); 
    xyzDotTorsoEst = trosoStateTemp.tail(3);
    qGen.head(3) = xyzTorsoEst;
    qDotGen.head(3) = xyzDotTorsoEst;

    //<--------------use supervisor to get waist xyz---Daniel 5.23----------------------------//
    xyzTorsoEst = imuData.segment(3,3);
    xyzDotTorsoEst = imuData.segment(6,3);
    qGen.head(3) = imuData.segment(3,3);
    qDotGen.head(3) = imuData.segment(6,3);

    // 获取根节点的位置和姿态 Daniel 5.26
    Eigen::VectorXd rootPose = biped->getRootXyzRpy(qGen);//

    //--------------------------------------------------------------------------------------->//
    Eigen::VectorXd footStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd footStateTemp1 = Eigen::VectorXd::Zero(12,1);

    footStateTemp0 = biped->estFootPosVelInWorld(qGen, qDotGen, 0);
    rpyFootEst[0] = footStateTemp0.head(3);
    xyzFootEst[0] = footStateTemp0.segment(3,3);
    rpyDotFootEst[0] = footStateTemp0.segment(6,3);
    xyzDotFootEst[0] = footStateTemp0.tail(3);


    footStateTemp1 = biped->estFootPosVelInWorld(qGen, qDotGen, 1);
    rpyFootEst[1] = footStateTemp1.head(3);
    xyzFootEst[1] = footStateTemp1.segment(3,3);
    rpyDotFootEst[1] = footStateTemp1.segment(6,3);
    xyzDotFootEst[1] = footStateTemp1.tail(3);

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Daniel cout test//
std::cout << "State Estimation Test<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    std::cout << "Root position and orientation: " << rootPose.transpose() << endl<<endl;

    std::cout << "腰部xyz位置\t" << qGen[0] << "\t" << qGen[1] << "\t" << qGen[2] << "\t" << std::endl
        << "腰部rpy姿态\t" << qGen[3] << "\t" << qGen[4] << "\t" << qGen[5] << "\t" << std::endl
        << "LL joint\t" << qGen[6] << "\t" << qGen[7] << "\t" << qGen[8] << "\t" << qGen[9] << "\t" << qGen[10] << std::endl
        << "RL joint\t" << qGen[11] << "\t" << qGen[12] << "\t" << qGen[13] << "\t" << qGen[14] << "\t" << qGen[15] << std::endl << std::endl;

    std::cout << "xyzDotFootEst0---> " << xyzDotFootEst[0].transpose() << endl 
         << "rpyDotFootEst0---> " << rpyDotFootEst[0].transpose() << endl
         << "xyzFootEst0---> "    << xyzFootEst[0].transpose() << "-------------" << endl
         << "rpyFootEst0---> "    << rpyFootEst[0].transpose() << endl<<endl;

    std::cout << "xyzDotFootEst1---> " << xyzDotFootEst[1].transpose() << std::endl 
         << "rpyDotFootEst1---> " << rpyDotFootEst[1].transpose() << endl
         << "xyzFootEst1---> "    << xyzFootEst[1].transpose() << "-------------" << endl
         << "rpyFootEst1---> "    << rpyFootEst[1].transpose() << endl<<endl;

    std::cout << "FootXyzRpy from webots supervisor: " << endl
                << "Left: " << LeftSoleXyzRpyAct.transpose() << endl
                << "Right: " << RightSoleXyzRpyAct.transpose() << endl<<endl;           

std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl << endl;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 24.5.21//


    // ------------------------------------- update Kinematics ---------------//

    return true;
}



bool BipedController::motionPlan(){//Daniel 5.23
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Daniel Motion Plan Test//
std::cout << "Motion Plan Test<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    if (timeCs < 5.0 - 0.5*DT){
        //flag
        std::cout << "plan stage 1"  << std::endl;
        //torso
        xyzDotTorsoTgt << 0.0, 0.0, 0.0;
        rpyTorsoTgt << 0.0, 0.0, 0.0;
        rpyDotTorsoTgt << 0.0, 0.0, 0.0;
        xyzTorsoTgt = xyzTorsoEst;
        //foot
        xyzFootTgt = xyzFootEst[0];
        xyzDotFootTgt = xyzDotFootEst[0];
        rpyFootTgt = rpyFootEst[0];
        rpyDotFootTgt = rpyDotFootEst[0];
        //init
        xyzTorsoInit = xyzTorsoEst;


    }else {
        if(timeCs < 5.0 + 0.5*DT){
            time = 0.0;
            std::cout << "Tonight is Christmas Eve" << std::endl;
        }else { 
            // TORSO XYZ
            xyzTorsoTgt << xyzTorsoInit(0), xyzTorsoInit(1), 0.1*cos(time/1*PI)+xyzTorsoInit(2);
            xyzDotTorsoTgt << 0.0, 0.0, -0.1*PI*sin(time/1*PI);
            
            std::cout << "plan stage 2"  << std::endl;

            // //FOOT XYZ
            // xyzFootTgt << 0.0, 0.2, 0.1+0.06*(1-cos(timeTemp/2*PI));
            // xyzDotFootTgt << 0.0, 0.0, 0.03*PI*sin(timeTemp/2*PI);

            // //FOOT RPY
            // rpyFootTgt << PI/6*sin(timeTemp/2*PI), PI/6*sin(timeTemp/3*PI), PI/30*sin(timeTemp/4*PI);
            // rpyDotFootTgt << PI*PI/12*cos(timeTemp/2*PI), PI*PI/18*cos(timeTemp/3*PI), PI*PI/120*cos(timeTemp/4*PI);
        }
    }
std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl << endl;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 24.5.21//
    return true;
}



// bool BipedController::motionPlan(){
//     if (timeCs < 5.0 - 0.5*DT){
//         if (stanceLeg == 1){
//             xyzTorsoTgt << 0.0, 0.0, 1.0-0.03*time;
//             //xyzTorsoTgt << 0.0, 0.1, 0.56;
//         }
//         else{
//             xyzTorsoTgt << 0.0, 0.0, 0.86;
//         }
//         xyzDotTorsoTgt << 0.0, 0.0, 0.0;
//         rpyTorsoTgt << 0.0, 0.0, 0.0;
//         rpyDotTorsoTgt << 0.0, 0.0, 0.0;
//         // do not swing foot
//         xyzFootTgt = xyzFootEst;
//         xyzDotFootTgt = xyzDotFootEst;
//         rpyFootTgt = rpyFootEst;
//         rpyDotFootTgt = rpyDotFootEst;
//     }else {
//         if(timeCs < 5.0 + 0.5*DT){
//             time = 0.0;
//         }else{
//             double timeTemp;
//             timeTemp = time - 2;
//             // xyzTorsoTgt << 0.0, 0.1-0.03*sin(timeTemp/2*PI), 0.56;
//             // xyzDotTorsoTgt << 0.0, 0.0, 0.0;
//             // rpyTorsoTgt << 0.0, 0.0, 0.0;
//             // rpyDotTorsoTgt << 0.0, 0.0, 0.0;
//             // xyzFootTgt = xyzFootEst;
//             // xyzDotFootTgt = xyzDotFootEst;
//             // rpyFootTgt = rpyFootEst;
//             // rpyDotFootTgt = rpyDotFootEst;
//             xyzTorsoTgt << 0.0, -0.05, 0.02*cos(timeTemp/1*PI)+0.54;
//             xyzDotTorsoTgt << 0.0, 0.0, -0.02*PI*sin(timeTemp/1*PI);
//             xyzFootTgt = xyzFootEst;
//             xyzDotFootTgt = xyzDotFootEst;
//             //xyzFootTgt << 0.0, 0.2, 0.02+0.06*(1-cos(timeTemp/2*PI));
//             //xyzDotFootTgt << 0.0, 0.0, 0.03*PI*sin(timeTemp/2*PI);
//             //rpyFootTgt << PI/6*sin(timeTemp/2*PI), PI/6*sin(timeTemp/3*PI), PI/30*sin(timeTemp/4*PI);
//             //rpyDotFootTgt << PI*PI/12*cos(timeTemp/2*PI), PI*PI/18*cos(timeTemp/3*PI), PI*PI/120*cos(timeTemp/4*PI);
//         }
//     }
//     return true;
// }

bool BipedController::taskControl(){

    // ------------------------------ Update Robot Dynamics ---------------------------
    myWbc->updateRobotDynamics(qGen, qDotGen);

    // ------------------------------ Set PD gains ------------------------------------
    kpTorsoRpy = {100., 100., 100.};
    kdTorsoRpy = {6., 6., 6.};
    kpTorsoXyz = {400., 400., 400.};
    kdTorsoXyz = {30., 30., 30.};
    kpFootXyz = {200., 200., 200.};
    kdFootXyz = {15., 15., 15.};
    kpFootRpy = {100., 100., 100.};
    kdFootRpy = {6., 6., 6.};

    // ------------------------------ Calculate Reference ------------------------------
    // workspace
    torsoRpyRef = diag(kpTorsoRpy)*(rpyTorsoTgt - rpyTorsoEst) + diag(kdTorsoRpy)*(rpyDotTorsoTgt - rpyDotTorsoEst);
    torsoXyzRef = diag(kpTorsoXyz)*(xyzTorsoTgt - xyzTorsoEst) + diag(kdTorsoXyz)*(xyzDotTorsoTgt - xyzDotTorsoEst);
    footPosRef.head(3) = diag(kpFootRpy)*(rpyFootTgt - rpyFootEst[0]) + diag(kdFootRpy)*(rpyDotFootTgt - rpyDotFootEst[0]);
    footPosRef.segment(3,3) = diag(kpFootXyz)*(xyzFootTgt - xyzFootEst[0]) + diag(kdFootXyz)*(xyzDotFootTgt - xyzDotFootEst[0]); 
    // force
    forceRef = Eigen::VectorXd::Zero(nFc);
    forceChangeRef = forceOpt;

    // ------------------------------ set weights --------------------------------------
    weightTorsoPosition << 100., 100., 100.;
    weightTorsoOrientation << 100., 100., 100.;
    weightFootPosition << 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000.;
    weightFootForce << 4., 4., 1., 3., 3., 0.03, 4., 4., 1., 3., 3., 0.03;
    weightFootForceChange << 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.;
    // weightFootForce << 4., 4.;
    // weightFootForceChange << 1., 1.;//Daniel 24.5.21

    // ------------------------------ Update task & constraint -------------------------
    myWbc->updateTask("BipedTorsoPosRpy", torsoRpyRef, weightTorsoOrientation);
    myWbc->updateTask("BipedTorsoPosXyz", torsoXyzRef, weightTorsoPosition);
    myWbc->updateTask("BipedFootPosition", footPosRef, weightFootPosition);//error from here;   ```In [Task::check], vector dimensions do not match!``` >Done, change the dimension in head file<
    myWbc->updateTask("BipedFootForce", forceRef, weightFootForce);
    myWbc->updateTask("BipedFootForceChange", forceChangeRef, weightFootForceChange);
    myWbc->updateConstraint("BipedDynamicConsistency");
    myWbc->updateConstraint("BipedFrictionCone");
    myWbc->updateConstraint("BipedCenterOfPressure");
    myWbc->updateConstraint("BipedJointTorqueSaturation");

    // ------------------------------ Update bounds -------------------------------------
    if (timeCs < 5.0 - 0.5*DT){
        // lowerbounds(23) = 0.0;
        // upperbounds(23) = 3*20.0*GRAVITY;
        // lowerbounds(29) = 0.0;
        // upperbounds(29) = 3*20.0*GRAVITY;
        lowerbounds(25) = 0.0;
        upperbounds(26) = 10.0*GRAVITY;
        lowerbounds(25) = 0.0;
        upperbounds(26) = 10.0*GRAVITY;
    }else {
        if(timeCs < 5.0 + 0.5*DT){
            time = 0.0;
        }
        // lowerbounds.segment(18,6) = Eigen::VectorXd::Zero(6);
        // upperbounds.segment(18,6) = Eigen::VectorXd::Zero(6);
        // lowerbounds(29) = 0.0;
        // upperbounds(29) = 3*20.0*GRAVITY;
        lowerbounds(25) = 0.0;
        upperbounds(26) = 10.0*GRAVITY;
        lowerbounds(25) = 0.0;
        upperbounds(26) = 10.0*GRAVITY;//Daniel 5.22
    }
    myWbc->updateBound(lowerbounds, upperbounds);

    // ------------------------------ WBC solve ------------------------------------------
    // exe wbc slove
    myWbc->wbcSolve();

    // get some data from solved wbc //
    myWbc->getAuxiliaryDataInt(intData);
    nWsrRes = intData.at(0);
    simpleStatus = intData.at(1);

    myWbc->getAuxiliaryDataDouble(doubleData);
    costOpt = doubleData.at(0);
    cpuTimeRes = doubleData.at(1);

    // get wbc variables output and clac toq //
    // here shows the nV=G + Fc //
    if (simpleStatus == 0){
        myWbc->getResultOpt(varOpt);
        qDDotOpt = varOpt.head(nJg);
        forceOpt = varOpt.tail(nFc);
        tauOpt = biped->eqCstrMatTau * varOpt + biped->eqCstrMatTauBias;
    }

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
