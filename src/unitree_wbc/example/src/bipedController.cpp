#include "bipedController.h"
#include "robotDynamics.h"
#include "robotDynamicsBiped.h"


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
    TAICHI::Task * ptrBipedUpTorsoPosRpy = new BipedUpTorsoPosRpy("BipedUpTorsoPosRpy", 3, nV);
    TAICHI::Task * ptrBipedUpTorsoPosXyz = new BipedUpTorsoPosXyz("BipedUpTorsoPosXyz", 3, nV);
    // TAICHI::Task * ptrForce = new BipedFootForce("Force", nFc, nV);
    // TAICHI::Task * ptrForceChange = new BipedFootForceChange("ForceChange", nFc, nV);
    // TAICHI::Task * ptrPosition = new BipedFootPosition("Position", nFc, nV);
    TAICHI::Task * ptrForce = new QuadSoleForce("Force", NFCC2, nV);
    TAICHI::Task * ptrForceChange = new QuadSoleForceChange("ForceChange", NFCC2, nV);
    TAICHI::Task * ptrPosition = new QuadSolePosition("Position", NFCC4, nV);

    TAICHI::Constraint * ptrBipedDynamicConsistency = new BipedDynamicConsistency("BipedDynamicConsistency", 6, nV);
    TAICHI::Constraint * ptrBipedFrictionCone = new BipedFrictionCone("BipedFrictionCone", 8, nV);
    TAICHI::Constraint * ptrBipedJointTorqueSaturation = new BipedJointTorqueSaturation("BipedJointTorqueSaturation", NJ, nV);
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
    myWbc->addTask(ptrBipedUpTorsoPosRpy, 0);
    myWbc->addTask(ptrBipedUpTorsoPosXyz, 0);
    myWbc->addTask(ptrForce, 0);
    myWbc->addTask(ptrForceChange, 0);
    myWbc->addTask(ptrPosition, 0);
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
    cout << "******************** Torso ****************************" << endl;
    cout << rpyTorsoEst.transpose() << endl;
    cout << xyzTorsoEst.transpose() << endl;

    //<<<upTorso pose//
    Eigen::VectorXd poseTemp0 = Eigen::VectorXd::Zero(12,1);
    poseTemp0 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 0);

    rpyUpTorsoEst = poseTemp0.head(3);
    xyzUpTorsoEst = poseTemp0.segment(3,3);
    rpyDotUpTorsoEst = poseTemp0.segment(6,3);
    xyzDotUpTorsoEst = poseTemp0.tail(3);

    cout << "******************** Up Torso ****************************" << endl;
    // cout << akiaPrint2(footStateTemp0, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    cout << rpyUpTorsoEst.transpose() << endl;
    cout << xyzUpTorsoEst.transpose() << endl;

    //<<<foot ee posVel//
    Eigen::VectorXd footStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd footStateTemp1 = Eigen::VectorXd::Zero(12,1);

    footStateTemp0 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 1);
    rpyFootEst[0] = footStateTemp0.head(3);
    xyzFootEst[0] = footStateTemp0.segment(3,3);
    rpyDotFootEst[0] = footStateTemp0.segment(6,3);
    xyzDotFootEst[0] = footStateTemp0.tail(3);
    cout << "Left foot" << "-------------------------------" << endl;
    // cout << akiaPrint2(footStateTemp0, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    cout << footStateTemp0.segment(3,3).transpose() << endl;

    footStateTemp1 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 2);
    rpyFootEst[1] = footStateTemp1.head(3);
    xyzFootEst[1] = footStateTemp1.segment(3,3);
    rpyDotFootEst[1] = footStateTemp1.segment(6,3);
    xyzDotFootEst[1] = footStateTemp1.tail(3);
    cout << "Right foot" << "-------------------------------" << endl;
    // cout << akiaPrint2(footStateTemp1, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    cout << footStateTemp1.segment(3,3).transpose() << endl;

    // <<<arm ee posVel//
    Eigen::VectorXd armStateTemp0 = Eigen::VectorXd::Zero(12,1);
    Eigen::VectorXd armStateTemp1 = Eigen::VectorXd::Zero(12,1);

    armStateTemp0 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 3);
    xyzArmEst[0] = armStateTemp0.head(3);
    xyzArmEst[0] = armStateTemp0.segment(3,3);
    rpyDotArmEst[0] = armStateTemp0.segment(6,3);
    xyzDotArmEst[0] = armStateTemp0.tail(3);
    cout << "Left arm" << "-------------------------------" << endl;
    // cout << akiaPrint2(armStateTemp0, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    cout << armStateTemp0.segment(3,3).transpose() << endl;

    armStateTemp1 = biped->estFootArmPosVelInWorld(qGen, qDotGen, 4);
    rpyArmEst[1] = armStateTemp1.head(3);
    xyzArmEst[1] = armStateTemp1.segment(3,3);
    rpyDotArmEst[1] = armStateTemp1.segment(6,3);
    xyzDotArmEst[1] = armStateTemp1.tail(3);
    cout << "Right arm" << "-------------------------------" << endl;
    // cout << akiaPrint2(armStateTemp1, 12, 4, 3, "rpy", 3, "***xyz", 3, "rpyDot", 3, "xyzDot") << endl;
    cout << armStateTemp1.segment(3,3).transpose() << endl;

    //<<<arm ee xyzRpy from supervisor//
    cout << "arm ee xyzRpy from supervisor" << "-----------------------------------" << endl
        << "Left: " << LeftArmHandXyzRpyAct.head(3).transpose() << endl
        << "Right: " << RightArmHandXyzRpyAct.head(3).transpose() << endl<<endl;   

    if(flagEstFirst == 0){
        flagEstFirst = 1;

        xyzTorsoInit = xyzTorsoEst;
        xyzUpTorsoInit = xyzUpTorsoEst;

        rpyTorsoInit = rpyTorsoEst;
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


    return true;
}


bool BipedController::motionPlan(){//Daniel 5.23

        
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
        // xyzTorsoTgt = xyzTorsoInit;
        // xyzDotTorsoTgt << 0.0, 0.0, 0.0;
        xyzTorsoTgt << xyzTorsoInit(0), xyzTorsoInit(1), -0.02*sin(time/1*PI)+xyzTorsoInit(2);
        xyzDotTorsoTgt << 0.0, 0.0, -0.02*PI*cos(time/1*PI);
        rpyTorsoTgt << 0.0, 0.0, 0.0;
        rpyDotTorsoTgt << 0.0, 0.0, 0.0;

        //Up torso
        // xyzUpTorsoTgt = xyzUpTorsoInit;
        // xyzDotUpTorsoTgt << 0.0, 0.0, 0.0;

        xyzUpTorsoTgt << xyzUpTorsoInit(0), xyzUpTorsoInit(1), -0.02*sin(time/1*PI)+xyzUpTorsoInit(2);
        xyzDotUpTorsoTgt << 0.0, 0.0, -0.02*PI*cos(time/1*PI);
        rpyUpTorsoTgt << 0.0, 0.0, 0.0;
        rpyDotUpTorsoTgt << 0.0, 0.0, 0.0;
        
        //foot
        xyzFootTgt[0] = xyzFootInit[0];
        xyzDotFootTgt[0] = xyzDotFootInit[0];
        rpyFootTgt[0] = rpyFootInit[0];
        rpyDotFootTgt[0] = rpyDotFootInit[0];

        xyzFootTgt[1] = xyzFootInit[1];
        xyzDotFootTgt[1] = xyzDotFootInit[1];
        rpyFootTgt[1] = rpyFootInit[1];
        rpyDotFootTgt[1] = rpyDotFootInit[1];

        xyzArmTgt[0] << xyzArmInit[0].x(), xyzArmInit[0].y(), xyzArmInit[0].z()+0.1*sin(time/1*PI);
        xyzDotArmTgt[0] << xyzDotArmInit[0].x(), xyzDotArmInit[0].y(), xyzDotArmInit[0].z()+0.1*PI*cos(time/1*PI);
        // xyzArmTgt[0] << xyzArmInit[0].x(), xyzArmInit[0].y(), xyzArmInit[0].z();
        // xyzDotArmTgt[0] << xyzDotArmInit[0].x(), xyzDotArmInit[0].y(), xyzDotArmInit[0].z();
        rpyArmTgt[0] = rpyArmInit[0];
        rpyDotArmTgt[0] = rpyDotArmInit[0];


        // xyzArmTgt[1] = xyzArmInit[1];
        // xyzDotArmTgt[1] = xyzDotArmInit[1];
        xyzArmTgt[1] << xyzArmInit[1].x(), xyzArmInit[1].y(), xyzArmInit[1].z()-0.1*sin(time/1*PI);
        xyzDotArmTgt[1] << xyzDotArmInit[1].x(), xyzDotArmInit[1].y(), xyzDotArmInit[1].z()-0.1*PI*cos(time/1*PI);
        rpyArmTgt[1] = rpyArmInit[1];
        rpyDotArmTgt[1] = rpyDotFootInit[1];

    return true;
}

bool BipedController::taskControl(){

    // ------------------------------ Update Robot Dynamics ---------------------------
    myWbc->updateRobotDynamics(qGen, qDotGen);

    // ------------------------------ Set PD gains ------------------------------------
    kpTorsoRpy = {1000., 1000., 1000.};
    kdTorsoRpy = {6., 6., 6.};
    kpTorsoXyz = {600., 600., 600.};
    kdTorsoXyz = {30., 30., 30.};

    kpUpTorsoRpy = {1200, 1200, 5000};
    kdUpTorsoRpy = fillVector2(100, 3);
    kpUpTorsoXyz = fillVector2(1200, 3);
    kdUpTorsoXyz = fillVector2(60, 3);

    kpFootXyz = {800., 800., 800.};
    kdFootXyz = {15., 15., 15.};
    kpFootRpy = {500., 500., 500.};
    kdFootRpy = {6., 6., 6.};

    kpArmXyz = fillVector2(1200, 3);
    kdArmXyz = fillVector2(120, 3);
    kpArmRpy = fillVector2(1000, 3);
    kdArmRpy = fillVector2(100, 3);
    // ------------------------------ Calculate Reference ------------------------------
    // torso
    torsoRpyRef = diag(kpTorsoRpy)*(rpyTorsoTgt - rpyTorsoEst) + diag(kdTorsoRpy)*(rpyDotTorsoTgt - rpyDotTorsoEst);
    torsoXyzRef = diag(kpTorsoXyz)*(xyzTorsoTgt - xyzTorsoEst) + diag(kdTorsoXyz)*(xyzDotTorsoTgt - xyzDotTorsoEst);

    //up torso
    upTorsoRpyRef = diag(kpUpTorsoRpy)*(rpyUpTorsoTgt - rpyUpTorsoEst) + diag(kdUpTorsoRpy)*(rpyDotUpTorsoTgt - rpyDotUpTorsoEst);
    upTorsoXyzRef = diag(kpUpTorsoXyz)*(xyzUpTorsoTgt - xyzUpTorsoEst) + diag(kdUpTorsoXyz)*(xyzDotUpTorsoTgt - xyzDotUpTorsoEst);
    // left foot
    footArmPosRef.segment(0,3) = diag(kpFootRpy)*(rpyFootTgt[0] - rpyFootEst[0]) + diag(kdFootRpy)*(rpyDotFootTgt[0] - rpyDotFootEst[0]);
    footArmPosRef.segment(3,3) = diag(kpFootXyz)*(xyzFootTgt[0] - xyzFootEst[0]) + diag(kdFootXyz)*(xyzDotFootTgt[0] - xyzDotFootEst[0]); 
    //right foot 
    footArmPosRef.segment(6,3) = diag(kpFootRpy)*(rpyFootTgt[1] - rpyFootEst[1]) + diag(kdFootRpy)*(rpyDotFootTgt[1] - rpyDotFootEst[1]);
    footArmPosRef.segment(9,3) = diag(kpFootXyz)*(xyzFootTgt[1] - xyzFootEst[1]) + diag(kdFootXyz)*(xyzDotFootTgt[1] - xyzDotFootEst[1]); 
    // left arm
    footArmPosRef.segment(12,3) = diag(kpArmRpy)*(rpyArmTgt[0] - rpyArmEst[0]) + diag(kdArmRpy)*(rpyDotArmTgt[0] - rpyDotArmEst[0]);
    footArmPosRef.segment(15,3) = diag(kpArmXyz)*(xyzArmTgt[0] - xyzArmEst[0]) + diag(kdArmXyz)*(xyzDotArmTgt[0] - xyzDotArmEst[0]); 
    //right arm 
    footArmPosRef.segment(18,3) = diag(kpArmRpy)*(rpyArmTgt[1] - rpyArmEst[1]) + diag(kdArmRpy)*(rpyDotArmTgt[1] - rpyDotArmEst[1]);
    footArmPosRef.segment(21,3) = diag(kpArmXyz)*(xyzArmTgt[1] - xyzArmEst[1]) + diag(kdArmXyz)*(xyzDotArmTgt[1] - xyzDotArmEst[1]);  
    // footArmPosRef.tail(12).setZero();
    // cout << "********************" << endl;
    // cout << footArmPosRef.head(12).transpose() << endl;
    // cout << footArmPosRef.tail(12).transpose() << endl<<endl;

    // force
    footArmforceRef = forceOpt;
    footArmforceChangeRef = Eigen::VectorXd::Zero(NFCC2);
    // footArmforceRef = forceOpt;


    cout << "foot force ref" << endl;
    akiaPrint1(forceOpt, NFCC2, 4, 6, 6, 6, 6);
    // cout<<endl;
    // akiaPrint1(footArmforceChangeRef, NFCC2, 4, 6, 6, 6, 6);

    
    // ------------------------------ set weights --------------------------------------
    weightTorsoPosition = fillVector3(600000);                                           
    weightTorsoOrientation = fillVector3(600000); 

    weightUpTorsoPosition = fillVector3(300000);                                        
    weightUpTorsoOrientation << 300000, 300000, 3000000;
    
    weightFootArmPosition = fillVector(1000, 1000);   
    weightFootArmForceChange << 4., 4., 1., 3., 3., 0.3, 4., 4., 1., 3., 3., 0.3;
                        // 0.4, 0.4, 0.1, 0.3, 0.3, 0.003, 0.4, 0.4, 0.1, 0.3, 0.3, 0.003;
    weightFootArmForce << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
                        // 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    // ------------------------------ Update task & constraint -------------------------                 
    myWbc->updateTask("BipedTorsoPosRpy", torsoRpyRef, weightTorsoOrientation);
    myWbc->updateTask("BipedTorsoPosXyz", torsoXyzRef, weightTorsoPosition);
    myWbc->updateTask("BipedUpTorsoPosRpy", upTorsoRpyRef, weightUpTorsoOrientation);
    myWbc->updateTask("BipedUpTorsoPosXyz", upTorsoXyzRef, weightUpTorsoPosition);
    myWbc->updateTask("Position", footArmPosRef, weightFootArmPosition);
    myWbc->updateTask("ForceChange", footArmforceChangeRef, weightFootArmForceChange);
    myWbc->updateTask("Force", footArmforceRef, weightFootArmForce);
    
    myWbc->updateConstraint("BipedDynamicConsistency");
    myWbc->updateConstraint("BipedFrictionCone");
    myWbc->updateConstraint("BipedCenterOfPressure");
    myWbc->updateConstraint("BipedJointTorqueSaturation");

    // ------------------------------ Update bounds -------------------------------------
    lowerbounds(NG+5) = 0.0;
    upperbounds(NG+5) = 90.0*GRAVITY;
    lowerbounds(NG+11) = 0.0;
    upperbounds(NG+11) = 90.0*GRAVITY;

    lowerbounds(16) = 0;
    upperbounds(16) = 0;

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
    // here shows the nV = G + Fc //
    if (simpleStatus == 0){
        myWbc->getResultOpt(varOpt);
        qDDotOpt = varOpt.head(nJg);
        forceOpt = varOpt.tail(nFc);
        tauOpt = biped->eqCstrMatTau * varOpt + biped->eqCstrMatTauBias;
    }else{
        cout<<endl  << "QP failed";
        varOpt.setZero();
        qDDotOpt = varOpt.head(nJg);
        forceOpt = varOpt.tail(nFc);
        tauOpt = biped->eqCstrMatTau * varOpt + biped->eqCstrMatTauBias;
    }
        cout << endl << "output*** " << timeCs << endl;
        cout << endl << "qDDotOpt-----------------" << endl;
        akiaPrint1(qDDotOpt, NJ, 5, 5, 5, 1, 4, 4);
        cout << endl << "forceOpt-----------------" << endl;
        akiaPrint1(forceOpt, NFCC2, 2, 6, 6);
        cout << endl << "tauOpt-----------------" << endl;
        akiaPrint1(tauOpt, NJ, 5, 5, 5, 1, 4, 4);
        cout << endl << "varOpt-----------------" << endl;
        akiaPrint1(varOpt, NV, 5, 6, 11, 8, 6, 6);
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
