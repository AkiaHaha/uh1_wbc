#ifndef AGIROBOT_EXAMPLE_BIPEDCONTROLLER_H
#define AGIROBOT_EXAMPLE_BIPEDCONTROLLER_H

#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

#include <Eigen/Dense>

#include "wqpWbc.h"
#include "hqpWbc.h"

#include "robotDynamicsBiped.h"
#include "taskDefinitionBiped.h"
#include "constraintDefinitionBiped.h"
#include "webotsInterface.h"
#include "operation.h"
#include "configParams.h"

#ifndef PI
    #define PI 3.141592654
#endif // PI

#ifndef GRAVITY
    #define GRAVITY 9.8
#endif // GRAVITY-CONST

#ifndef EPSILON
    #define EPSILON 1e-6
#endif // EPSILON

#ifndef DT
    #define DT 1e-3
#endif // DELTA-T

class RobotController
{
public:
    RobotController();
    ~RobotController();

    //=====================================================
    // Functions for invocation
    //=====================================================
    bool update(double timeCtrlSys, webotsState & robotStateSim);
    bool getValueTauOpt(Eigen::VectorXd & jntTorOpt);
    bool getValuePosCurrent(Eigen::VectorXd & jntPosCur);
    bool getValueQdd(Eigen::VectorXd & Qdd);


private:
    ConfigParams configParams;
    bool stateEstimation(webotsState & robotStateSim);                         
    bool motionPlan();
    bool taskControl();
    Eigen::MatrixXd diag(const std::vector<double>& diagElement);

    //=====================================================
    // Pointers of main class
    //=====================================================
    RobotDynamicsBiped * robotDynamics;
    AGIROBOT::Wbc * myWbc;


    //=====================================================
    // Constant and parameters of the robot's and QP
    //=====================================================

    // Current
    int stanceLeg{1};                       // 1: right; 0:left
    double timeCs{0.0};                     // time of CS (Control System)
    int tick{0};                            // the tick-tack for time accumulation
    double time{0.0};                       // run time (sec) for current behavior
    int nJg{NG};
    int nJa{NJ};
    int nFc{NFCC4};
    
    Eigen::VectorXd qActuated = Eigen::VectorXd::Zero(nJa); ;
    Eigen::VectorXd qDotActuated = Eigen::VectorXd::Zero(nJa);
    Eigen::VectorXd groundReactiveForce = Eigen::VectorXd::Zero(2);//Daniel nFc->2
    Eigen::VectorXd qGen = Eigen::VectorXd::Zero(nJg);
    Eigen::VectorXd qDotGen = Eigen::VectorXd::Zero(nJg);
    Eigen::Vector3d xyzTorsoEst = Eigen::Vector3d::Zero(), xyzDotTorsoEst = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyTorsoEst = Eigen::Vector3d::Zero(), rpyDotTorsoEst = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzUpTorsoEst = Eigen::Vector3d::Zero(), xyzDotUpTorsoEst = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyUpTorsoEst = Eigen::Vector3d::Zero(), rpyDotUpTorsoEst = Eigen::Vector3d::Zero();
    // Eigen::Vector3d xyzFootEst = Eigen::Vector3d::Zero(), xyzDotFootEst = Eigen::Vector3d::Zero();
    // Eigen::Vector3d rpyFootEst = Eigen::Vector3d::Zero(), rpyDotFootEst = Eigen::Vector3d::Zero();

    Eigen::Vector3d xyzFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // Target
    double dsp{};
    Eigen::Vector3d xyzTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzDotTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyDotTorsoTgt = Eigen::Vector3d::Zero();

    Eigen::Vector3d xyzUpTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzDotUpTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyUpTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyDotUpTorsoTgt = Eigen::Vector3d::Zero();
    // Eigen::Vector3d xyzFootTgt = Eigen::Vector3d::Zero();
    // Eigen::Vector3d xyzDotFootTgt = Eigen::Vector3d::Zero();
    // Eigen::Vector3d rpyFootTgt = Eigen::Vector3d::Zero();
    // Eigen::Vector3d rpyDotFootTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    Eigen::Vector3d xyzFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // Eigen::Vector3d xyzArmTgt = Eigen::Vector3d::Zero();
    // Eigen::Vector3d xyzDotArmTgt = Eigen::Vector3d::Zero();
    // Eigen::Vector3d rpyArmTgt = Eigen::Vector3d::Zero();
    // Eigen::Vector3d rpyDotArmTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    Eigen::Vector3d xyzArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // Reference state
    Eigen::Vector3d torsoXyzRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d torsoRpyRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d upTorsoXyzRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d upTorsoRpyRef = Eigen::Vector3d::Zero();
    // Eigen::VectorXd footPosRef = Eigen::VectorXd::Zero(nFc);//Daniel 24.5.21
    // Eigen::VectorXd footPosRef = Eigen::VectorXd::Zero(12);//Daniel
    // Eigen::VectorXd forceRef = Eigen::VectorXd::Zero(nFc);
    // Eigen::VectorXd forceChangeRef = Eigen::VectorXd::Zero(nFc);
    Eigen::VectorXd footArmPosRef = Eigen::VectorXd::Zero(NFCC4);//Daniel
    Eigen::VectorXd footForceRef = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd footForceChangeRef = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd footArmForceRef = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd footArmForceChangeRef = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd floatBaseDynamicRef = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd GlobalVelocityLimitationRef = Eigen::VectorXd::Zero(19);

    // Result of QP
    int nV{NV};
    Eigen::VectorXd varOpt = Eigen::VectorXd::Zero(nV);
    Eigen::VectorXd qDDotOpt = Eigen::VectorXd::Zero(nJg);
    Eigen::VectorXd forceOpt = Eigen::VectorXd::Zero(NFCC4);//Force
    Eigen::VectorXd tauOpt = Eigen::VectorXd::Zero(nJa);
    int simpleStatus{0};       // 0: solve; 1: fail
    int nWsrRes{0};            // the number of working set recalculations actually performed
    double cpuTimeRes{0.};     // the cputime actually performed
    double costOpt{0.};        // QP cost function value

    // Constraint parameters of friction cone and torque limitation
    double muStatic{0.6};
    double jointTauLimit{200.0};
    double jointQddotLimit{1e4};
    double myInfinity{1e6};
    double CopFactor{0.9};  
    double soleFront{0.186};
    double soleBack{0.094};
    double soleLeft{0.015};
    double soleRight{0.015};//this item modified based on uh1 foot//Daniel.24.5.10//
    Eigen::VectorXd lowerbounds = -jointQddotLimit * Eigen::VectorXd::Ones(nV);
    Eigen::VectorXd upperbounds = jointQddotLimit * Eigen::VectorXd::Ones(nV);

    // Auxiliary Data
    std::vector<int> intData;
    std::vector<double> doubleData;

    // Additional body control task @Daniel240523
    Eigen::Vector3d xyzTorsoInit = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzUpTorsoInit = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyTorsoInit = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyUpTorsoInit = Eigen::Vector3d::Zero();
    int flagTimeSetZero{};
    int flagEstFirst{};
};

#endif //AGIROBOT_EXAMPLE_BIPEDCONTROLLER_H
