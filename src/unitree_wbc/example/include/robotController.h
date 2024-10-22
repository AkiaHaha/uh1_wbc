#ifndef AGIROBOT_EXAMPLE_BIPEDCONTROLLER_H
#define AGIROBOT_EXAMPLE_BIPEDCONTROLLER_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>

#include <Eigen/Dense>

#include "wqpWbc.h"
#include "hqpWbc.h"

#include "dynamics.h"
#include "robotDynamics.h"
#include "taskDefinition.h"
#include "constraintDefinition.h"
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

// #define USING_TWIST

class RobotController
{
public:
    RobotController();
    ~RobotController();
    bool update(double timeCtrlSys, webotsState & robotStateSim);
    bool getValueTauOpt(Eigen::VectorXd & jntTorOpt);
    bool getValuePosCurrent(Eigen::VectorXd & jntPosCur);
    bool getValueQdd(Eigen::VectorXd & Qdd);

private:
    ConfigParams configParams;
    webotsState robotStateSim;
    bool stateEstimation(webotsState & robotStateSim);                         
    bool motionPlan();
    bool taskControl();
    RobotDynamics * robotDynamics;
    AGIROBOT::Wbc * myWbc;

    int stanceLeg{1};                       // 1: right; 0:left
    double timeCs{0.0};                     // time of CS (Control System)
    int tick{0};                            // the tick-tack for time accumulation
    double time{0.0};                       // run time (sec) for current behavior
    int nJg{NG};
    int nJa{NJ};
    int nFc{NFCC4};
    double dsp{};
    
    Eigen::VectorXd biFootForce6D = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd qActuated = Eigen::VectorXd::Zero(nJa); ;
    Eigen::VectorXd qDotActuated = Eigen::VectorXd::Zero(nJa);
    Eigen::VectorXd groundReactiveForce = Eigen::VectorXd::Zero(2);//Daniel nFc->2
    Eigen::VectorXd qGen = Eigen::VectorXd::Zero(nJg);
    Eigen::VectorXd qDotGen = Eigen::VectorXd::Zero(nJg);

#ifdef USING_TWIST
    // === Vectors of twist in 12 Dof; the Seq.: Rpy,Xyz,dRpy,dXyz;
    // Estimation - Est
    Eigen::VectorXd pelvisTwistEst = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd torsoTwistEst = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftFootTwistEst = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightFootTwistEst = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftArmTwistEst = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightArmTwistEst = Eigen::VectorXd::Zero(12);

    // Target - Tgt
    Eigen::VectorXd pelvisTwistTgt = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd torsoTwistTgt = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftFootTwistTgt = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightFootTwistTgt = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftArmTwistTgt = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightArmTwistTgt = Eigen::VectorXd::Zero(12);

    // Initial - Init
    Eigen::VectorXd pelvisTwistInit = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd torsoTwistInit = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftFootTwistInit = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightFootTwistInit = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd leftArmTwistInit = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd rightArmTwistInit = Eigen::VectorXd::Zero(12);
#else
    // Estimation - Est
    Eigen::Vector3d xyzPelvisEst = Eigen::Vector3d::Zero(), xyzDotPelvisEst = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyPelvisEst = Eigen::Vector3d::Zero(), rpyDotPelvisEst = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzTorsoEst = Eigen::Vector3d::Zero(), xyzDotTorsoEst = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyTorsoEst = Eigen::Vector3d::Zero(), rpyDotTorsoEst = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmEst[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // Target - Tgt
    Eigen::Vector3d xyzPelvisTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzDotPelvisTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyPelvisTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyDotPelvisTgt = Eigen::Vector3d::Zero();

    Eigen::Vector3d xyzTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzDotTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyDotTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    Eigen::Vector3d xyzArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    // Initial - Init
    Eigen::Vector3d xyzFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    Eigen::Vector3d xyzArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
#endif

    // Reference state
    Eigen::Vector3d pelvisXyzRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d pelvisRpyRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d torsoXyzRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d torsoRpyRef = Eigen::Vector3d::Zero();
    Eigen::VectorXd footArmPosRef = Eigen::VectorXd::Zero(NFCC4);//@Danny
    Eigen::VectorXd footForceRef = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd footForceChangeRef = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd footArmForceRef = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd footArmForceChangeRef = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd floatBaseDynamicRef = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd GVLimitationRef = Eigen::VectorXd::Zero(19);

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
    double soleRight{0.015};//# this item modified based on uh1 foot//@Danny240510
    Eigen::VectorXd lowerbounds = -jointQddotLimit * Eigen::VectorXd::Ones(nV);
    Eigen::VectorXd upperbounds = jointQddotLimit * Eigen::VectorXd::Ones(nV);

    // Auxiliary Data
    std::vector<int> intData;
    std::vector<double> doubleData;

    // Additional body control task @Daniel240523
    Eigen::Vector3d xyzPelvisInit = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzTorsoInit = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyPelvisInit = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyTorsoInit = Eigen::Vector3d::Zero();
    int flagTimeSetZero{};
    int flagEstFirst{};
};

#endif //AGIROBOT_EXAMPLE_BIPEDCONTROLLER_H
