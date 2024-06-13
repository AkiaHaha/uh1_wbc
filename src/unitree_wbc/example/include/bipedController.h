#ifndef TAICHI_EXAMPLE_BIPEDCONTROLLER_H
#define TAICHI_EXAMPLE_BIPEDCONTROLLER_H

#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <Eigen/Dense>

#include "wqpWbc.h"
#include "robotDynamicsBiped.h"
#include "taskDefinitionBiped.h"
#include "constraintDefinitionBiped.h"
#include "operation.h"

#ifndef PI
    #define PI 3.141592654
#endif // PI

#ifndef GRAVITY
    #define GRAVITY 9.80665
#endif // GRAVITY-CONST

#ifndef EPSILON
    #define EPSILON 1e-6
#endif // EPSILON

#ifndef DT
    #define DT 1e-3
#endif // DELTA-T

class BipedController
{
public:

    BipedController();
    ~BipedController();

    bool update(double timeCtrlSys, const Eigen::VectorXd & imuData,
                const Eigen::VectorXd & jntPos, const Eigen::VectorXd & jntVel,
                const Eigen::VectorXd & forceSensorData, 
                const Eigen::VectorXd & LeftSoleXyzRpyAct, const Eigen::VectorXd & RightSoleXyzRpyAct);
    bool getValueTauOpt(Eigen::VectorXd & jntTorOpt);
    bool getValuePosCurrent(Eigen::VectorXd & jntPosCur);
    bool getValueQdd(Eigen::VectorXd & Qdd);


private:
    bool stateEstimation(const Eigen::VectorXd & imuData,
                        const Eigen::VectorXd & jntPos, const Eigen::VectorXd & jntVel,
                        const Eigen::VectorXd & forceSensorData, 
                        const Eigen::VectorXd & LeftSoleXyzRpyAct, const Eigen::VectorXd & RightSoleXyzRpyAct);                         
    bool motionPlan();
    bool taskControl();
    Eigen::MatrixXd diag(const std::vector<double>& diagElement);

    // ----------------------- pointers ------------------------------
    RobotDynamicsBiped * biped;
    TAICHI::Wbc * myWbc;
    // ----------------------- pointers ------------------------------

    // ----------------------- robot state ---------------------------
    // NOTE: in this example, we set the right leg as the stance leg.
    int stanceLeg{1};                       // 1: right; 0:left
    double timeCs{0.0};                     // time of CS (Control System)
    int tick{0};                            // the tick-tack for time accumulation
    double time{0.0};                       // run time (sec) for current behavior
    int nJg{16};
    int nJa{10};
    int nFc{12};//Daniel 24.5.21
    
    Eigen::VectorXd qActuated = Eigen::VectorXd::Zero(nJa); ;
    Eigen::VectorXd qDotActuated = Eigen::VectorXd::Zero(nJa);
    Eigen::VectorXd groundReactiveForce = Eigen::VectorXd::Zero(2);//Daniel nFc->2
    Eigen::VectorXd qGen = Eigen::VectorXd::Zero(nJg);
    Eigen::VectorXd qDotGen = Eigen::VectorXd::Zero(nJg);
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
    // ----------------------- robot state ---------------------------

    // ----------------------- plan desired --------------------------
    Eigen::Vector3d xyzTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyzDotTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyTorsoTgt = Eigen::Vector3d::Zero();
    Eigen::Vector3d rpyDotTorsoTgt = Eigen::Vector3d::Zero();

    Eigen::Vector3d xyzFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    Eigen::Vector3d xyzFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotFootInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    Eigen::Vector3d xyzArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmTgt[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};

    Eigen::Vector3d xyzArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d xyzDotArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    Eigen::Vector3d rpyDotArmInit[2] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    // ----------------------- plan desired --------------------------

    // ----------------------- task control --------------------------
    // reference
    Eigen::Vector3d torsoXyzRef = Eigen::Vector3d::Zero();
    Eigen::Vector3d torsoRpyRef = Eigen::Vector3d::Zero();
    Eigen::VectorXd footArmPosRef = Eigen::VectorXd::Zero(nFc);//Daniel
    Eigen::VectorXd footArmforceRef = Eigen::VectorXd::Zero(nFc);
    Eigen::VectorXd footArmforceChangeRef = Eigen::VectorXd::Zero(nFc);

    // result of QP-WBC
    int nV{nJg + nFc};
    Eigen::VectorXd varOpt = Eigen::VectorXd::Zero(nV);
    Eigen::VectorXd qDDotOpt = Eigen::VectorXd::Zero(nJg);
    Eigen::VectorXd forceOpt = Eigen::VectorXd::Zero(nFc);
    Eigen::VectorXd tauOpt = Eigen::VectorXd::Zero(nJa);
    int simpleStatus{0};       // 0: solve; 1: fail
    int nWsrRes{0};            // the number of working set recalculations actually performed
    double cpuTimeRes{0.};     // the cputime actually performed
    double costOpt{0.};        // QP cost function value
    // PD gains
    std::vector<double> kpTorsoXyz{0., 0., 0.};
    std::vector<double> kdTorsoXyz{0., 0., 0.};
    std::vector<double> kpTorsoRpy = {0., 0., 0.};
    std::vector<double> kdTorsoRpy = {0., 0., 0.};
    std::vector<double> kpFootArmXyz{0., 0., 0.};
    std::vector<double> kdFootArmXyz{0., 0., 0.};
    std::vector<double> kpFootArmRpy{0., 0., 0.};
    std::vector<double> kdFootArmRpy{0., 0., 0.};

    // parameters
    double muStatic{0.6};
    double jointTauLimit{200.0};
    double jointQddotLimit{1e4};
    double myInfinity{1e6};
    double CopFactor{0.9};  
    double soleFront{0.186};
    double soleBack{0.094};
    double soleLeft{0.015};
    double soleRight{0.015};
    // weights
    Eigen::Vector3d weightTorsoPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightTorsoOrientation = Eigen::Vector3d::Zero();
    Eigen::VectorXd weightFootArmPosition = Eigen::VectorXd::Zero(nFc);
    Eigen::VectorXd weightFootArmForce = Eigen::VectorXd::Zero(nFc);
    Eigen::VectorXd weightFootArmForceChange = Eigen::VectorXd::Zero(nFc);

    // bounds
    Eigen::VectorXd lowerbounds = -jointQddotLimit * Eigen::VectorXd::Ones(nV);
    Eigen::VectorXd upperbounds = jointQddotLimit * Eigen::VectorXd::Ones(nV);
    // Auxiliary Data
    std::vector<int> intData;
    std::vector<double> doubleData;
    // ----------------------- task control --------------------------

    //<------------------------------------------Daniel 24.5.23-----//
    Eigen::Vector3d xyzTorsoInit = Eigen::Vector3d::Zero();
    int flagTimeSetZero{};
    int flagEstFirst{};
    //------------------------------------------------------------->//
};

#endif //TAICHI_EXAMPLE_BIPEDCONTROLLER_H
