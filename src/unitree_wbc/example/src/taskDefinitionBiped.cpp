#include "taskDefinitionBiped.h"

bool BipedFloatingBaseDynamics::update(const TAICHI::RobotDynamics &robot){
    taskMatA << robot.selMatFloatingBase * robot.inertiaMat,
                -robot.selMatFloatingBase * robot.contactJacoTc.J.transpose();
    taskVecB = - robot.selMatFloatingBase * robot.nonlinearBias;
}

bool BipedCentroidalMomentum::update(const TAICHI::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix;
    taskVecB = ref - robot.centroidalMomentumBiased;
    return true;
}

bool BipedLinearMomentum::update(const TAICHI::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.bottomRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.tail(3);
}

bool BipedAngularMomentum::update(const TAICHI::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.topRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.head(3);
    return true;
} 

bool BipedTorsoPosition::update(const TAICHI::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J;
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot;
    return true;
}

bool BipedTorsoPosRpy::update(const TAICHI::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.topRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.head(3);
    return true;
}

bool BipedTorsoPosXyz::update(const TAICHI::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.bottomRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.tail(3);
    return true;
}

bool BipedFootPosition::update(const TAICHI::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.contactJacoTc.J;
    taskVecB = ref - robot.contactJacoTc.JdotQdot;
    return true;
}

bool BipedFootForce::update(const TAICHI::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool BipedFootForceChange::update(const TAICHI::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

// bool QuadSoleForceChange::update(const TAICHI::RobotDynamics &robot){
//     taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
//     taskVecB = ref;
//     return true;
// }
// bool QuadSoleForce::update(const TAICHI::RobotDynamics &robot){
//     taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
//     taskVecB = ref;
//     return true;
// }
// bool QuadSolePosition::update(const TAICHI::RobotDynamics &robot){
//     taskMatA.leftCols(robot.NJG) = robot.contactJacoTc.J;
//     taskVecB = ref - robot.contactJacoTc.JdotQdot;
//     return true;
// }