#include "taskDefinitionBiped.h"
bool BipedFloatingBaseDynamics::update(const HUMANOID::RobotDynamics &robot){
    taskMatA << robot.selMatFloatingBase * robot.inertiaMat,
                -robot.selMatFloatingBase * robot.quadContactJacoTc.J.transpose();
    taskVecB =  -robot.selMatFloatingBase * robot.nonlinearBias;
    return true;
}

bool BipedCentroidalMomentum::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix;
    taskVecB = ref - robot.centroidalMomentumBiased;
    return true;
}

bool BipedLinearMomentum::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.bottomRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.tail(3);
    return true;
}

bool BipedAngularMomentum::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.topRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.head(3);
    return true;
} 

bool BipedTorsoPosition::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J;
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot;
    return true;
}

bool BipedTorsoRpy::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.topRows(3);
    taskMatA.block(0,16,3,9) = Eigen::MatrixXd::Zero(3,9);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.head(3);
    return true;
}

bool BipedTorsoXyz::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.bottomRows(3);
    taskMatA.block(0,16,3,9) = Eigen::MatrixXd::Zero(3,9);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.tail(3);
    return true;
}

bool BipedComRpy::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.comJacoTc.J.topRows(3);
    taskMatA.block(0,16,3,9) = Eigen::MatrixXd::Zero(3,9);
    taskVecB = ref - robot.comJacoTc.JdotQdot.head(3);
    return true;
}

bool BipedComXyz::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.comJacoTc.J.bottomRows(3);
    taskMatA.block(0,16,3,9) = Eigen::MatrixXd::Zero(3,9);
    taskVecB = ref - robot.comJacoTc.JdotQdot.tail(3);
    return true;
}

bool BipedUpTorsoRpy::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.upTorsoJacoTc.J.topRows(3);
    taskMatA.block(0,16,3,9) = Eigen::MatrixXd::Zero(3,9);
    taskVecB = ref - robot.upTorsoJacoTc.JdotQdot.head(3);
    return true;
}

bool BipedUpTorsoXyz::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.upTorsoJacoTc.J.bottomRows(3);
    taskMatA.block(0,16,3,9) = Eigen::MatrixXd::Zero(3,9);
    taskVecB = ref - robot.upTorsoJacoTc.JdotQdot.tail(3);
    return true;
}

bool BipedFootPose::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.biContactJacoTc.J;
    taskMatA.block(0,16,12,9) = Eigen::MatrixXd::Zero(12,9);
    taskVecB = ref - robot.biContactJacoTc.JdotQdot;
    return true;
}

bool BipedArmPose::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.quadContactJacoTc.J.bottomRows(12);
    taskMatA.block(0,6,12,11) = Eigen::MatrixXd::Zero(12,11);
    taskVecB = ref - robot.quadContactJacoTc.JdotQdot.bottomRows(12);
    return true;
}

bool BipedArmPoseStatic::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.block(0,17,8,8) = Eigen::MatrixXd::Identity(8,8);
    taskVecB = ref;
    return true;
}

bool BipedFootForce::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.rightCols(12) = Eigen::MatrixXd::Identity(12, 12);
    taskVecB = ref;
    return true;
}

bool BipedArmForce::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.rightCols(12) = Eigen::MatrixXd::Identity(12, 12);
    taskVecB = ref;
    return true;
}

bool BipedFootForceChange::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool QuadSoleForceChange::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool QuadSoleForce::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}
bool QuadSolePosition::update(const HUMANOID::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.quadContactJacoTc.J;
    taskVecB = ref - robot.quadContactJacoTc.JdotQdot;
    return true;
}

bool GlobalVelocityLimitation::update(const HUMANOID::RobotDynamics &robot){
    // taskMatA.block(0,6,19,19) = 0.001*Eigen::MatrixXd::Identity(19,19);
    taskMatA.block(0,6,19,19) = Eigen::MatrixXd::Identity(19,19);
    taskVecB = ref;
    return true;
}