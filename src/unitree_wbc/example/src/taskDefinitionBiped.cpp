#include "taskDefinitionBiped.h"

bool BipedFloatingBaseDynamics::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA << robot.selMatFloatingBase * robot.inertiaMat,
                -robot.selMatFloatingBase * robot.quadContactJacoTc.J.transpose();
    taskVecB = - robot.selMatFloatingBase * robot.nonlinearBias;
    return true;
}

bool BipedCentroidalMomentum::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix;
    taskVecB = ref - robot.centroidalMomentumBiased;
    return true;
}

bool BipedLinearMomentum::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.bottomRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.tail(3);
    return true;
}

bool BipedAngularMomentum::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.topRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.head(3);
    return true;
} 

bool BipedPelvisPosition::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.pelvisJacoTc.J;
    taskVecB = ref - robot.pelvisJacoTc.JdotQdot;
    return true;
}

bool BipedPelvisRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.pelvisJacoTc.J.topRows(3);
    taskVecB = ref - robot.pelvisJacoTc.JdotQdot.head(3);
    return true;
}

bool BipedPelvisXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.pelvisJacoTc.J.bottomRows(3);
    taskVecB = ref - robot.pelvisJacoTc.JdotQdot.tail(3);
    return true;
}

bool BipedTorsoRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.torsoJacoTc.J.topRows(3);
    taskVecB = ref - robot.torsoJacoTc.JdotQdot.head(3);
    return true;
}

bool BipedTorsoXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.torsoJacoTc.J.bottomRows(3);
    taskVecB = ref - robot.torsoJacoTc.JdotQdot.tail(3);
    return true;
}

bool BipedFootPosition::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.biContactJacoTc.J;
    taskVecB = ref - robot.biContactJacoTc.JdotQdot;
    return true;
}

bool BipedFootForce::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool BipedFootForceChange::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool QuadSoleForceChange::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}
bool QuadSoleForce::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}
bool QuadSolePosition::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.quadContactJacoTc.J;
    taskVecB = ref - robot.quadContactJacoTc.JdotQdot;
    return true;
}

bool GlobalVelocityLimitation::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.block(0,6,19,19) = 0.001*Eigen::MatrixXd::Identity(19,19);
    // taskMatA.block(0,6,19,19) = 6*Eigen::MatrixXd::Identity(19,19);
    taskVecB = ref;
    return true;
}
