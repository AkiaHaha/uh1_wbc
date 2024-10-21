#include "taskDefinition.h"

bool FloatingBaseDynamics::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA << robot.selMatFloatingBase * robot.inertiaMat,
                -robot.selMatFloatingBase * robot.quadContactJacoTc.J.transpose();
    taskVecB = - robot.selMatFloatingBase * robot.nonlinearBias;
    return true;
}

bool CentroidalMomentum::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix;
    taskVecB = ref - robot.centroidalMomentumBiased;
    return true;
}

bool LinearMomentum::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.bottomRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.tail(3);
    return true;
}

bool AngularMomentum::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.centroidalMomentumMatrix.topRows(3);
    taskVecB = ref - robot.centroidalMomentumBiased.head(3);
    return true;
} 

bool PelvisPosition::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J;
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot;
    return true;
}

bool PelvisPosRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.topRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.head(3);
    return true;
}

bool PelvisPosXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.bottomRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.tail(3);
    return true;
}

bool TorsoPosRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.topRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.head(3);
    return true;
}

bool TorsoPosXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.bottomRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.tail(3);
    return true;
}

bool FootPosition::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.biContactJacoTc.J;
    taskVecB = ref - robot.biContactJacoTc.JdotQdot;
    return true;
}

bool FootForce::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool FootForceChange::update(const AGIROBOT::RobotDynamics &robot){
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
