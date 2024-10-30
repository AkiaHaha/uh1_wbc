#include "taskDefinition.h"


//=====================================================================//
// Task base class functions
//=====================================================================//
Task::Task(const std::string & taskName, int taskDim, int varDim){
    name = taskName;
    dim = taskDim;
    varDof = varDim;
    taskMatA = Eigen::MatrixXd::Zero(dim, varDof);
    taskVecB = Eigen::VectorXd::Zero(dim);
    wei = Eigen::VectorXd::Zero(dim);
    ref = Eigen::VectorXd::Zero(dim);
}

bool Task::updateRefence(const Eigen::VectorXd & newRef){
    ref = newRef;
    return true;
}

bool Task::updateWeight(const Eigen::VectorXd & newWei){

    #ifdef USE_ERROR

        for(int i=0; i<newWei.size(); ++i) {
            if(newWei(i) < 0.) {
                throw NotPositive(
                    "Error: in [Task::updateWeight] the elements must be positive!"
                );
                break;
            }
        }
    #else

        for(int i=0; i<newWei.size(); ++i) {
            try {
                if(newWei(i) < 0.) {
                    throw NotPositive(
                        "Error: in [Task::updateWeight] the elements must be positive!"
                    );
                }
            } catch (NotPositive notPositive) {
                std::cout << notPositive.what() << std::endl;
                return false;
            }
        }
    #endif

    wei = newWei;
    return true;
}

//=====================================================================//
// Dynamics
//=====================================================================//
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

//=====================================================================//
// Kinematics of Control task
//=====================================================================//
bool PelvisRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.topRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.head(3);
    return true;
}

bool PelvisXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.bottomRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.tail(3);
    return true;
}

bool TorsoRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.topRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.head(3);
    return true;
}

bool TorsoXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) = robot.floatBaseJacoTc.J.bottomRows(3);
    taskVecB = ref - robot.floatBaseJacoTc.JdotQdot.tail(3);
    return true;
}

bool FootXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) << robot.quadContactJacoTc.J.block(3, 0, 3, robot.NJG),
                                  robot.quadContactJacoTc.J.block(9, 0, 3, robot.NJG);
    taskVecB << robot.quadContactJacoTc.JdotQdot.segment(3,3), robot.quadContactJacoTc.JdotQdot.segment(9,3);                                  
    taskVecB = ref - taskVecB;
    return true;
}

bool FootRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) << robot.quadContactJacoTc.J.block(0, 0, 3, robot.NJG),
                                  robot.quadContactJacoTc.J.block(6, 0, 3, robot.NJG);
    taskVecB << robot.quadContactJacoTc.JdotQdot.segment(0,3), robot.quadContactJacoTc.JdotQdot.segment(6,3);       
    taskVecB = ref - taskVecB;
    return true;
}

bool ArmXyz::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) << robot.quadContactJacoTc.J.block(15, 0, 3, robot.NJG),
                                  robot.quadContactJacoTc.J.block(21, 0, 3, robot.NJG);
    taskVecB << robot.quadContactJacoTc.JdotQdot.segment(15,3), robot.quadContactJacoTc.JdotQdot.segment(21,3);
    taskVecB = ref - taskVecB;
    return true;
}

bool ArmRpy::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.leftCols(robot.NJG) << robot.quadContactJacoTc.J.block(12, 0, 3, robot.NJG),
                                  robot.quadContactJacoTc.J.block(18, 0, 3, robot.NJG);
    taskVecB << robot.quadContactJacoTc.JdotQdot.segment(12,3), robot.quadContactJacoTc.JdotQdot.segment(18,3); 
    taskVecB = ref - taskVecB;
    return true;
}

bool FootForce::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool ArmForce::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.rightCols(robot.NFC) = Eigen::MatrixXd::Identity(robot.NFC, robot.NFC);
    taskVecB = ref;
    return true;
}

bool GVLimitation::update(const AGIROBOT::RobotDynamics &robot){
    taskMatA.block(0,6,19,19) = 0.001*Eigen::MatrixXd::Identity(19,19);
    taskVecB = ref;
    return true;
}