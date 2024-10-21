#include "constraintDefinition.h"
using namespace std;

// Dynamic consistency
bool DynamicConsistency::update(const AGIROBOT::RobotDynamics &robot){
    cstrMatC.leftCols(robot.NJG) = robot.selMatFloatingBase * robot.inertiaMat;
    // cstrMatC.rightCols(robot.NFC) = -robot.selMatFloatingBase * robot.biContactJacoTc.J.transpose();//Force@@
    cstrMatC.rightCols(robot.NFC) = -robot.selMatFloatingBase * robot.quadContactJacoTc.J.transpose();//Force@@
    lbC = - robot.selMatFloatingBase * robot.nonlinearBias;
    ubC = - robot.selMatFloatingBase * robot.nonlinearBias; 
    return true; 
}

// Friction cone
FrictionCone::FrictionCone(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){
    fricMat = Eigen::MatrixXd::Zero(4, 6);
}

bool FrictionCone::setParameter(const std::vector<double> &params){
    if (params.size() >= 2){
        muStaticFriction = params.at(0);
        myInfinity = params.at(1);
    }else{
        std::cout << "Error : Friction cone params empty !" << std::endl;
        return false;
    }
    return true;
}

bool FrictionCone::update(const AGIROBOT::RobotDynamics &robot){
    fricMat <<  0.0, 0.0, 0.0, 1.0, 0.0, -muStaticFriction,
                0.0, 0.0, 0.0, -1.0, 0.0, -muStaticFriction,
                0.0, 0.0, 0.0, 0.0, 1.0, -muStaticFriction,
                0.0, 0.0, 0.0, 0.0, -1.0, -muStaticFriction;
    cstrMatC.block(0, NG, 4, 6) = fricMat;
    cstrMatC.block(4, NG+6, 4, 6) = fricMat;//@Danny240521
    lbC  = -myInfinity * Eigen::VectorXd::Ones(8);
    ubC  = Eigen::VectorXd::Zero(8);
    return true;
}

// Center of pressure
CenterOfPressure::CenterOfPressure(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){
    copMat = Eigen::MatrixXd::Zero(4, 6);
}

bool CenterOfPressure::setParameter(const std::vector<double> &params){
    if (params.size() >= 6){
        sole2Front = params.at(0);
        sole2Back = params.at(1);
        sole2Left = params.at(2);
        sole2Right = params.at(3);
        copFactor = params.at(4);
        myInfinity = params.at(5);
    }else{
        std::cout << "Error : Center of pressure params empty !" << std::endl;
        return false;
    }
    return true;
}

bool CenterOfPressure::update(const AGIROBOT::RobotDynamics &robot){
    copMat <<   0.0, -1.0, 0.0, 0.0, 0.0, -sole2Front * copFactor,
                0.0, 1.0, 0.0, 0.0, 0.0, -sole2Back * copFactor,
                1.0, 0.0, 0.0, 0.0, 0.0, -sole2Left * copFactor,
               -1.0, 0.0, 0.0, 0.0, 0.0, -sole2Right * copFactor;
    cstrMatC.block(0, NG, 4, 6) = copMat;
    cstrMatC.block(4, NG+6, 4, 6) = copMat;//Daniel 5.22
    lbC  = -myInfinity * Eigen::VectorXd::Ones(8);
    ubC  = Eigen::VectorXd::Zero(8);
    return true;
}

// Joint torque limitation
bool JointTorqueSaturation::setParameter(const std::vector<double> &params){
    if(params.size() >= 1){
        jointTauLimit = params.at(0);
    }else{
        std::cout << "Error : Joint torque saturation params empty !" << std::endl;
        return false;
    }
    return true;
}

bool JointTorqueSaturation::update(const AGIROBOT::RobotDynamics &robot){
    cstrMatC = robot.eqCstrMatTau;///< NJA, NJG+NFC
    lbC = - jointTauLimit * Eigen::VectorXd::Ones(robot.NJA) - robot.eqCstrMatTauBias;
    ubC = jointTauLimit * Eigen::VectorXd::Ones(robot.NJA) - robot.eqCstrMatTauBias;
    return true;
}