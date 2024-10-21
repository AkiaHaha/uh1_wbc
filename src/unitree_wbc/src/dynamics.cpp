#include "dynamics.h"
#include <ErrorMsg.h>

namespace AGIROBOT {

bool RobotDynamics::setJntStates(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot){
    jntPositions = q;
    jntVelocities = qdot;
    return true;
}

bool RobotDynamics::displayDynamicInformation(){
    std::cout << "================ Dynamic Information : ================ " << std::endl;
    std::cout << "Joint Angles Q^T = " << std::endl
              << jntPositions.transpose() << std::endl
              << "Joint Velocities Qdot^T = " << std::endl
              << jntVelocities.transpose() << std::endl
              << "Inertia matrix M = " << std::endl
              << inertiaMat << std::endl
              << "Inverse of inertia matrix M^-1 = " << std::endl
              << invInertiaMat << std::endl
              << "Nonlinear terms bng^T = " << std::endl
              << nonlinearBias.transpose() << std::endl
              << "centroidalMomentum = " << std::endl
              << centroidalMomentumMatrix << std::endl
              << "centroidalMomentumBiased^T = " << std::endl
              << centroidalMomentumBiased.transpose() << std::endl;
    std::cout << "================ Dynamic Information End ================" << std::endl;
    return true;
}
} // namespace AGIROBOT
