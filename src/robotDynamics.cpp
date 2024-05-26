/**
 *	This file is part of TAICHI.
 *
 *	TAICHI -- Task Arrangement In Control HIerarchy.
 *	Copyright (C) 2015-2021 Beijing Research Institute of UBTECH Robotics.
 *	All rights reserved.
 *
 *	Licensed under the Apache License 2.0. See LICENSE for more details.
 */

/**
 * @file RobotDynamics.cpp
 * @brief Function implementation part of class RobotDynamics
 * @author Jiajun Wang
 * @date 2020-2021
 * @version alpha
 */

#include "robotDynamics.h"
#include <ErrorMsg.h>

namespace TAICHI {

bool RobotDynamics::setJntStates(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot){
    if (check(q, NJG) && check(qdot, NJG)){
        jntPositions = q;
        jntVelocities = qdot;
        calcWbcDependenceDone = false;
    }else{
        return false;
    }
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

bool RobotDynamics::isCalcWbcDependenceDone(){
    return calcWbcDependenceDone;
}

bool RobotDynamics::check(const Eigen::MatrixXd & M, int row, int col){
    #ifdef USE_ERROR
        if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [RobotDynamics::check], matrix dimensions do not match" );
                }
        }
    #else
        try {
            if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [RobotDynamics::check], matrix dimensions do not match" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        } 
    #endif
    return true;
}

bool RobotDynamics::check(const Eigen::VectorXd & v, int row){
    #ifdef USE_ERROR
        if(v.rows() != row) {
            throw InvalidDimension(
                "Error : In [RobotDynamics::check], vector dimensions do not match!" );
            }
    #else

        try {
            if(v.rows() != row) {
                throw InvalidDimension(
                    "In [RobotDynamics::check], vector dimensions do not match!" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        }
    #endif
    return true;
}

} // namespace TAICHI
