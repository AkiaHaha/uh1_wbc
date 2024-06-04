#ifndef OPERATION_H_
#define OPERATION_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <cstdarg>

#include "robotDynamics.h"

void akiaPrint1(const Eigen::VectorXd &vector, int length, int numRows, ...);
std::string akiaPrint2(const Eigen::VectorXd &vector, int length, int numRows, ...);
void akiaPrint3(const Eigen::VectorXd &vector, int length, int numRows, ...);

RigidBodyDynamics::Body BodyAkia(double mass,
                                const RigidBodyDynamics::Math::Vector3d& com,
                                double Ixx, double Iyy, double Izz,
                                double Ixy, double Ixz, double Iyz);
#endif//OPERATION_H_