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

class Integrator {
public:
    Integrator();
    double Integrate(double Qdd);

private:
    double Qd_prev;  // Previous first-order derivative value
    double Q_prev;   // Previous zero-order integrator value
    const double dt; // Sample time (1 ms)
};
#endif//OPERATION_H_