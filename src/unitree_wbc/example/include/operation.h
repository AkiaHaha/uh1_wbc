#ifndef OPERATION_H_
#define OPERATION_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <cstdarg>

#include "robotDynamics.h"

#ifndef NG
    #define NG 17
#endif // DELTA-T

#ifndef NJ
    #define NJ 11
#endif // DELTA-T

#ifndef NFCC
    #define NFCC 12
#endif // DELTA-T

#ifndef NV
    #define NV 29
#endif // DELTA-T

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