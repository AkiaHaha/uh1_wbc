#ifndef OPERATION_H_
#define OPERATION_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <cstdarg>

#include "robotDynamics.h"

#ifndef NG
    #define NG 25
#endif // DELTA-T

#ifndef NJ
    #define NJ 19
#endif // DELTA-T

#ifndef NFCC4
    #define NFCC4 24
#endif // DELTA-T

#ifndef NFCC2
    #define NFCC2 12
#endif // DELTA-T

#ifndef NV
    #define NV 49 //Force@@
    // #define NV 37 //Force@@
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

Eigen::VectorXd fillVector(double firstValue, double secondValue);
std::vector<double> fillVector2(double value, int length);
Eigen::Vector3d fillVector3(double value);
#endif//OPERATION_H_