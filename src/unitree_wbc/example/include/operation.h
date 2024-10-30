#ifndef OPERATION_H_
#define OPERATION_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <cstdarg>

#include "dynamics.h"

#ifndef NG25
    #define NG25 25
#endif // DELTA-T

#ifndef NJ19
    #define NJ19 19
#endif // DELTA-T

#ifndef NFCC24
    #define NFCC24 24
#endif // DELTA-T

#ifndef NFCC2
    #define NFCC2 12
#endif // DELTA-T

#ifndef NV49
    #define NV49 49 //Force@@
#endif // DELTA-T

#ifndef NV25
    #define NV25 25 //Force@@
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

Eigen::MatrixXd createDiagonalMatrix(const Eigen::VectorXd& vec);

#endif//OPERATION_H_