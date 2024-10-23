#ifndef AGIROBOT_EXAMPLE_WEBOTS_INTERFACE_H
#define AGIROBOT_EXAMPLE_WEBOTS_INTERFACE_H


#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/GPS.hpp>
#include <webots/Node.hpp>

#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "operation.h"

#ifndef PI
    #define PI 3.141592654
#endif // PI

#ifndef TIME_STEP
    #define TIME_STEP    (1)
#endif // TIME_STEP

#ifndef SAMPLE_TIME
    #define SAMPLE_TIME  (0.001f)
#endif // SAMPLE_TIME

#ifndef LEFTFOOT
    #define LEFTFOOT 0
#endif // LEFTFOOT

#ifndef RIGHTFOOT
    #define RIGHTFOOT 1
#endif // RIGHTFOOT

using namespace webots;

/**
 * @brief The Derivative class
 * Derivative in S-Domain:
 *            s
 * D(s) = -----------
 *          cs + 1
 *
 * e.g. c = 1e-4
 *
 * Tusting approximation:
 *                    1 - z^-1
 *  S(z) = alpha * ---------------
 *                    1 + z^-1
 */
class Derivative {
    public:
        Derivative();
        Derivative( double dT, double c );
        void init(double dT, double c, double initValue);
        double mSig( double sigIn );
    private:
        double a0, a1, b0, b1;
        double sigInPrev;
        double sigOutPrev;
};

struct webotsState
{
    Eigen::VectorXd jointPosAct = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointVelAct = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointTorAct = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd imuAct = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd footGrfAct = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd wristItaAct = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d pelvisRpyAct = Eigen::Vector3d::Zero();
    Eigen::Vector3d pelvisDRpyAct = Eigen::Vector3d::Zero();
    Eigen::Vector3d pelvisDDXyzAct = Eigen::Vector3d::Zero();

    Eigen::VectorXd pelvisXyzDXyzAct = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd LeftSoleXyzRpyAct = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd RightSoleXyzRpyAct = Eigen::VectorXd::Zero(6);
    // Eigen::VectorXd LeftArmHandXyzRpyAct = Eigen::VectorXd::Zero(6);
    // Eigen::VectorXd RightArmHandXyzRpyAct = Eigen::VectorXd::Zero(6);
};

/**
 * @brief The WebotsRobot class
 */
class WebotsRobot{
public:
    webots::Supervisor *robot = new Supervisor();

    void initWebots();
    void deleteRobot();
    bool readData(double simTime, webotsState &robotState);
    bool setMotorPos(const Eigen::VectorXd & jointPosTar);
    bool setMotorTau(const Eigen::VectorXd & jointTauTar);
    bool setMotorPosTau(const Eigen::VectorXd & jointTauPosMixed);
    bool setMotorPosTau2(const Eigen::VectorXd & jointTauPosMixed);
    bool setMotorPosTau3(const Eigen::VectorXd & jointTauPosMixed);
    bool setMotorPosTau4(const Eigen::VectorXd & jointTauPosMixed);

private:
    Eigen::VectorXd getMotorPos();
    Eigen::VectorXd getMotorTau();
    Eigen::Vector3d getPelvisAcc();
    Eigen::VectorXd getBiFootForce6D();
    Eigen::VectorXd getBiWristForce6D();
    Eigen::Vector3d rotm2Rpy(const Eigen::Matrix3d & rotm);
    std::vector<Motor*> legMotor;
    std::vector<PositionSensor*> legSensor;
    std::vector<TouchSensor*> forceSensorFootSole;
    std::vector<TouchSensor*> forceSensorArmWrist;

    InertialUnit *imu;
    Gyro *gyro;
    Accelerometer *accelerometer;
    
    Node* Pelvis;
    Node* SoleLeft;
    Node* SoleRight;
    // Node* ArmHandLeft;
    // Node* ArmHandRight;

    std::vector<Derivative> dRpy;
    std::vector<Derivative> dJnt;
};

#endif // AGIROBOT_EXAMPLE_WEBOTS_INTERFACE_H
