#include "webotsInterface.h"

using namespace webots;
using namespace std;

void WebotsRobot::initWebots()
{
    // motors
    legMotor.resize(10);
    legMotor[0] = robot->getMotor("left_hip_yaw_joint");
    legMotor[1] = robot->getMotor("left_hip_roll_joint");
    legMotor[2] = robot->getMotor("left_hip_pitch_joint");
    legMotor[3] = robot->getMotor("left_knee_joint");
    legMotor[4] = robot->getMotor("left_ankle_joint");
    legMotor[5] = robot->getMotor("right_hip_yaw_joint");
    legMotor[6] = robot->getMotor("right_hip_roll_joint");
    legMotor[7] = robot->getMotor("right_hip_pitch_joint");
    legMotor[8] = robot->getMotor("right_knee_joint");
    legMotor[9] = robot->getMotor("right_ankle_joint");

    
    // motor sensors
    legSensor.resize(10);
    legSensor[0] = robot->getPositionSensor("left_hip_yaw_joint_sensor");
    legSensor[1] = robot->getPositionSensor("left_hip_roll_joint_sensor");
    legSensor[2] = robot->getPositionSensor("left_hip_pitch_joint_sensor");
    legSensor[3] = robot->getPositionSensor("left_knee_joint_sensor");
    legSensor[4] = robot->getPositionSensor("left_ankle_joint_sensor");//torqueSensor-0
    legSensor[5] = robot->getPositionSensor("right_hip_yaw_joint_sensor");
    legSensor[6] = robot->getPositionSensor("right_hip_roll_joint_sensor");
    legSensor[7] = robot->getPositionSensor("right_hip_pitch_joint_sensor");
    legSensor[8] = robot->getPositionSensor("right_knee_joint_sensor");
    legSensor[9] = robot->getPositionSensor("right_ankle_joint_sensor");//torqueSensor-1
    
    // other sensors
    imu = robot->getInertialUnit("inertial_unit_upperBody");
    accelerometer = robot->getAccelerometer("accelerometer_upperBody");
    
    Waist = robot->getFromDef("UnitreeH1");

    // enable
    for (int i = 0; i < 10; i++) {
        legMotor[i]->enableTorqueFeedback(TIME_STEP);
    }
    for (int i = 0; i < 10; i++) {
        legSensor[i]->enable(TIME_STEP);
    }
    imu->enable(TIME_STEP);
    accelerometer->enable(TIME_STEP);

    // Derivative
    dRpy.resize(3);
    for (int i = 0; i < 3; i++) {
        dRpy.at(i).init(SAMPLE_TIME, 1e-3, 0.);
    }
    dJnt.resize(10);
    for (int i = 0; i < 10; i++) {
        dJnt.at(i).init(SAMPLE_TIME, 1e-3, 0.);
    }
}


void WebotsRobot::deleteRobot()
{
    // delete robot;
}


bool WebotsRobot::readData(double simTime, webotState & robotStateSim)
{
    // Motor pos
    robotStateSim.jointPosAct = getMotorPos();
    
    // Motor vel
    if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6){
        for (int i = 0; i < 10; i++) {
            dJnt.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.jointPosAct(i));
        }
    }
    for (int i = 0; i < 10; i++) {
        robotStateSim.jointVelAct(i) = dJnt.at(i).mSig(robotStateSim.jointPosAct(i));
    }

    // Motor torque
    robotStateSim.jointTorAct = getMotorTau();

    // IMU Data 9-dof
    const double* rotmArray = Waist->getOrientation(); 
    Eigen::Matrix3d rotm;
    rotm << rotmArray[0], rotmArray[1], rotmArray[2],
            rotmArray[3], rotmArray[4], rotmArray[5],
            rotmArray[6], rotmArray[7], rotmArray[8];
    robotStateSim.waistRpyAct = rotm2Rpy(rotm);
  
    if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6){
        for (int i = 0; i < 3; i++) {
            dRpy.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.waistRpyAct(i));
        }
    }
    for (int i = 0; i < 3; i++) {
        robotStateSim.waistRpyVelAct(i) = dRpy.at(i).mSig(robotStateSim.waistRpyAct(i));
    }

    // waist acc
    robotStateSim.waistXyzAccAct = getWaistAcc();

    //waist xyz PosVel
    const double* xyzArray1 = Waist->getPosition();
    const double* xyzArray2 = Waist->getVelocity();
    robotStateSim.waistXyzPosVelAct << xyzArray1[0], xyzArray1[1], xyzArray1[2], xyzArray2[0], xyzArray2[1], xyzArray2[2];

    //data summary//
    robotStateSim.imu9DAct << robotStateSim.waistRpyAct, robotStateSim.waistXyzPosVelAct, robotStateSim.waistRpyVelAct;

    //External Force
    robotStateSim.footGrfAct = getFootForce2D();

    return true;
}

bool WebotsRobot::setMotorPos(const Eigen::VectorXd& jointPosTar) {
    for (int i = 0; i < 10; i++) {
        legMotor[i]->setPosition(jointPosTar(i, 0));
    }
    return true;
}

bool WebotsRobot::setMotorTau(const Eigen::VectorXd& jointTauTar) {
    for (int i = 0; i < 10; i++) {
        legMotor[i]->setTorque(jointTauTar(i, 0));
    }
    return true;
}

Eigen::VectorXd WebotsRobot::getMotorPos() {
    Eigen::VectorXd Q = Eigen::VectorXd::Zero(10);
    for (int i = 0; i < 10; i++) {
        Q(i, 0) = legSensor[i]->getValue();
    }
    return Q;
}

Eigen::VectorXd WebotsRobot::getMotorTau() {
    Eigen::VectorXd Tau = Eigen::VectorXd::Zero(10);
    for (int i = 0; i < 10; i++) {
        Tau(i, 0) = legMotor[i]->getTorqueFeedback();
    }
    return Tau;
}

Eigen::Vector3d WebotsRobot::getWaistAcc() {
    const double* data = accelerometer->getValues();
    Eigen::Vector3d acceleration(data[0], data[1], data[2]);
    return acceleration;
}



Eigen::VectorXd WebotsRobot::getFootForce(const int& footFlag) {
    Eigen::VectorXd torqueY(1); 
    double toq;
    switch (footFlag) {
        case LEFTFOOT: {
            toq = legMotor[4]->getTorqueFeedback();
            break;
        }
        case RIGHTFOOT: {
            toq = legMotor[9]->getTorqueFeedback();
            break;
        }
        default: {
            std::cout << "footFlag is wrong, return the values of LEFTFOOT by default!" << std::endl;
            toq = legMotor[4]->getTorqueFeedback();
            break;
        }
    }
    torqueY(0) = toq; 
    return torqueY;
}
Eigen::VectorXd WebotsRobot::getFootForce2D() {
    Eigen::VectorXd LFootForce = getFootForce(LEFTFOOT);
    Eigen::VectorXd RFootForce = getFootForce(RIGHTFOOT);
    Eigen::VectorXd FootForce = Eigen::VectorXd::Zero(2);
    FootForce << LFootForce,  RFootForce;
    return FootForce;
}
//---------------------------------------------------------------------------------------------------Daniel--//

Eigen::Vector3d WebotsRobot::rotm2Rpy(const Eigen::Matrix3d & rotm) {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    rpy(0) = atan2(rotm(2,1), rotm(2,2));
    rpy(1) = atan2(-rotm(2,0), sqrt(rotm(2,1) * rotm(2,1) + rotm(2,2) * rotm(2,2)));
    rpy(2) = atan2(rotm(1,0), rotm(0,0));
    return rpy;
}

// **************************************************************************************************************//

Derivative :: Derivative () {}

Derivative :: Derivative ( double dT, double c ) {
    double alpha( 2. / dT );
    this -> a0 = c * alpha + 1.;
    this -> a1 = ( 1. - c * alpha ) / this -> a0;
    this -> b0 = alpha / this -> a0;
    this -> b1 = -alpha / this -> a0;
    this -> a0 = 1.;
    this -> sigInPrev = 0.;
    this -> sigOutPrev = 0.;
}

void Derivative :: init ( double dT, double c, double initValue ) {
    double alpha( 2. / dT );
    this -> a0 = c * alpha + 1.;
    this -> a1 = ( 1. - c * alpha ) / this -> a0;
    this -> b0 = alpha / this -> a0;
    this -> b1 = -alpha / this -> a0;
    this -> a0 = 1.;
    this -> sigInPrev = initValue;
    this -> sigOutPrev = initValue;
}

double Derivative :: mSig( double sigIn ) {
    double sigOut( 0. );
    sigOut = (sigIn - sigInPrev) * 1000.;
    sigOutPrev = sigOut;
    sigInPrev = sigIn;
    return sigOut;
}
