#include "webotsInterface.h"

using namespace webots;
using namespace std;

void WebotsRobot::initWebots()
{
    // motors
    legMotor.resize(NJ);
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
    legMotor[10] = robot->getMotor("torso_joint");
    legMotor[11] = robot->getMotor("left_shoulder_pitch_joint");
    legMotor[12] = robot->getMotor("left_shoulder_roll_joint");
    legMotor[13] = robot->getMotor("left_shoulder_yaw_joint");
    legMotor[14] = robot->getMotor("left_elbow_joint");
    legMotor[15] = robot->getMotor("right_shoulder_pitch_joint");
    legMotor[16] = robot->getMotor("right_shoulder_roll_joint");
    legMotor[17] = robot->getMotor("right_shoulder_yaw_joint");
    legMotor[18] = robot->getMotor("right_elbow_joint");
    
    // motor sensors
    legSensor.resize(NJ);
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
    legSensor[10] = robot->getPositionSensor("torso_joint_sensor");
    legSensor[11] = robot->getPositionSensor("left_shoulder_pitch_joint_sensor");
    legSensor[12] = robot->getPositionSensor("left_shoulder_roll_joint_sensor");
    legSensor[13] = robot->getPositionSensor("left_shoulder_yaw_joint_sensor");
    legSensor[14] = robot->getPositionSensor("left_elbow_joint_sensor");
    legSensor[15] = robot->getPositionSensor("right_shoulder_pitch_joint_sensor");
    legSensor[16] = robot->getPositionSensor("right_shoulder_roll_joint_sensor");
    legSensor[17] = robot->getPositionSensor("right_shoulder_yaw_joint_sensor");
    legSensor[18] = robot->getPositionSensor("right_elbow_joint_sensor");
    
    // other sensors
    imu = robot->getInertialUnit("inertial_unit_upperBody");
    accelerometer = robot->getAccelerometer("accelerometer_upperBody");
    
    // Key control node
    Pelvis = robot->getFromDef("UnitreeH1");
    SoleLeft = robot->getFromDef("LeftFootSole");
    SoleRight = robot->getFromDef("RightFootSole");
    ArmHandLeft = robot->getFromDef("LeftArmSole");
    ArmHandRight = robot->getFromDef("RightArmSole");

    // Enable sensors
    for (int i = 0; i < NJ; i++) {
        legMotor[i]->enableTorqueFeedback(TIME_STEP);
        legSensor[i]->enable(TIME_STEP);
    }
    imu->enable(TIME_STEP);
    accelerometer->enable(TIME_STEP);

    // Derivative
    dRpy.resize(3);
    for (int i = 0; i < 3; i++) {
        dRpy.at(i).init(SAMPLE_TIME, 1e-3, 0.);
    }
    dJnt.resize(NJ);
    for (int i = 0; i < NJ; i++) {
        dJnt.at(i).init(SAMPLE_TIME, 1e-3, 0.);
    }

}

void WebotsRobot::deleteRobot()
{
    // delete robot;
}


bool WebotsRobot::readData(double simTime, webotsState & robotStateSim)
{
    // Motor pos
    robotStateSim.jointPosAct = getMotorPos();
    
    // Motor vel
    if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6){
        for (int i = 0; i < NJ; i++) {
            dJnt.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.jointPosAct(i));
        }
    }
    for (int i = 0; i < NJ; i++) {
        robotStateSim.jointVelAct(i) = dJnt.at(i).mSig(robotStateSim.jointPosAct(i));
    }

    // Motor torque
    robotStateSim.jointTorAct = getMotorTau();

    // IMU Data of RPY and dRPY; 
    const double* rotmArray = Pelvis->getOrientation();
    Eigen::Matrix3d rotm;
    rotm << rotmArray[0], rotmArray[1], rotmArray[2],
            rotmArray[3], rotmArray[4], rotmArray[5],
            rotmArray[6], rotmArray[7], rotmArray[8];
    robotStateSim.pelvisRpyAct = rotm2Rpy(rotm);
  
    if (simTime > SAMPLE_TIME - 1e-6 && simTime < SAMPLE_TIME + 1e-6){
        for (int i = 0; i < 3; i++) {
            dRpy.at(i).init(SAMPLE_TIME, 1e-3, robotStateSim.pelvisRpyAct(i));
        }
    }
    for (int i = 0; i < 3; i++) {
        robotStateSim.pelvisDRpyAct(i) = dRpy.at(i).mSig(robotStateSim.pelvisRpyAct(i));
    }
    //data summary//
    robotStateSim.imuAct << robotStateSim.pelvisRpyAct, robotStateSim.pelvisDRpyAct;

    // Pelvis's ddXYZ;
    robotStateSim.pelvisDDXyzAct = getPelvisAcc();

    // Pelvis's XYZ & dXYZ;
    const double* xyzArray1 = Pelvis->getPosition();
    const double* xyzArray2 = Pelvis->getVelocity();
    robotStateSim.pelvisXyzDXyzAct << xyzArray1[0], xyzArray1[1], xyzArray1[2], xyzArray2[0], xyzArray2[1], xyzArray2[2];

    //External Force
    robotStateSim.footGrfAct = getFootForce2D();

    //Foot xyz and rpy @Danny240516
    const double* a1 = SoleLeft->getPosition();
    const double* a2Array = SoleLeft->getOrientation();
    Eigen::Matrix3d a2;
    a2 << a2Array[0], a2Array[1], a2Array[2],
        a2Array[3], a2Array[4], a2Array[5],
        a2Array[6], a2Array[7], a2Array[8];

    const double* a3 = SoleRight->getPosition();
    const double* a4Array = SoleRight->getOrientation();
    Eigen::Matrix3d a4;
    a4 << a4Array[0], a4Array[1], a4Array[2],
        a4Array[3], a4Array[4], a4Array[5],
        a4Array[6], a4Array[7], a4Array[8];

    robotStateSim.LeftSoleXyzRpyAct.head(3) << a1[0], a1[1], a1[2];
    robotStateSim.LeftSoleXyzRpyAct.tail(3) = rotm2Rpy(a2);

    robotStateSim.RightSoleXyzRpyAct.head(3) << a3[0], a3[1], a3[2];
    robotStateSim.RightSoleXyzRpyAct.tail(3) = rotm2Rpy(a4);

    //Arm xyz and rpy @Danny240516
    const double* b1 = ArmHandLeft->getPosition();
    const double* b2Array = ArmHandLeft->getOrientation();
    Eigen::Matrix3d b2;
    b2 << b2Array[0], b2Array[1], b2Array[2],
        b2Array[3], b2Array[4], b2Array[5],
        b2Array[6], b2Array[7], b2Array[8];

    const double* b3 = ArmHandRight->getPosition();
    const double* b4Array = ArmHandRight->getOrientation();
    Eigen::Matrix3d b4;
    b4 << b4Array[0], b4Array[1], b4Array[2],
        b4Array[3], b4Array[4], b4Array[5],
        b4Array[6], b4Array[7], b4Array[8];

    robotStateSim.LeftArmHandXyzRpyAct.head(3) << b1[0], b1[1], b1[2];
    robotStateSim.LeftArmHandXyzRpyAct.tail(3) = rotm2Rpy(b2);

    robotStateSim.RightArmHandXyzRpyAct.head(3) << b3[0], b3[1], b3[2];
    robotStateSim.RightArmHandXyzRpyAct.tail(3) = rotm2Rpy(b4);

    return true;
}

bool WebotsRobot::setMotorPos(const Eigen::VectorXd& jointPosTar) {
    for (int i = 0; i < NJ; i++) {
        legMotor[i]->setPosition(jointPosTar(i, 0));
    }
    return true;
}

bool WebotsRobot::setMotorPosTau(const Eigen::VectorXd& jointTauPosMixed) {
    for (int i = 0; i < 11; i++) {
        legMotor[i]->setPosition(jointTauPosMixed(i, 0));}

    for (int i = 0; i < 8; i++) { 
        legMotor[11+i]->setTorque(jointTauPosMixed(11+i, 0));}

    return true;
}

bool WebotsRobot::setMotorPosTau2(const Eigen::VectorXd& jointTauPosMixed) {
    for (int i = 0; i < 10; i++) {
        legMotor[i]->setTorque(jointTauPosMixed(i, 0));}

    for (int i = 0; i < 9; i++) { 
        legMotor[10+i]->setPosition(jointTauPosMixed(10+i, 0));}
        
    return true;
}

bool WebotsRobot::setMotorPosTau3(const Eigen::VectorXd& jointTauPosMixed) {
    for (int i = 0; i < 10; i++) {
        legMotor[i]->setPosition(jointTauPosMixed(i, 0));}

    for (int i = 0; i < 9; i++) { 
        legMotor[10+i]->setTorque(jointTauPosMixed(10+i, 0));}
        
    return true;
}

bool WebotsRobot::setMotorPosTau4(const Eigen::VectorXd& jointTauPosMixed) {
    for (int i = 0; i < 10; i++) {
        legMotor[i]->setTorque(jointTauPosMixed(i, 0));}

        legMotor[10]->setPosition(0.);

    for (int i = 0; i < 8; i++) { 
        legMotor[11+i]->setTorque(jointTauPosMixed(11+i, 0));}
        
    return true;
}

bool WebotsRobot::setMotorTau(const Eigen::VectorXd& jointTauTar) {
    for (int i = 0; i < NJ; i++) {
        legMotor[i]->setTorque(jointTauTar(i, 0));
    }
    return true;
}

Eigen::VectorXd WebotsRobot::getMotorPos() {
    Eigen::VectorXd Q = Eigen::VectorXd::Zero(NJ);
    for (int i = 0; i < NJ; i++) {
        Q(i, 0) = legSensor[i]->getValue();
    }
    return Q;
}

Eigen::VectorXd WebotsRobot::getMotorTau() {
    Eigen::VectorXd Tau = Eigen::VectorXd::Zero(NJ);
    for (int i = 0; i < NJ; i++) {
        Tau(i, 0) = legMotor[i]->getTorqueFeedback();
    }
    return Tau;
}

Eigen::Vector3d WebotsRobot::getPelvisAcc() {
    const double* data = accelerometer->getValues();
    Eigen::Vector3d acceleration(data[0], data[1], data[2]);
    return acceleration;
}

// The foot force feedback is adjusted from 6d 2 1d, only torque on y_axis @Danny240520
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

Eigen::Vector3d WebotsRobot::rotm2Rpy(const Eigen::Matrix3d & rotm) {
    Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
    rpy(0) = atan2(rotm(2,1), rotm(2,2));
    rpy(1) = atan2(-rotm(2,0), sqrt(rotm(2,1) * rotm(2,1) + rotm(2,2) * rotm(2,2)));
    rpy(2) = atan2(rotm(1,0), rotm(0,0));
    return rpy;
}


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
    // sigOut = b0 * sigIn + b1 * sigInPrev - a1 * sigOutPrev;
    sigOutPrev = sigOut;
    sigInPrev = sigIn;
    return sigOut;
}
