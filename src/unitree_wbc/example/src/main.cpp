#include <ros/ros.h>
#include <unitree_wbc/MotorData.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Time.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept> 
#include <csignal>
#include <chrono>
#include <thread>
#include <unistd.h>

#include "webotsInterface.h"
#include "robotController.h"
#include "operation.h"

using namespace std;

struct Publishers {
    ros::Publisher motor_data_pub;
};

bool runWebots(Publishers& pubs);

void signalHandler(int signum) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "webots_controller");
    ros::NodeHandle nh;
    Publishers pubs;
    pubs.motor_data_pub = nh.advertise<unitree_wbc::MotorData>("sim/motor_data", 20);
    runWebots(pubs);
    return 0;
}

bool runWebots(Publishers& pubs){
    WebotsRobot bipedWebots;
    webotsState robotStateSim;
    bipedWebots.initWebots();
    Integrator integrator;
    ConfigParams prm;

    int simCnt = 0;
    double simTime = 0;
    const int goStandCnt = prm.standingCnt;
    const double goStandTime = goStandCnt * SAMPLE_TIME;
    const int simStopCnt  = goStandCnt + 1000000;
    const double simStopTime = simStopCnt * SAMPLE_TIME;
 
    RobotController RobotController;
    Eigen::VectorXd standPosCmd = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointToqCmd = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointPosInteg = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointPosAcc = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointPosAtStartCtrl = Eigen::VectorXd::Zero(NJ);
    

    std_msgs::Float64 sim_time_msg_float;
    std_msgs::Float64MultiArray pos_msg;
    std_msgs::Float64MultiArray toq_msg;
    unitree_wbc::MotorData motor_data_msg;
    pos_msg.data.resize(NJ);
    toq_msg.data.resize(NJ);

int publish_rate = 100; // 设置一个发布频率
int publish_counter = 0; 

    while (bipedWebots.robot->step(TIME_STEP) != -1)
    {
        simTime = bipedWebots.robot->getTime();
        bipedWebots.readData(simTime, robotStateSim);

        if (simCnt < goStandCnt){
            standPosCmd << 0, 0, -0.3, 0.8, -0.46, 
                           0, 0, -0.3, 0.8, -0.46,
                           0,                     
                           0, 0, 0, 0,  
                           0, 0, 0, 0; 
            bipedWebots.setMotorPos(standPosCmd);

        }else if (simCnt < simStopCnt){
            RobotController.update(simTime-goStandTime, robotStateSim);
            RobotController.getValueTauOpt(jointToqCmd);
            bipedWebots.setMotorTau(jointToqCmd);
        }else{
            RobotController.getValuePosCurrent(standPosCmd);
            bipedWebots.setMotorPos(standPosCmd);
        }
        if (simTime > simStopTime){
            break;
        }

        //=========================================================//
        // Set data for ROS topic
        if (simCnt % publish_rate == 0) {
            for (size_t i = 0; i < NJ; i++){
                // pos_msg.data[i] = 10;
                // pos_msg.data[i] = jointPosAcc[i];
                // pos_msg.data[i] = jointToqCmd[i];                
                pos_msg.data[i] = robotStateSim.jointPosAct[i];
                toq_msg.data[i] = jointToqCmd[i];
            }
            sim_time_msg_float.data = simTime - goStandTime;
            motor_data_msg.positions = pos_msg;
            motor_data_msg.torques = toq_msg;
            motor_data_msg.time_float = sim_time_msg_float;
            pubs.motor_data_pub.publish(motor_data_msg);
            ros::spinOnce();
        }
        simCnt++;
        publish_counter++;         
        //=========================================================//        
    };        

//==========================================================================================//
//Integrate acc for position
            // if (flagStartCtrl == 0){
            //     jointPosAtStartCtrl = robotStateSim.jointPosAct;
            //     flagStartCtrl = 1;
            // }
            // RobotController.getValueQdd(jointPosAcc);
            // for (size_t i = 0; i < NJ; i++){
            //      jointPosInteg[i] = integrator.Integrate(jointPosAcc[i]);
            // }
            // jointPosInteg += jointPosAtStartCtrl;

//universal set toq / pos
            // bipedWebots.setMotorPos(jointPosInteg);

//pos-leg&pelvis 
            // standPosCmd.segment(11,8) = jointToqCmd.segment(11,8);
            // bipedWebots.setMotorPosTau(standPosCmd);
            // akiaPrint1(standPosCmd, 19, 5, 5, 5, 1, 4, 4);
//pos-leg
            // standPosCmd.segment(10,9) = jointToqCmd.segment(10,9);
            // bipedWebots.setMotorPosTau3(standPosCmd);
            // akiaPrint1(standPosCmd, 19, 5, 5, 5, 1, 4, 4);

//pos-shoulder
            // standPosCmd.head(10) = jointToqCmd.head(10);
            // bipedWebots.setMotorPosTau2(standPosCmd);  

//only-pos-pelvis
            // bipedWebots.setMotorPosTau4(jointToqCmd);

//==========================================================================================//
    bipedWebots.deleteRobot();
    std::cout << "Program ended." << std::endl;
    return true;
}
