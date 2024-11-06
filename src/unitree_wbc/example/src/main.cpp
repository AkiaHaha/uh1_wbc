#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

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

bool runWebots(ros::Publisher& joint_pos_pub, ros::Publisher& sim_info_pub);

void signalHandler(int signum) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    exit(signum);
}

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    ros::init(argc, argv, "webots_controller");
    ros::NodeHandle nh;
    ros::Publisher joint_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_positions", NJ);
    ros::Publisher sim_info_pub = nh.advertise<std_msgs::Float64MultiArray>("sim_info", 1);
    runWebots(joint_pos_pub, sim_info_pub);
    return 0;
}

bool runWebots(ros::Publisher& joint_pos_pub, ros::Publisher& sim_info_pub){
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
    

    std_msgs::Float64MultiArray joint_pos_msg;
    std_msgs::Float64MultiArray sim_info_msg;
    joint_pos_msg.data.resize(NJ);
    sim_info_msg.data.resize(10);

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
        simCnt++;

        //=========================================================//
        // Set data for ROS topic
            for (size_t i = 0; i < NJ; i++){
                // joint_pos_msg.data[i] = 10;
                // joint_pos_msg.data[i] = jointPosAcc[i];
                // joint_pos_msg.data[i] = jointToqCmd[i];
                joint_pos_msg.data[i] = robotStateSim.jointPosAct[i];
                joint_pos_msg.data[i] = standPosCmd[i];
                
            }
            sim_info_msg.data[0] = simTime;
            sim_info_pub.publish(sim_info_msg);
            joint_pos_pub.publish(joint_pos_msg);                
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
