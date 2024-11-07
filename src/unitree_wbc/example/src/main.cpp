#include <ros/ros.h>
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
    ros::Publisher jnt_pos_pub;
    ros::Publisher sim_info_pub;
    ros::Publisher sim_time_pub;
    ros::Publisher jnt_toq_pub;
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
    pubs.jnt_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("jnt_pos", 20);
    pubs.jnt_toq_pub = nh.advertise<std_msgs::Float64MultiArray>("jnt_toq", 20);
    pubs.sim_info_pub = nh.advertise<std_msgs::Float64MultiArray>("sim_info", 20);
    pubs.sim_time_pub = nh.advertise<std_msgs::Time>("sim_time", 20);
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
    

    std_msgs::Time sim_time_msg;
    std_msgs::Float64MultiArray jnt_pos_msg;
    std_msgs::Float64MultiArray sim_info_msg;
    std_msgs::Float64MultiArray jnt_toq_msg;
    jnt_pos_msg.data.resize(NJ);
    jnt_toq_msg.data.resize(NJ);
    sim_info_msg.data.resize(1);

    while (bipedWebots.robot->step(TIME_STEP) != -1)
    {
        simTime = bipedWebots.robot->getTime();
        bipedWebots.readData(simTime, robotStateSim);
        ros::Rate loop_rate(1000);
        sim_time_msg.data = ros::Time::now();

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
                // jnt_pos_msg.data[i] = 10;
                // jnt_pos_msg.data[i] = jointPosAcc[i];
                // jnt_pos_msg.data[i] = jointToqCmd[i];
                jnt_pos_msg.data[i] = robotStateSim.jointPosAct[i];
                jnt_toq_msg.data[i] = jointToqCmd[i];
            }
            sim_info_msg.data[0] = simTime;
            pubs.sim_time_pub.publish(sim_time_msg);
            pubs.sim_info_pub.publish(sim_info_msg);
            pubs.jnt_pos_pub.publish(jnt_pos_msg);
            pubs.jnt_toq_pub.publish(jnt_toq_msg);
            ros::spinOnce();
            loop_rate.sleep();            
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
