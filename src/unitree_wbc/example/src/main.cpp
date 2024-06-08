/**
 *	This file is part of TAICHI.
 *
 *	TAICHI -- Task Arrangement In Control HIerarchy.
 *	Copyright (C) 2015-2021 Beijing Research Institute of UBTECH Robotics.
 *	All rights reserved.
 *
 *	Licensed under the Apache License 2.0. See LICENSE for more details.
 */

/**
 * @file main.cpp
 * @brief the main function file to run controller in Webots.
 * @author Jiajun Wang
 * @date 2021
 * @version
 */
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept> 

#include "webotsInterface.h"
#include "bipedController.h"
#include "operation.h"

using namespace std;

bool runWebots(ros::Publisher& joint_pos_pub);



// the arguments of the main() can be specified by the "controllerArgs" field of the Robot node.
int main(int argc, char **argv) {
    ros::init(argc, argv, "webots_controller");
    ros::NodeHandle nh;
    ros::Publisher joint_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_positions", 19);

    runWebots(joint_pos_pub);
    return 0;
}


bool runWebots(ros::Publisher& joint_pos_pub){
    // timing
    int simCnt = 0;
    double simTime = 0;
    const int goStandCnt = 3;
    const double goStandTime = goStandCnt * SAMPLE_TIME;	//second机器人曲膝的时间
    const int simStopCnt  = goStandCnt + 100000;
    const double simStopTime = simStopCnt * SAMPLE_TIME;    //second机器人停止仿真的时间
    
    // webots
    WebotsRobot bipedWebots;
    bipedWebots.initWebots();
    webotState robotStateSim;
 
    // controller
    BipedController bipedCtrl;

    // vector
    Eigen::VectorXd standPosCmd = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd jointTorCmd = Eigen::VectorXd::Zero(19);

    Eigen::VectorXd jointPosInteg = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd jointPosAcc = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd jointPosAtStartCtrl = Eigen::VectorXd::Zero(19);
    
    // integrator
    Integrator integrator;
    double flagStartCtrl{}; 

    // ros init
    std_msgs::Float64MultiArray joint_pos_msg;
    joint_pos_msg.data.resize(19);


    // simulation loop
    std::cout << "Program started ." << std::endl << endl;
    while (bipedWebots.robot->step(TIME_STEP) != -1)
    {
        // read data from Webots
        simTime = bipedWebots.robot->getTime();
        bipedWebots.readData(simTime, robotStateSim);

        // control robot
        if (simCnt < goStandCnt){
            standPosCmd << 0, 0, -0.3, 0.8, -0.46, //left leg--RYP
                           0, 0, -0.3, 0.8, -0.46,//right leg
                           0,                     //torso
                           0, 0, 0, 0.5 ,  //left arm--PRY
                           0, 0, 0, 0.5 ; //right arm

            // standPosCmd << 0, 0, 0, 0, 0, //left leg--RYP
            //                0, 0, 0, 0, 0,//right leg
            //                0,           //torso
            //                0, 0, 0, 0,  //left arm--PRY
            //                0, 0, 0, 0; //right arm
            bipedWebots.setMotorPos(standPosCmd);//设置初始位置曲腿

        }else if (simCnt < simStopCnt){

            bipedCtrl.update(simTime-goStandTime, robotStateSim.imu9DAct,
                             robotStateSim.jointPosAct, robotStateSim.jointVelAct, robotStateSim.footGrfAct,
                             robotStateSim.LeftSoleXyzRpyAct, robotStateSim.RightSoleXyzRpyAct,
                             robotStateSim.LeftArmHandXyzRpyAct, robotStateSim.RightArmHandXyzRpyAct);
            bipedCtrl.getValueTauOpt(jointTorCmd);

            // bipedWebots.setMotorTau(jointTorCmd);
            // if (jointTorCmd(0) > 0){
            //     cout << "*************" << endl;
            //     throw std::runtime_error("Torque command greater than 0");
            // }
            
            if (flagStartCtrl == 0){
                jointPosAtStartCtrl = robotStateSim.jointPosAct;
                flagStartCtrl = 1;
            }

            bipedCtrl.getValueQdd(jointPosAcc);
            for (size_t i = 0; i < 19; i++){
                jointPosInteg[i] = integrator.Integrate(jointPosAcc[i]);
            }
            jointPosInteg += jointPosAtStartCtrl;

            for (size_t i = 0; i < 19; i++){
                joint_pos_msg.data[i] = robotStateSim.jointPosAct[i];
            }
            
            joint_pos_pub.publish(joint_pos_msg);

            bipedWebots.setMotorPos(jointPosInteg);
            // cout << "*** cnt " << simCnt << " ***" << endl;
            // akiaPrint1(robotStateSim.jointPosAct, 19, 5, 5, 5, 1, 4, 4);
            // cout << "------------------------------------" << endl;
            // akiaPrint1(jointTorCmd, 19, 5, 5, 5, 1, 4, 4);
            // cout << endl;
            
        }else{
            // keep current position
            bipedCtrl.getValuePosCurrent(standPosCmd);
            bipedWebots.setMotorPos(standPosCmd);
        }

        // stop simulation
        if (simTime > simStopTime){
            break;
        }
        simCnt++;
    };

    // Free memory
    bipedWebots.deleteRobot();
    std::cout << "Program ended." << std::endl;

    return true;
}
