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

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept> 

#include "webotsInterface.h"
#include "bipedController.h"
#include "operation.h"

using namespace std;

bool runWebots();

// the arguments of the main() can be specified by the "controllerArgs" field of the Robot node.
int main(int argc, char **argv) {
    runWebots();
    return 0;
}

bool runWebots(){
    // timing
    int simCnt = 0;
    double simTime = 0;
    const int goStandCnt = 100;
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

    // simulation loop
    std::cout << "Program started." << std::endl << endl;
    while (bipedWebots.robot->step(TIME_STEP) != -1)
    {
        // read data from Webots
        simTime = bipedWebots.robot->getTime();
        bipedWebots.readData(simTime, robotStateSim);
        // akiaPrint1(robotStateSim.jointPosAct, 19, 5, 5, 5, 1, 4, 4);

        // control robot
        if (simCnt < goStandCnt){
            //go to desired position
            standPosCmd << 0, 0, -0.3, 0.8, -0.9, //left leg--RYP
                           0, 0, -0.3, 0.8, -0.47,//right leg
                           0,                     //torso
                           0, 0, 0, -1.8,  //left arm--PRY
                           0, 0, 0, -2.5; //right arm

            // standPosCmd << 0, 0, 0, 0, 0, //left leg--RYP
            //                0, 0, 0, 0, 0,//right leg
            //                0,           //torso
            //                0, 0, 0, 0,  //left arm--PRY
            //                0, 0, 0, 0; //right arm
            bipedWebots.setMotorPos(standPosCmd);//设置初始位置曲腿

            // cout << "posCmd at count " << simCnt << " --------------------------------------" << endl
            //     << akiaPrint2(standPosCmd, 19, 5, 5, "LL", 5, "RL", 1, "torso", 4, "LS", 4, "RS") << endl;
            
            // bipedWebots.readData(simTime, robotStateSim);
            // cout << "posAct" << "------------------------" << endl
            //     << akiaPrint2(robotStateSim.jointPosAct, 19, 5, 5, "LL", 5, "RL", 1, "torso", 4, "LS", 4, "RS") << endl<<endl;
            

            if (simCnt % 100 == 0)
            {
                //Daniel: State Data test at Stance Stage<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//
                std::cout << "State Data test at Stance Stage <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl; 
                    cout << "--------------------------------------------" << endl
                     << "imu data test" << endl
                     << akiaPrint2(robotStateSim.imu9DAct, 12, 4, 3, "rpy", 3, "rpyDot", 3, "xyz", 3, "xyzDot");

                    cout << "--------------------------------------------" << endl
                     << "jointPosAct data test" << endl
                     << akiaPrint2(robotStateSim.jointPosAct, 19, 5, 5, "LL", 5, "RL", 1, "torso", 4, "LS", 4, "RS");

                    cout << "--------------------------------------------" << endl
                     << "jointTorAct data test" << endl
                     << akiaPrint2(robotStateSim.jointTorAct, 19, 5, 5, "LL", 5, "RL", 1, "torso", 4, "LS", 4, "RS");

                    cout << "--------------------------------------------" << endl
                     << "waistRpyAct data test" << endl
                     << robotStateSim.waistRpyAct.transpose() <<endl;                     

                    cout << "--------------------------------------------" << endl
                     << "waistXyzAccAct data test" << endl
                     << robotStateSim.waistXyzAccAct.transpose() <<endl;

                    cout << "--------------------------------------------" << endl
                     << "footGrfAct data test" << endl
                     << robotStateSim.footGrfAct.transpose() <<endl;

                std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl << endl;
                //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>24.5.24//
            }
            
            }else if (simCnt < simStopCnt){
            bipedCtrl.update(simTime-goStandTime, robotStateSim.imu9DAct,
                             robotStateSim.jointPosAct, robotStateSim.jointVelAct, robotStateSim.footGrfAct,
                             robotStateSim.LeftSoleXyzRpyAct, robotStateSim.RightSoleXyzRpyAct,
                             robotStateSim.LeftArmHandXyzRpyAct, robotStateSim.RightArmHandXyzRpyAct);
            bipedCtrl.getValueTauOpt(jointTorCmd);

            bipedWebots.setMotorTau(jointTorCmd);
            cout << "Torque Cmd at simcnt of " << simCnt << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl
                << akiaPrint2(jointTorCmd, 19, 5, 5, "LL", 5, "RL", 1, "torso", 4, "LS", 4, "RS" )
                << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl<<endl;
            
            // if (jointTorCmd(0) > 0){
            //     cout << "*************" << endl;
            //     throw std::runtime_error("Torque command greater than 0");
            // }
            
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