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

#include "webotsInterface.h"
#include "bipedController.h"
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
    const int goStandCnt = 1;
    const double goStandTime = goStandCnt * SAMPLE_TIME;	//second机器人曲膝的时间
    const int simStopCnt  = goStandCnt + 31000;
    const double simStopTime = simStopCnt * SAMPLE_TIME;    //second机器人停止仿真的时间
    // webots

    WebotsRobot bipedWebots;
   
    bipedWebots.initWebots();

    webotState robotStateSim;
 
    // controller
    BipedController bipedCtrl;

    Eigen::VectorXd standPosCmd = Eigen::VectorXd::Zero(19);
    Eigen::VectorXd jointTorCmd = Eigen::VectorXd::Zero(19);

    // simulation loop
    std::cout << "Program started." << std::endl << endl;

    while (bipedWebots.robot->step(TIME_STEP) != -1)
    {
        // read data from Webots
        simTime = bipedWebots.robot->getTime();
       
        bipedWebots.readData(simTime, robotStateSim);

        // control robot
        if (simCnt < goStandCnt){
            // go to desired position
            standPosCmd << 0, 0, -0.3, 0.8, -0.9, //left leg--RYP
                           0, 0, -0.3, 0.8, -0.46,//right leg
                           0,                     //torso
                           0, 0, 0, 0,  //left arm--PRY
                           0, 0, 0, 0; //right arm
            bipedWebots.setMotorPos(standPosCmd);//设置初始位置曲腿
            // std::cout << "simCnt" <<simCnt<< std::endl;
            // std::cout << "goStandCnt" <<goStandCnt<< std::endl;

            if (simCnt % 10 == 0)
            {
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Daniel: State Data test at Stance Stage//
std::cout << "State Data test at Stance Stage <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
                cout << "--------------------------------------------" << endl;
                cout << "imu data test" << endl;
    // Print rpy: the first 3 elements
    std::cout << "rpy: \t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[0] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[1] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[2] << std::endl;

    // Print rpyDot: the last 3 elements
    std::cout << "rpyDot: \t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[9] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[10] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[11] << std::endl;

    // Print xyz: the 4th to 6th elements
    std::cout << "xyz: \t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[3] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[4] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[5] << std::endl;

    // Print xyzDot: the 7th to 9th elements
    std::cout << "xyzDot: \t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[6] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[7] << "\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.imu9DAct[8] << std::endl;

                cout << "--------------------------------------------" << endl;
                cout << "jointTorAct data test" << endl;
    std::cout << "Joint Torque Data" << std::endl;

    // Print Left Leg: first 5 elements
    std::cout << "Left Leg:\t";
    for (int i = 0; i < 5; ++i) {
        std::cout << std::fixed << std::setprecision(9) << robotStateSim.jointTorAct[i];
        if (i < 4) std::cout << "\t";
    }
    std::cout << std::endl;

    // Print Right Leg: next 5 elements
    std::cout << "Right Leg:\t";
    for (int i = 5; i < 10; ++i) {
        std::cout << std::fixed << std::setprecision(9) << robotStateSim.jointTorAct[i];
        if (i < 9) std::cout << "\t";
    }
    std::cout << std::endl;

    // Print Waist: 11th element
    std::cout << "Waist:\t" 
              << std::fixed << std::setprecision(9) << robotStateSim.jointTorAct[10] << std::endl;

    // Print Left Shoulder: next 4 elements
    std::cout << "Left Shoulder:\t";
    for (int i = 11; i < 15; ++i) {
        std::cout << std::fixed << std::setprecision(9) << robotStateSim.jointTorAct[i];
        if (i < 14) std::cout << "\t";
    }
    std::cout << std::endl;

    // Print Right Shoulder: last 4 elements
    std::cout << "Right Shoulder:\t";
    for (int i = 15; i < 19; ++i) {
        std::cout << std::fixed << std::setprecision(9) << robotStateSim.jointTorAct[i];
        if (i < 18) std::cout << "\t";
    }
    std::cout << std::endl;

                cout << "--------------------------------------------" << endl;
                cout << "waistRpyAct data test" << endl
                     << robotStateSim.waistRpyAct <<endl;                     

                cout << "--------------------------------------------" << endl;
                cout << "waistXyzAccAct data test" << endl
                     << robotStateSim.waistXyzAccAct <<endl;


                cout << "--------------------------------------------" << endl;
                cout << "footGrfAct data test" << endl
                     << robotStateSim.footGrfAct <<endl;
std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl << endl;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>24.5.24//
            }


        }else if (simCnt < simStopCnt){
            // your control loops
            bipedCtrl.update(simTime-goStandTime, robotStateSim.imu9DAct,
                             robotStateSim.jointPosAct, robotStateSim.jointVelAct, robotStateSim.footGrfAct);
            std::cout << "222222" << std::endl;
            bipedCtrl.getValueTauOpt(jointTorCmd);

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Daniel: State Data test at Stance Stage//
std::cout << "Torque Cmd Test at simcnt of " << simCnt << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
        int index = 0;
            std::vector<int> rowLengths = {5, 5, 1, 4, 4};

            for (size_t i = 0; i < rowLengths.size(); ++i) {
                std::cout << "row " << i + 1 << ": ";
                for (int j = 0; j < rowLengths[i]; ++j) {
                    std::cout << jointTorCmd[index];
                    if (j < rowLengths[i] - 1) {
                        std::cout << ", ";
                    }
                    ++index;
                }
                std::cout << std::endl;
            }
std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl << endl;
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>24.5.24//
            bipedWebots.setMotorTau(jointTorCmd);
        }
        else{

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