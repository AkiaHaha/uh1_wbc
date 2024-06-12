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

bool runWebots(ros::Publisher& joint_pos_pub, ros::Publisher& sim_info_pub);



// the arguments of the main() can be specified by the "controllerArgs" field of the Robot node.
int main(int argc, char **argv) {
    ros::init(argc, argv, "webots_controller");
    ros::NodeHandle nh;
    ros::Publisher joint_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_positions", 19);
    ros::Publisher sim_info_pub = nh.advertise<std_msgs::Float64MultiArray>("sim_info", 1);

    runWebots(joint_pos_pub, sim_info_pub);
    return 0;
}


bool runWebots(ros::Publisher& joint_pos_pub, ros::Publisher& sim_info_pub){
    // timing
    int simCnt = 0;
    double simTime = 0;
    const int goStandCnt = 3;
    const double goStandTime = goStandCnt * SAMPLE_TIME;	//second机器人曲膝的时间
    const int simStopCnt  = goStandCnt + 1000000;
    const double simStopTime = simStopCnt * SAMPLE_TIME;    //second机器人停止仿真的时间
    
    // webots
    WebotsRobot bipedWebots;
    bipedWebots.initWebots();
    webotState robotStateSim;
 
    // controller
    BipedController bipedCtrl;

    // vector //
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
    std_msgs::Float64MultiArray sim_info_msg;
    joint_pos_msg.data.resize(19);
    sim_info_msg.data.resize(10);

    // simulation loopsim_information_msg
    std::cout << "Program started dd21." << std::endl << endl;
    while (bipedWebots.robot->step(TIME_STEP) != -1)
    {
        // read data from Webots
        simTime = bipedWebots.robot->getTime();
        bipedWebots.readData(simTime, robotStateSim);
        // akiaPrint1(robotStateSim.jointPosAct, 19, 5, 5, 5, 1, 4, 4);

        // control robot
        if (simCnt < goStandCnt){
            standPosCmd << 0, 0, -0.3, 0.8, -0.46, //left leg--RYP
                           0, 0, -0.3, 0.8, -0.46,//right leg
                           0,                     //torso
                           0, 0, 0, -1.6 ,  //left arm--PRY
                           0, 0, 0, -1.6 ; //right arm

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

            // jointTorCmd = Eigen::VectorXd::Zero(19);

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
                // joint_pos_msg.data[i] = 10;
                // joint_pos_msg.data[i] = jointPosAcc[i];
                joint_pos_msg.data[i] = jointTorCmd[i];
                // joint_pos_msg.data[i] = robotStateSim.jointPosAct[i];
            }
            sim_info_msg.data[0] = simTime;

            sim_info_pub.publish(sim_info_msg);
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
