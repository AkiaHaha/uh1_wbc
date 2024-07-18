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
    ros::Publisher joint_pos_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_positions", NJ);
    ros::Publisher sim_info_pub = nh.advertise<std_msgs::Float64MultiArray>("sim_info", 1);

    runWebots(joint_pos_pub, sim_info_pub);
    return 0;
}


bool runWebots(ros::Publisher& joint_pos_pub, ros::Publisher& sim_info_pub){
    // timing
    int simCnt = 0;
    double simTime = 0;
    const int goStandCnt = 1;
    const double goStandTime = goStandCnt * SAMPLE_TIME;
    const int simStopCnt  = goStandCnt + 1000000;
    const double simStopTime = simStopCnt * SAMPLE_TIME;
    
    // webots
    WebotsRobot bipedWebots;
    bipedWebots.initWebots();
    webotState robotStateSim;
 
    // controller
    BipedController bipedCtrl;

    // vector //
    Eigen::VectorXd standPosCmd = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointTorCmd = Eigen::VectorXd::Zero(NJ);

    Eigen::VectorXd jointPosInteg = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointPosAcc = Eigen::VectorXd::Zero(NJ);
    Eigen::VectorXd jointPosAtStartCtrl = Eigen::VectorXd::Zero(NJ);
    
    // integrator
    Integrator integrator;
    // double flagStartCtrl{}; 

    // ros init
    std_msgs::Float64MultiArray joint_pos_msg;
    std_msgs::Float64MultiArray sim_info_msg;
    joint_pos_msg.data.resize(NJ);
    sim_info_msg.data.resize(10);

    // simulation loopsim_information_msg
    std::cout << "Program started dd81." << std::endl << endl;
    while (bipedWebots.robot->step(TIME_STEP) != -1)
    {
        // read data from Webots //
        simTime = bipedWebots.robot->getTime();
        bipedWebots.readData(simTime, robotStateSim);

        // control robot //
        if (simCnt < goStandCnt){
            standPosCmd << 0, 0, -0.3, 0.8, -0.46, //left leg--RYP
                           0, 0, -0.3, 0.8, -0.46,//right leg
                           0,                     //torso
                           0, 0, 0, 0,  //left arm--PRY
                           0, 0, 0, 0; //right arm

            // standPosCmd << 0, 0, 0, 0, 0, //left leg--RYP
            //                0, 0, 0, 0, 0,//right leg
            //                0,           //torso
            //                0, 0, 0, 0,  //left arm--PRY
            //                0, 0, 0, 0; //right arm
            bipedWebots.setMotorPos(standPosCmd);//设置初始位置曲腿

        }else if (simCnt < simStopCnt){
            //dynamic control get torque
            bipedCtrl.update(simTime-goStandTime, robotStateSim.imu9DAct,
                             robotStateSim.jointPosAct, robotStateSim.jointVelAct, robotStateSim.footGrfAct,
                             robotStateSim.LeftSoleXyzRpyAct, robotStateSim.RightSoleXyzRpyAct,
                             robotStateSim.LeftArmHandXyzRpyAct, robotStateSim.RightArmHandXyzRpyAct);
            bipedCtrl.getValueTauOpt(jointTorCmd);

/*          
            //Integrate acc for pos
            if (flagStartCtrl == 0){
                jointPosAtStartCtrl = robotStateSim.jointPosAct;
                flagStartCtrl = 1;
            }
            bipedCtrl.getValueQdd(jointPosAcc);
            for (size_t i = 0; i < NJ; i++){
                 jointPosInteg[i] = integrator.Integrate(jointPosAcc[i]);
            }
            jointPosInteg += jointPosAtStartCtrl;

            //set data for ROS topic
            for (size_t i = 0; i < NJ; i++){
                // joint_pos_msg.data[i] = 10;
                // joint_pos_msg.data[i] = jointPosAcc[i];
                // joint_pos_msg.data[i] = jointTorCmd[i];
                joint_pos_msg.data[i] = robotStateSim.jointPosAct[i];
            }
            sim_info_msg.data[0] = simTime;
            sim_info_pub.publish(sim_info_msg);
            joint_pos_pub.publish(joint_pos_msg);                            */

//-----------------------------------------------------------------
//universal set toq / pos
            // jointTorCmd(16) = 0;
            bipedWebots.setMotorTau(jointTorCmd);
            
            // bipedWebots.setMotorPos(jointPosInteg);


//pos-leg&torso 
            // standPosCmd.segment(11,8) = jointTorCmd.segment(11,8);
            // bipedWebots.setMotorPosTau(standPosCmd);
            // akiaPrint1(standPosCmd, 19, 5, 5, 5, 1, 4, 4);
//pos-leg
            // standPosCmd.segment(10,9) = jointTorCmd.segment(10,9);
            // bipedWebots.setMotorPosTau3(standPosCmd);
            // akiaPrint1(standPosCmd, 19, 5, 5, 5, 1, 4, 4);

//pos-shoulder
            // standPosCmd.head(10) = jointTorCmd.head(10);
            // bipedWebots.setMotorPosTau2(standPosCmd);  

//only-pos-torso
            // bipedWebots.setMotorPosTau4(jointTorCmd);

//-----------------------------------------------------------------

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
