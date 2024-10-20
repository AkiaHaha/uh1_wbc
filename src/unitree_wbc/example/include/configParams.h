#include "json.hpp"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using json = nlohmann::json;


struct ConfigParams {
    // PArams for task control
    double kpTorsoR, kpTorsoP, kpTorsoY, kdTorsoR, kdTorsoP, kdTorsoY;
    double kpTorsoX, kpTorsoYY, kpTorsoZ, kdTorsoX, kdTorsoYY, kdTorsoZ;
    double kpUpTorsoR, kpUpTorsoP, kpUpTorsoY, kdUpTorsoR, kdUpTorsoP, kdUpTorsoY;
    double kpUpTorsoX, kpUpTorsoYY, kpUpTorsoZ, kdUpTorsoX, kdUpTorsoYY, kdUpTorsoZ;
    double kpFootR, kpFootP, kpFootY, kdFootR, kdFootP, kdFootY;
    double weightGVL, weightGVL10, weightGVLKnee, weightGVLAnkle;
    double weightFootForceR, weightFootForceP, weightFootForceYaw;
    double weightFootForceX, weightFootForceY, weightFootForceZ;
    double weightArmForceR, weightArmForceP, weightArmForceYaw;
    double weightArmForceX, weightArmForceY, weightArmForceZ;
    double weightFBD, weightFootPos, weightFootYaw, weightArmPos, weightArmRPY, weightArmZ;
    double weightTorsoPos, weightTorsoRPY;

    // Params for motion plan
    double height, pitchApt, pitchFrq;

    // PD gains
    std::vector<double> kpTorsoXyz{0., 0., 0.};
    std::vector<double> kdTorsoXyz{0., 0., 0.};
    std::vector<double> kpTorsoRpy = {0., 0., 0.};
    std::vector<double> kdTorsoRpy = {0., 0., 0.};
    std::vector<double> kpUpTorsoXyz{0., 0., 0.};
    std::vector<double> kdUpTorsoXyz{0., 0., 0.};
    std::vector<double> kpUpTorsoRpy = {0., 0., 0.};
    std::vector<double> kdUpTorsoRpy = {0., 0., 0.};
    std::vector<double> kpFootXyz{0., 0., 0.};
    std::vector<double> kdFootXyz{0., 0., 0.};
    std::vector<double> kpFootRpy{0., 0., 0.};
    std::vector<double> kdFootRpy{0., 0., 0.};
    std::vector<double> kpArmXyz{0., 0., 0.};
    std::vector<double> kdArmXyz{0., 0., 0.};
    std::vector<double> kpArmRpy{0., 0., 0.};
    std::vector<double> kdArmRpy{0., 0., 0.};

    // Weight value of tasks and constraints
    Eigen::Vector3d weightTorsoPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightTorsoOrientation = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightUpTorsoPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightUpTorsoOrientation = Eigen::Vector3d::Zero();
    Eigen::VectorXd weightFootArmPosition = Eigen::VectorXd::Zero(NFCC4);// @Daniel240521
    Eigen::VectorXd weightFootForce = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd weightFootForceChange = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd weightFootArmForce = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd weightFootArmForceChange = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd weightFloatBaseDynamic = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd weightGlobalVelLimitation = Eigen::VectorXd::Constant(19,weightGVL);


    ConfigParams() {
        // Constructor to initialize from JSON file
        std::ifstream inputFile("/home/ukia/gitRepo/uh1_wbc/src/unitree_wbc/config/controller.json");
        json jsonData;
        inputFile >> jsonData;

        kpTorsoR = jsonData["kpTorsoR"];
        kpTorsoP = jsonData["kpTorsoP"];
        kpTorsoY = jsonData["kpTorsoY"];
        kdTorsoR = jsonData["kdTorsoR"];
        kdTorsoP = jsonData["kdTorsoP"];
        kdTorsoY = jsonData["kdTorsoY"];

        kpTorsoX = jsonData["kpTorsoX"];
        kpTorsoYY = jsonData["kpTorsoYY"];
        kpTorsoZ = jsonData["kpTorsoZ"];
        kdTorsoX = jsonData["kdTorsoX"];
        kdTorsoYY = jsonData["kdTorsoYY"];
        kdTorsoZ = jsonData["kdTorsoZ"];

        kpUpTorsoR = jsonData["kpUpTorsoR"];
        kpUpTorsoP = jsonData["kpUpTorsoP"];
        kpUpTorsoY = jsonData["kpUpTorsoY"];
        kdUpTorsoR = jsonData["kdUpTorsoR"];
        kdUpTorsoP = jsonData["kdUpTorsoP"];
        kdUpTorsoY = jsonData["kdUpTorsoY"];

        kpUpTorsoX = jsonData["kpUpTorsoX"];
        kpUpTorsoYY = jsonData["kpUpTorsoYY"];
        kpUpTorsoZ = jsonData["kpUpTorsoZ"];
        kdUpTorsoX = jsonData["kdUpTorsoX"];
        kdUpTorsoYY = jsonData["kdUpTorsoYY"];
        kdUpTorsoZ = jsonData["kdUpTorsoZ"];

        kpFootR = jsonData["kpFootR"];
        kpFootP = jsonData["kpFootP"];
        kpFootY = jsonData["kpFootY"];
        kdFootR = jsonData["kdFootR"];
        kdFootP = jsonData["kdFootP"];
        kdFootY = jsonData["kdFootY"];

        weightGVL = jsonData["weightGVL"];
        weightGVL10 = jsonData["weightGVL10"];
        weightGVLKnee = jsonData["weightGVLKnee"];
        weightGVLAnkle = jsonData["weightGVLAnkle"];

        weightFootForceR = jsonData["weightFootForceR"];
        weightFootForceP = jsonData["weightFootForceP"];
        weightFootForceYaw = jsonData["weightFootForceYaw"];
        weightFootForceX = jsonData["weightFootForceX"];
        weightFootForceY = jsonData["weightFootForceY"];
        weightFootForceZ = jsonData["weightFootForceZ"];

        weightArmForceR = jsonData["weightArmForceR"];
        weightArmForceP = jsonData["weightArmForceP"];
        weightArmForceYaw = jsonData["weightArmForceYaw"];
        weightArmForceX = jsonData["weightArmForceX"];
        weightArmForceY = jsonData["weightArmForceY"];
        weightArmForceZ = jsonData["weightArmForceZ"];

        weightFBD = jsonData["weightFBD"];
        weightFootPos = jsonData["weightFootPos"];
        weightFootYaw = jsonData["weightFootYaw"];
        weightArmPos = jsonData["weightArmPos"];
        weightArmRPY = jsonData["weightArmRPY"];
        weightArmZ = jsonData["weightArmZ"];
        weightTorsoPos = jsonData["weightTorsoPos"];
        weightTorsoRPY = jsonData["weightTorsoRPY"];

        // Set PD gains
        kpTorsoRpy = {kpTorsoR, kpTorsoP, kpTorsoY};
        kdTorsoRpy = {kdTorsoR, kdTorsoP, kdTorsoY};
        kpTorsoXyz = {kpTorsoX, kpTorsoYY, kpTorsoZ};
        kdTorsoXyz = {kdTorsoX, kdTorsoYY, kdTorsoZ};

        kpUpTorsoRpy = {kpUpTorsoR, kpUpTorsoP, kpUpTorsoY};
        kdUpTorsoRpy = {kdUpTorsoR, kdUpTorsoP, kdUpTorsoY};
        kpUpTorsoXyz = {kpUpTorsoX, kpUpTorsoYY, kpUpTorsoZ};
        kdUpTorsoXyz = {kdUpTorsoX, kdUpTorsoYY, kdUpTorsoZ};


        kpFootXyz = {800., 800., 800.};
        kdFootXyz = {15., 15., 15.};
        kpFootRpy = {kpFootR, kpFootP, kpFootY};
        kdFootRpy = {kdFootR, kdFootP, kdFootY};

        kpArmXyz = fillVector2(1200, 3);
        kdArmXyz = fillVector2(120, 3);
        kpArmRpy = fillVector2(1000, 3);
        kdArmRpy = fillVector2(100, 3);

        // Set weights 
        weightTorsoPosition = fillVector3(600000);                                           
        weightTorsoOrientation = fillVector3(600000); 
        weightUpTorsoPosition = fillVector3(300000);                                        
        weightUpTorsoOrientation << 300000, 300000, 3000000;

        weightFootArmPosition = fillVector(weightFootPos, weightArmPos);   
        weightFootArmPosition(2) = weightFootYaw;
        weightFootArmPosition(8) = weightFootYaw;
        weightFootArmPosition(17) = weightArmZ;
        weightFootArmPosition(23) = weightArmZ;
        weightFootArmPosition.segment(12,3) = Eigen::VectorXd::Constant(3,weightArmRPY);
        weightFootArmPosition.segment(18,3) = Eigen::VectorXd::Constant(3,weightArmRPY);
        
        weightFootArmForce <<   weightFootForceR, weightFootForceP, weightFootForceYaw, 
                                weightFootForceX, weightFootForceY, weightFootForceZ, 
                                weightFootForceR, weightFootForceP, weightFootForceYaw, 
                                weightFootForceX, weightFootForceY, weightFootForceZ,
                                weightArmForceR, weightArmForceP, weightArmForceYaw, 
                                weightArmForceX, weightArmForceY, weightArmForceZ, 
                                weightArmForceR, weightArmForceP, weightArmForceYaw, 
                                weightArmForceX, weightArmForceY, weightArmForceZ;
                                // 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 
                                // 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;

        weightFootArmForceChange << weightFootForceR, weightFootForceP, weightFootForceYaw, 
                                    weightFootForceX, weightFootForceY, weightFootForceZ, 
                                    weightFootForceR, weightFootForceP, weightFootForceYaw, 
                                    weightFootForceX, weightFootForceY, weightFootForceZ,
                                    weightArmForceR, weightArmForceP, weightArmForceYaw, 
                                    weightArmForceX, weightArmForceY, weightArmForceZ, 
                                    weightArmForceR, weightArmForceP, weightArmForceYaw, 
                                    weightArmForceX, weightArmForceY, weightArmForceZ;

        weightFloatBaseDynamic = Eigen::VectorXd::Constant(6,weightFBD);

        weightGlobalVelLimitation(10) = weightGVL10;
        weightGlobalVelLimitation(3) = weightGVLKnee;
        weightGlobalVelLimitation(8) = weightGVLKnee;
        weightGlobalVelLimitation(4) = weightGVLAnkle;
        weightGlobalVelLimitation(9) = weightGVLAnkle;

        height = jsonData["height"];
        pitchApt = jsonData["pitchApt"];
        pitchFrq = jsonData["pitchFrq"];
    }

};