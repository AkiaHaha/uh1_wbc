#include "json.hpp"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using json = nlohmann::json;


struct ConfigParams {
    // PArams for task control
    double kpPelvisR, kpPelvisP, kpPelvisY, kdPelvisR, kdPelvisP, kdPelvisY;
    double kpPelvisX, kpPelvisYY, kpPelvisZ, kdPelvisX, kdPelvisYY, kdPelvisZ;
    double kpTrunkR, kpTrunkP, kpTrunkY, kdTrunkR, kdTrunkP, kdTrunkY;
    double kpTrunkX, kpTrunkYY, kpTrunkZ, kdTrunkX, kdTrunkYY, kdTrunkZ;
    double kpFootR, kpFootP, kpFootY, kdFootR, kdFootP, kdFootY;
    double weightGVL, weightGVL10, weightGVLKnee, weightGVLAnkle;
    double weightFootForceR, weightFootForceP, weightFootForceYaw;
    double weightFootForceX, weightFootForceY, weightFootForceZ;
    double weightArmForceR, weightArmForceP, weightArmForceYaw;
    double weightArmForceX, weightArmForceY, weightArmForceZ;
    double weightFBD, weightFootPos, weightFootYaw, weightArmPos, weightArmRPY, weightArmZ;
    double weightPelvisPos, weightPelvisRPY;

    // Params for motion plan
    double height, pitchApt, pitchFrq;

    // PD gains
    std::vector<double> kpPelvisXyz{0., 0., 0.};
    std::vector<double> kdPelvisXyz{0., 0., 0.};
    std::vector<double> kpPelvisRpy = {0., 0., 0.};
    std::vector<double> kdPelvisRpy = {0., 0., 0.};
    std::vector<double> kpTrunkXyz{0., 0., 0.};
    std::vector<double> kdTrunkXyz{0., 0., 0.};
    std::vector<double> kpTrunkRpy = {0., 0., 0.};
    std::vector<double> kdTrunkRpy = {0., 0., 0.};
    std::vector<double> kpFootXyz{0., 0., 0.};
    std::vector<double> kdFootXyz{0., 0., 0.};
    std::vector<double> kpFootRpy{0., 0., 0.};
    std::vector<double> kdFootRpy{0., 0., 0.};
    std::vector<double> kpArmXyz{0., 0., 0.};
    std::vector<double> kdArmXyz{0., 0., 0.};
    std::vector<double> kpArmRpy{0., 0., 0.};
    std::vector<double> kdArmRpy{0., 0., 0.};

    // Weight value of tasks and constraints
    Eigen::Vector3d weightPelvisPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightPelvisOrientation = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightTrunkPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightTrunkOrientation = Eigen::Vector3d::Zero();
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

        kpPelvisR = jsonData["kpPelvisR"];
        kpPelvisP = jsonData["kpPelvisP"];
        kpPelvisY = jsonData["kpPelvisY"];
        kdPelvisR = jsonData["kdPelvisR"];
        kdPelvisP = jsonData["kdPelvisP"];
        kdPelvisY = jsonData["kdPelvisY"];

        kpPelvisX = jsonData["kpPelvisX"];
        kpPelvisYY = jsonData["kpPelvisYY"];
        kpPelvisZ = jsonData["kpPelvisZ"];
        kdPelvisX = jsonData["kdPelvisX"];
        kdPelvisYY = jsonData["kdPelvisYY"];
        kdPelvisZ = jsonData["kdPelvisZ"];

        kpTrunkR = jsonData["kpTrunkR"];
        kpTrunkP = jsonData["kpTrunkP"];
        kpTrunkY = jsonData["kpTrunkY"];
        kdTrunkR = jsonData["kdTrunkR"];
        kdTrunkP = jsonData["kdTrunkP"];
        kdTrunkY = jsonData["kdTrunkY"];

        kpTrunkX = jsonData["kpTrunkX"];
        kpTrunkYY = jsonData["kpTrunkYY"];
        kpTrunkZ = jsonData["kpTrunkZ"];
        kdTrunkX = jsonData["kdTrunkX"];
        kdTrunkYY = jsonData["kdTrunkYY"];
        kdTrunkZ = jsonData["kdTrunkZ"];

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
        weightPelvisPos = jsonData["weightPelvisPos"];
        weightPelvisRPY = jsonData["weightPelvisRPY"];

        // Set PD gains
        kpPelvisRpy = {kpPelvisR, kpPelvisP, kpPelvisY};
        kdPelvisRpy = {kdPelvisR, kdPelvisP, kdPelvisY};
        kpPelvisXyz = {kpPelvisX, kpPelvisYY, kpPelvisZ};
        kdPelvisXyz = {kdPelvisX, kdPelvisYY, kdPelvisZ};

        kpTrunkRpy = {kpTrunkR, kpTrunkP, kpTrunkY};
        kdTrunkRpy = {kdTrunkR, kdTrunkP, kdTrunkY};
        kpTrunkXyz = {kpTrunkX, kpTrunkYY, kpTrunkZ};
        kdTrunkXyz = {kdTrunkX, kdTrunkYY, kdTrunkZ};


        kpFootXyz = {800., 800., 800.};
        kdFootXyz = {15., 15., 15.};
        kpFootRpy = {kpFootR, kpFootP, kpFootY};
        kdFootRpy = {kdFootR, kdFootP, kdFootY};

        kpArmXyz = fillVector2(1200, 3);
        kdArmXyz = fillVector2(120, 3);
        kpArmRpy = fillVector2(1000, 3);
        kdArmRpy = fillVector2(100, 3);

        // Set weights 
        weightPelvisPosition = fillVector3(600000);                                           
        weightPelvisOrientation = fillVector3(600000); 
        weightTrunkPosition = fillVector3(300000);                                        
        weightTrunkOrientation << 300000, 300000, 3000000;

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