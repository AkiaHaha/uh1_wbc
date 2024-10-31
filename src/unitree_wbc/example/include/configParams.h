#include "json.hpp"
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
using json = nlohmann::json;


struct ConfigParams {
    // PArams for task control
    double kpPelvisR, kpPelvisP, kpPelvisY, kdPelvisR, kdPelvisP, kdPelvisY;
    double kpPelvisX, kpPelvisYY, kpPelvisZ, kdPelvisX, kdPelvisYY, kdPelvisZ;
    double kpTorsoR, kpTorsoP, kpTorsoY, kdTorsoR, kdTorsoP, kdTorsoY;
    double kpTorsoX, kpTorsoYY, kpTorsoZ, kdTorsoX, kdTorsoYY, kdTorsoZ;

    double kpFootR, kpFootP, kpFootY, kdFootR, kdFootP, kdFootY;
    double kpArmR, kpArmP, kpArmY, kdArmR, kdArmP, kdArmY;
    double kpFootX, kpFootYY, kpFootZ, kdFootX, kdFootYY, kdFootZ;
    double kpArmX, kpArmYY, kpArmZ, kdArmX, kdArmYY, kdArmZ;

    double weightGVL, weightGVL10, weightGVLKnee, weightGVLAnkle;
    double weightFootForceR, weightFootForceP, weightFootForceYaw;
    double weightFootForceX, weightFootForceY, weightFootForceZ;
    double weightArmForceR, weightArmForceP, weightArmForceYaw;
    double weightArmForceX, weightArmForceY, weightArmForceZ;
    double weightFBD, weightFootPos, weightFootYaw, weightArmPos, weightArmRPY, weightArmZ;

    double weightPelvisX, weightPelvisYY, weightPelvisZ, weightPelvisR, weightPelvisP, weightPelvisY;
    double weightTorsoX, weightTorsoYY, weightTorsoZ, weightTorsoR, weightTorsoP, weightTorsoY;

    // Params for motion plan
    double pelvisUpDown, pelvisForward, torsoUpDown, torsoForward, pitchApt, motionFrq, armForward, armUpDown, armAside; 

    // PD gains
    Eigen::Vector3d kpPelvisXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdPelvisXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpPelvisRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdPelvisRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpTorsoXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdTorsoXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpTorsoRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdTorsoRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpFootXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdFootXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpFootRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdFootRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpArmXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdArmXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d kpArmRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d kdArmRpy = Eigen::Vector3d::Zero();

    // Diag matrix of PD gains
    Eigen::Matrix3d kpPelvisRpyDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdPelvisRpyDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kpPelvisXyzDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdPelvisXyzDiag = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d kpTorsoRpyDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdTorsoRpyDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kpTorsoXyzDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdTorsoXyzDiag = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d kpFootXyzDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdFootXyzDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kpFootRpyDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdFootRpyDiag = Eigen::Matrix3d::Zero();

    Eigen::Matrix3d kpArmXyzDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdArmXyzDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kpArmRpyDiag = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d kdArmRpyDiag = Eigen::Matrix3d::Zero();

    // Weight value of tasks and constraints
    Eigen::Vector3d weightPelvisXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightPelvisRpy = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightTorsoXyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d weightTorsoRpy = Eigen::Vector3d::Zero();
    Eigen::VectorXd weightFootArmPosition = Eigen::VectorXd::Zero(NFCC4);// @Daniel240521
    Eigen::VectorXd weightFootForce = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd weightFootForceChange = Eigen::VectorXd::Zero(NFCC2);
    Eigen::VectorXd weightFootArmForce = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd weightFootArmForceChange = Eigen::VectorXd::Zero(NFCC4);
    Eigen::VectorXd weightFloatBaseDynamic = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd weightGVLimitation = Eigen::VectorXd::Constant(19,weightGVL);


    ConfigParams() {
        // Constructor to initialize from JSON file
        std::ifstream inputFile("src/unitree_wbc/config/controller.json");
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

        kpFootR = jsonData["kpFootR"];
        kpFootP = jsonData["kpFootP"];
        kpFootY = jsonData["kpFootY"];
        kdFootR = jsonData["kdFootR"];
        kdFootP = jsonData["kdFootP"];
        kdFootY = jsonData["kdFootY"];

        kpArmR = jsonData["kpArmR"];
        kpArmP = jsonData["kpArmP"];
        kpArmY = jsonData["kpArmY"];
        kdArmR = jsonData["kdArmR"];
        kdArmP = jsonData["kdArmP"];
        kdArmY = jsonData["kdArmY"];

        kpFootX = jsonData["kpFootX"];
        kpFootYY = jsonData["kpFootYY"];
        kpFootZ = jsonData["kpFootZ"];
        kdFootX = jsonData["kdFootX"];
        kdFootYY = jsonData["kdFootYY"];
        kdFootZ = jsonData["kdFootZ"];

        kpArmX = jsonData["kpArmX"];
        kpArmYY = jsonData["kpArmYY"];
        kpArmZ = jsonData["kpArmZ"];
        kdArmX = jsonData["kdArmX"];
        kdArmYY = jsonData["kdArmYY"];
        kdArmZ = jsonData["kdArmZ"];

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

        weightPelvisR = jsonData["weightPelvisR"];
        weightPelvisP = jsonData["weightPelvisP"];
        weightPelvisY = jsonData["weightPelvisY"];
        weightPelvisX = jsonData["weightPelvisX"];
        weightPelvisYY = jsonData["weightPelvisYY"];
        weightPelvisZ = jsonData["weightPelvisZ"];

        weightTorsoR = jsonData["weightTorsoR"];
        weightTorsoP = jsonData["weightTorsoP"];
        weightTorsoY = jsonData["weightTorsoY"];
        weightTorsoX = jsonData["weightTorsoX"];
        weightTorsoYY = jsonData["weightTorsoYY"];
        weightTorsoZ = jsonData["weightTorsoZ"];

        // Set PD gains
        Eigen::Vector3d kpPelvisXyz(kpPelvisX, kpPelvisYY, kpPelvisZ);
        Eigen::Vector3d kdPelvisXyz(kdPelvisX, kdPelvisYY, kdPelvisZ);
        Eigen::Vector3d kpPelvisRpy(kpPelvisR, kpPelvisP, kpPelvisY);
        Eigen::Vector3d kdPelvisRpy(kdPelvisR, kdPelvisP, kdPelvisY);
        Eigen::Vector3d kpTorsoXyz(kpTorsoX, kpTorsoYY, kpTorsoZ);
        Eigen::Vector3d kdTorsoXyz(kdTorsoX, kdTorsoYY, kdTorsoZ);
        Eigen::Vector3d kpTorsoRpy(kpTorsoR, kpTorsoP, kpTorsoY);
        Eigen::Vector3d kdTorsoRpy(kdTorsoR, kdTorsoP, kdTorsoY);
        Eigen::Vector3d kpFootXyz(kpFootX, kpFootYY, kpFootZ);
        Eigen::Vector3d kdFootXyz(kdFootX, kdFootYY, kdFootZ);
        Eigen::Vector3d kpFootRpy(kpFootR, kpFootP, kpFootY);
        Eigen::Vector3d kdFootRpy(kdFootR, kdFootP, kdFootY);
        Eigen::Vector3d kpArmXyz(kpArmX, kpArmYY, kpArmZ);
        Eigen::Vector3d kdArmXyz(kdArmX, kdArmYY, kdArmZ);
        Eigen::Vector3d kpArmRpy(kpArmR, kpArmP, kpArmY);
        Eigen::Vector3d kdArmRpy(kdArmR, kdArmP, kdArmY);
        
        
        // Calculate diagonal matrix 
        kpPelvisRpyDiag = createDiagonalMatrix(kpPelvisRpy);
        kdPelvisRpyDiag = createDiagonalMatrix(kdPelvisRpy);
        kpPelvisXyzDiag = createDiagonalMatrix(kpPelvisXyz);
        kdPelvisXyzDiag = createDiagonalMatrix(kdPelvisXyz);

        kpTorsoRpyDiag = createDiagonalMatrix(kpTorsoRpy);
        kdTorsoRpyDiag = createDiagonalMatrix(kdTorsoRpy);
        kpTorsoXyzDiag = createDiagonalMatrix(kpTorsoXyz);
        kdTorsoXyzDiag = createDiagonalMatrix(kdTorsoXyz);

        kpFootRpyDiag = createDiagonalMatrix(kpFootRpy);
        kdFootRpyDiag = createDiagonalMatrix(kdFootRpy);
        kpFootXyzDiag = createDiagonalMatrix(kpFootXyz);
        kdFootXyzDiag = createDiagonalMatrix(kdFootXyz);

        kpArmRpyDiag = createDiagonalMatrix(kpArmRpy);
        kdArmRpyDiag = createDiagonalMatrix(kdArmRpy);
        kpArmXyzDiag = createDiagonalMatrix(kpArmXyz);
        kdArmXyzDiag = createDiagonalMatrix(kdArmXyz);

        // Set weights 
        weightPelvisXyz = {weightPelvisX, weightPelvisY, weightPelvisZ};                                           
        weightPelvisRpy = {weightPelvisR, weightPelvisP, weightPelvisY}; 
        weightTorsoXyz = {weightTorsoX, weightTorsoY, weightTorsoZ};                                       
        weightTorsoRpy = {weightTorsoR, weightTorsoP, weightTorsoY};

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

        weightFootArmForceChange << weightFootForceR, weightFootForceP, weightFootForceYaw, 
                                    weightFootForceX, weightFootForceY, weightFootForceZ, 
                                    weightFootForceR, weightFootForceP, weightFootForceYaw, 
                                    weightFootForceX, weightFootForceY, weightFootForceZ,
                                    weightArmForceR, weightArmForceP, weightArmForceYaw, 
                                    weightArmForceX, weightArmForceY, weightArmForceZ, 
                                    weightArmForceR, weightArmForceP, weightArmForceYaw, 
                                    weightArmForceX, weightArmForceY, weightArmForceZ;

        weightFloatBaseDynamic = Eigen::VectorXd::Constant(6,weightFBD);

        weightGVLimitation(10) = weightGVL10;
        weightGVLimitation(3) = weightGVLKnee;
        weightGVLimitation(8) = weightGVLKnee;
        weightGVLimitation(4) = weightGVLAnkle;
        weightGVLimitation(9) = weightGVLAnkle;

        pelvisUpDown = jsonData["pelvisUpDown"];
        pelvisForward = jsonData["pelvisForward"];
        torsoUpDown = jsonData["torsoUpDown"];
        torsoForward = jsonData["torsoForward"];
        pitchApt = jsonData["pitchApt"];
        motionFrq = jsonData["motionFrq"];
        armForward = jsonData["armForward"];
        armUpDown = jsonData["armUpDown"];
        armAside = jsonData["armAside"];
    }
};