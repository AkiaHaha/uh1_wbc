#include "json.hpp"
#include <iostream>
#include <fstream>

struct ConfigParams {
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

    // Constructor to initialize from JSON file
    void loadFromJson(const std::string& filePath) {
        std::ifstream inputFile(filePath);
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
    }
};