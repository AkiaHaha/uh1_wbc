#include "operation.h"

void akiaPrint1(const Eigen::VectorXd &vector, int length, int numRows, ...) {
    std::vector<int> elementsPerRow(numRows);

    // 获取可变参数
    va_list args;
    va_start(args, numRows);
    for (int i = 0; i < numRows; ++i) {
        elementsPerRow[i] = va_arg(args, int);
    }
    va_end(args);

    int currentIndex = 0;
    for (int i = 0; i < numRows; ++i) {
        int numElements = elementsPerRow[i];

        // 确保不超过向量长度
        if (currentIndex + numElements > length) {
            numElements = length - currentIndex;
        }

        // 打印每行的元素
        for (int j = 0; j < numElements; ++j) {
            if (currentIndex < length) {
                std::cout << vector[currentIndex] << " ";
                currentIndex++;
            }
        }
        std::cout << std::endl;
    }

    // 打印剩余元素（如果有）
    while (currentIndex < length) {
        std::cout << vector[currentIndex] << " ";
        currentIndex++;
    }
    if (currentIndex < length) {
        std::cout << std::endl;
    }
}

std::string akiaPrint2(const Eigen::VectorXd &vector, int length, int numRows, ...) {
    std::ostringstream oss;
    std::vector<int> elementsPerRow(numRows);
    std::vector<std::string> rowRemarks(numRows);

    // 获取可变参数
    va_list args;
    va_start(args, numRows);
    for (int i = 0; i < numRows; ++i) {
        elementsPerRow[i] = va_arg(args, int);
        rowRemarks[i] = va_arg(args, char*);
    }
    va_end(args);

    int currentIndex = 0;
    for (int i = 0; i < numRows; ++i) {
        int numElements = elementsPerRow[i];
        std::string remark = rowRemarks[i];

        // 确保不超过向量长度
        if (currentIndex + numElements > length) {
            numElements = length - currentIndex;
        }

        // 打印每行的备注
        oss << remark << ": ";

        // 打印每行的元素
        for (int j = 0; j < numElements; ++j) {
            if (currentIndex < length) {
                oss << vector[currentIndex] << " ";
                currentIndex++;
            }
        }
        oss << "\n";
    }

    // 打印剩余元素（如果有）
    while (currentIndex < length) {
        oss << vector[currentIndex] << " ";
        currentIndex++;
    }
    if (currentIndex < length) {
        oss << "\n";
    }

    return oss.str();
}

void akiaPrint3(const Eigen::VectorXd &vector, int length, int numRows, ...) {
    std::vector<int> elementsPerRow(numRows);
    std::vector<std::string> rowRemarks(numRows);

    // 获取可变参数
    va_list args;
    va_start(args, numRows);
    for (int i = 0; i < numRows; ++i) {
        elementsPerRow[i] = va_arg(args, int);
        rowRemarks[i] = va_arg(args, char*);
    }
    va_end(args);

    int currentIndex = 0;
    for (int i = 0; i < numRows; ++i) {
        int numElements = elementsPerRow[i];
        std::string remark = rowRemarks[i];

        // 确保不超过向量长度
        if (currentIndex + numElements > length) {
            numElements = length - currentIndex;
        }

        // 打印每行的备注
        std::cout << remark << ": ";

        // 打印每行的元素
        for (int j = 0; j < numElements; ++j) {
            if (currentIndex < length) {
                std::cout << vector[currentIndex] << " ";
                currentIndex++;
            }
        }
        std::cout << "\n";
    }

    // 打印剩余元素（如果有）
    while (currentIndex < length) {
        std::cout << vector[currentIndex] << " ";
        currentIndex++;
    }
    if (currentIndex < length) {
        std::cout << "\n";
    }
}

RigidBodyDynamics::Body BodyAkia(double mass,
                                                    const RigidBodyDynamics::Math::Vector3d& com,
                                                    double Ixx, double Iyy, double Izz,
                                                    double Ixy, double Ixz, double Iyz) {
    RigidBodyDynamics::Math::Matrix3d inertia_C;
    inertia_C << Ixx, Ixy, Ixz,
                 Ixy, Iyy, Iyz,
                 Ixz, Iyz, Izz;

    return RigidBodyDynamics::Body(mass, com, inertia_C);
}


Integrator::Integrator() : Qd_prev(0), Q_prev(0), dt(0.001) {}

double Integrator::Integrate(double Qdd) {
    double Qd = Qd_prev + Qdd * dt;
    double Q = Q_prev + Qd * dt;
    Qd_prev = Qd;
    Q_prev = Q;

    return Q;
}

Eigen::VectorXd fillVector(double firstValue, double secondValue) {
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(NFCC4);
    
    for (int i = 0; i < 12; ++i) {
        vector(i) = firstValue;
    }

    for (int i = 12; i < NFCC4; ++i) {
        vector(i) = secondValue;
    }

    return vector;
}


std::vector<double> fillVector2(double value, int length) {
    std::vector<double> vector(length, value);
    return vector;
}

Eigen::Vector3d fillVector3(double value){
    Eigen::Vector3d vector = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        vector(i) = value;
    }
    return vector;
}