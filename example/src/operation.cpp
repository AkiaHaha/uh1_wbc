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
