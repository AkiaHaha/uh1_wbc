#include "operation.h"

/**
 * @brief Prints the elements of an Eigen::VectorXd in a structured format.
 * @param vector The Eigen vector to be printed.
 * @param length The total number of elements in the vector.
 * @param numRows The number of rows to print.
 * @param ... A variable list of integers specifying the number of elements 
 *            to print in each row. The total number of integers passed 
 *            should equal numRows.
 * @example
 * Eigen::VectorXd myVector(10);
 * myVector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
 * akiaPrint1(myVector, myVector.size(), 3, 3, 4, 3); 
 * // This will print:
 * // 1 2 3 
 * // 4 5 6 7 
 * // 8 9 10 
 */
void akiaPrint1(const Eigen::VectorXd &vector, int length, int numRows, ...) {
    std::vector<int> elementsPerRow(numRows);
    va_list args;
    va_start(args, numRows);
    for (int i = 0; i < numRows; ++i) {
        elementsPerRow[i] = va_arg(args, int);
    }
    va_end(args);

    int currentIndex = 0;
    for (int i = 0; i < numRows; ++i) {
        int numElements = elementsPerRow[i];

        if (currentIndex + numElements > length) {
            numElements = length - currentIndex;
        }

        for (int j = 0; j < numElements; ++j) {
            if (currentIndex < length) {
                std::cout << vector[currentIndex] << " ";
                currentIndex++;
            }
        }
        std::cout << std::endl;
    }

    while (currentIndex < length) {
        std::cout << vector[currentIndex] << " ";
        currentIndex++;
    }
    if (currentIndex < length) {
        std::cout << std::endl;
    }
}

/**
 * @example
 * Eigen::VectorXd myVector(10);
 * myVector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
 * std::string output = akiaPrint2(myVector, myVector.size(), 3, 3, "Row 1", 4, "Row 2", 3, "Row 3");
 * std::cout << output;
 * // This will print:
 * // Row 1: 1 2 3 
 * // Row 2: 4 5 6 7 
 * // Row 3: 8 9 10 
 */
std::string akiaPrint2(const Eigen::VectorXd &vector, int length, int numRows, ...) {
    std::ostringstream oss;
    std::vector<int> elementsPerRow(numRows);
    std::vector<std::string> rowRemarks(numRows);

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

        if (currentIndex + numElements > length) {
            numElements = length - currentIndex;
        }

        oss << remark << ": ";

        for (int j = 0; j < numElements; ++j) {
            if (currentIndex < length) {
                oss << vector[currentIndex] << " ";
                currentIndex++;
            }
        }
        oss << "\n";
    }

    while (currentIndex < length) {
        oss << vector[currentIndex] << " ";
        currentIndex++;
    }
    if (currentIndex < length) {
        oss << "\n";
    }

    return oss.str();
}

/**
 * @example
 * Eigen::VectorXd myVector(10);
 * myVector << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10;
 * akiaPrint3(myVector, myVector.size(), 3, 3, "Row 1", 4, "Row 2", 3, "Row 3");
 * // This will print:
 * // Row 1: 1 2 3 
 * // Row 2: 4 5 6 7 
 * // Row 3: 8 9 10 
 */
void akiaPrint3(const Eigen::VectorXd &vector, int length, int numRows, ...) {
    std::vector<int> elementsPerRow(numRows);
    std::vector<std::string> rowRemarks(numRows);

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

        if (currentIndex + numElements > length) {
            numElements = length - currentIndex;
        }

        std::cout << remark << ": ";

        for (int j = 0; j < numElements; ++j) {
            if (currentIndex < length) {
                std::cout << vector[currentIndex] << " ";
                currentIndex++;
            }
        }
        std::cout << "\n";
    }

    while (currentIndex < length) {
        std::cout << vector[currentIndex] << " ";
        currentIndex++;
    }
    if (currentIndex < length) {
        std::cout << "\n";
    }
}


/**
 * @brief To generate a tigid body in RBDL
 */
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


/**
 * @brief A math tool for integrate;
 */
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

Eigen::MatrixXd createDiagonalMatrix(const Eigen::VectorXd& vec) {
    Eigen::MatrixXd mat(vec.size(), vec.size());
    mat.setZero();
    mat.diagonal() = vec;
    return mat;
}