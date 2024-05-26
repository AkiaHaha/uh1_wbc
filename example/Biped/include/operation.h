#ifndef OPERATION_H_
#define OPERATION_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <cstdarg>

void akiaPrint1(const Eigen::VectorXd &vector, int length, int numRows, ...);
std::string akiaPrint2(const Eigen::VectorXd &vector, int length, int numRows, ...);

#endif//OPERATION_H_