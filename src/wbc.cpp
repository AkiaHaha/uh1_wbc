/**
 *	This file is part of TAICHI.
 *
 *	TAICHI -- Task Arrangement In Control HIerarchy.
 *	Copyright (C) 2015-2021 Beijing Research Institute of UBTECH Robotics.
 *	All rights reserved.
 *
 *	Licensed under the Apache License 2.0. See LICENSE for more details.
 */

/**
 * @file Wbc.cpp
 * @brief Function implementation part of class Wbc
 * @author Jiajun Wang
 * @date 2020-2021
 * @version alpha
 */

#include "wbc.h"

namespace TAICHI {

// ======================================== public Functions ====================================================

Wbc::Wbc(int dimVar, RobotDynamics * roDy){
    nV = dimVar;
    robot = roDy;
    tasks.clear();
    constraints.clear();
    priorityTaskNames.clear();
    priorityConstraintNames.clear();
}

Wbc::Wbc(const Wbc & wbcInstance){
    copyFromWbc(wbcInstance);
}

bool Wbc::addTask(Task * const taskPtr, int priority, bool mandatory){

    #ifdef USE_ERROR
        if(taskPtr == nullptr) {
            throw IsNullptr(
                "Error : In [Wbc::addTask], taskPtr = nullptr!" );
            }
    #else

        try {
            if(taskPtr == nullptr) {
                throw IsNullptr(
                    "Error : In [Wbc::addTask], taskPtr = nullptr!" );
                }
        } catch ( IsNullptr isNullptr ) {
            std::cout << isNullptr.what() << std::endl;
            return false;
        }
    #endif

    if (tasks.find(taskPtr->name) != tasks.end()){
        if (mandatory){
            std::cout << "Note : the name of Task already exists. We will remove the former and add this new one!" << std::endl;
            removeTask(taskPtr->name);
        }else{
            std::cout << "Note : the name of Task already exists. We will maintain the former and ignore this new one!" << std::endl;
            return true;
        }
    }

    taskPtr->priority = priority;
    tasks.insert({taskPtr->name, taskPtr});
    nO += taskPtr->dim;
    nOChange = true;

    if (priority >= static_cast<int>(priorityTaskNames.size())){
        for(int i = static_cast<int>(priorityTaskNames.size()); i <= priority; ++i){
            std::vector<std::string> temp;
            priorityTaskNames.push_back(temp);
        }
    }
    priorityTaskNames.at(priority).emplace_back(taskPtr->name);

    return true;
}

bool Wbc::addConstraint(Constraint * const cstrPtr, int priority, bool mandatory){
    #ifdef USE_ERROR
        if(cstrPtr == nullptr) {
            throw IsNullptr(
                "Error : In [Wbc::addConstraint], constrPtr = nullptr!" );
            }
    #else

        try {
            if(cstrPtr == nullptr) {
                throw IsNullptr(
                    "Error : In [Wbc::addConstraint], constrPtr = nullptr!" );
                }
        } catch ( IsNullptr isNullptr ) {
            std::cout << isNullptr.what() << std::endl;
            return false;
        }
    #endif

    if (tasks.find(cstrPtr->name) != tasks.end()){
        if (mandatory){
            std::cout << "Note : the name of Constraint already exists. We will remove the former and add this new one!" << std::endl;
            removeConstraint(cstrPtr->name);
        }else{
            std::cout << "Note : the name of Constraint already exists. We will maintain the former and ignore this new one!" << std::endl;
            return true;
        }
    }

    cstrPtr->priority = priority;
    constraints.insert({cstrPtr->name, cstrPtr});
    nC += cstrPtr->dim;
    nCChange = true;

    if (priority >= static_cast<int>(priorityConstraintNames.size())){
        for(int i = static_cast<int>(priorityConstraintNames.size()); i <= priority; ++i){
            std::vector<std::string> temp;
            priorityConstraintNames.push_back(temp);
        }
    }
    priorityConstraintNames.at(priority).emplace_back(cstrPtr->name);

    return true;
}

bool Wbc::removeTask(const std::string &taskName){

    auto iter = tasks.find(taskName);

    if (iter == tasks.end()){
        std::cout << "Warning : the Task name : " << taskName << " NOT FOUND!" << std::endl;
    }else {
        std::cout << std::endl;
        auto name_iter = std::find(priorityTaskNames.at(iter->second->priority).begin(), priorityTaskNames.at(iter->second->priority).end(), taskName);
        if (name_iter != priorityTaskNames.at(iter->second->priority).end()){
            priorityTaskNames.at(iter->second->priority).erase(name_iter);
        }

        nO -= iter->second->dim;
        tasks.erase(taskName);

        nOChange = true;
    }

    return true;
}

bool Wbc::removeConstraint(const std::string &constrName){

    auto iter = constraints.find(constrName);

    if (iter == constraints.end()){
        std::cout << "Warning : the Constraint name : " << constrName << " NOT FOUND!" << std::endl;
    }else {
        auto name_iter = std::find(priorityConstraintNames.at(iter->second->priority).begin(), priorityConstraintNames.at(iter->second->priority).end(), constrName);
        if (name_iter != priorityConstraintNames.at(iter->second->priority).end()){
            priorityConstraintNames.at(iter->second->priority).erase(name_iter);
        }

        nC -= iter->second->dim;
        constraints.erase(constrName);

        nCChange = true;
    }

    return true;
}

bool Wbc::adjustTaskPriority(const std::string &taskName, int priority){

    auto iter = tasks.find(taskName);
    if (iter == tasks.end()){
        std::cout << "Warning : the Task name : " << taskName << " NOT FOUND!" << std::endl;
    }else {
        if(iter->second->priority != priority){
            auto ptrTemp = iter->second;
            removeTask(taskName);
            addTask(ptrTemp, priority);
        }
    }

    return true;
}

bool Wbc::adjustConstraintPriority(const std::string &constrName, int priority){
    auto iter = constraints.find(constrName);

    if (iter == constraints.end()){
        std::cout << "Warning : the Constraint name : " << constrName << " NOT FOUND!" << std::endl;
    }else {
        if(iter->second->priority != priority){
            auto ptrTemp = iter->second;
            removeConstraint(constrName);
            addConstraint(ptrTemp, priority);
        }
    }

    return true;
}

bool Wbc::clearTask(){
    tasks.clear();
    nO = 0;
    nOChange = true;
    priorityTaskNames.clear();
    return true;
}

bool Wbc::clearConstraint(){
    constraints.clear();
    nC = 0;
    nCChange = true;
    priorityConstraintNames.clear();
    return true;
}

bool Wbc::updateTask(const std::string & taskName,
                     const Eigen::VectorXd & ref,
                     const Eigen::VectorXd & wei,
                     const std::vector<double> * params){
    
    auto iter = tasks.find(taskName);

    #ifdef USE_ERROR
        if(iter == tasks.end()) {
            throw NotExist(
                "Error : In [Wbc::updateTask], the task does not exist!" );
            }
    #else

        try {
            if(iter == tasks.end()) {
                throw NotExist(
                    "Error : In [Wbc::updateTask], The task does not exist!" );
                }
        } catch ( NotExist notExist ) {
            std::cout << notExist.what() << std::endl;
            return false;
        }
    #endif

    if (params == nullptr){
        if (iter->second->updateRefence(ref) && iter->second->updateWeight(wei)){
            iter->second->update(* robot);
        }else{
            return false;
        }
    }else{
        if (iter->second->updateRefence(ref) && iter->second->updateWeight(wei) && iter->second->setParameter(*params)){
            iter->second->update(* robot);
        }else{
            return false;
        }
    }

    return true;
}

bool Wbc::updateTask(const std::string & taskName,
                     const Eigen::VectorXd & ref,
                     const std::vector<double> * params){

    auto iter = tasks.find(taskName);
    #ifdef USE_ERROR
        if(iter == tasks.end()) {
            throw NotExist(
                "Error : In [Wbc::updateTask], the task does not exist!" );
            }
    #else

        try {
            if(iter == tasks.end()) {
                throw NotExist(
                    "Error : In [Wbc::updateTask], The task does not exist!" );
                }
        } catch ( NotExist notExist ) {
            std::cout << notExist.what() << std::endl;
            return false;
        }
    #endif

    if (params == nullptr){
        if (iter->second->updateRefence(ref)){
            iter->second->update(* robot);
        }else{
            return false;
        }
    }else{
        if (iter->second->updateRefence(ref) && iter->second->setParameter(*params)){
            iter->second->update(* robot);
        }else{
            return false;
        }
    }

    return true;
}

bool Wbc::updateConstraint(const std::string & constrName,
                           const std::vector<double> * params){

    auto iter = constraints.find(constrName);
    #ifdef USE_ERROR
        if(iter == constraints.end()) {
            throw NotExist(
                "Error : In [Wbc::updateConstraint], the constraint does not exist!" );
            }
    #else

        try {
            if(iter == constraints.end()) {
                throw NotExist(
                    "Error : In [Wbc::updateConstraint], The constraint does not exist!" );
                }
        } catch ( NotExist notExist ) {
            std::cout << notExist.what() << std::endl;
            return false;
        }
    #endif

    if (params == nullptr){
        iter->second->update(* robot);
    }else{
        if (iter->second->setParameter(*params)){
            iter->second->update(* robot);
        }else{
            return false;
        }
    }

    return true;
}

bool Wbc::getDimension(int &varDim, int &objDim, int &conDim) const{
    varDim = nV;
    objDim = nO;
    conDim = nC;
    return true;
}

bool Wbc::getRobotPointer(RobotDynamics *roDy) const{
    roDy = robot;
    return true;
}

bool Wbc::getContainers(std::unordered_map<std::string, Task *> &wbcTasks,
                        std::unordered_map<std::string, Constraint *> &wbcConstraints,
                        std::vector<std::vector<std::string> > &wbcPriorityTaskNames,
                        std::vector<std::vector<std::string> > &wbcPriorityConstraintNames) const{
    wbcTasks = tasks;
    wbcConstraints = constraints;
    wbcPriorityTaskNames = priorityTaskNames;
    wbcPriorityConstraintNames = priorityConstraintNames;
    return true;
}

bool Wbc::copyFromWbc(const Wbc &wbcInstance){
    wbcInstance.getContainers(tasks, constraints, priorityTaskNames, priorityConstraintNames);
    wbcInstance.getDimension(nV, nO, nC);
    wbcInstance.getRobotPointer(robot);
    return true;
}

bool Wbc::wbcInit(){
    nOChange = false;
    nCChange = false;
    nVChange = false;

    displayWbcInformation();

    return true;
}

bool Wbc::displayWbcInformation() const{

    std::cout << "###################   WBC  --  Information   ##################" << std::endl
              << "-------------------------   Variable   ------------------------" << std::endl
              << "variables dimension : nV = " << nV << std::endl
              << "---------------------------   Task   --------------------------" << std::endl
              << "All tasks dimension : nO = " << nO << std::endl
              << "Level" << "\t" << " |   " << "Item Keys" << std::endl;
    for(int i = 0; i != static_cast<int>(priorityTaskNames.size()); ++i){
        std::cout << i << "\t" << " |   ";
        for(auto item : priorityTaskNames.at(i)){
            std::cout << item << "(" << tasks.find(item)->second->dim << ")" << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------------   Constraint   ----------------------" << std::endl
              << "All constraints dimension : nC = " << nC << std::endl
              << "Level" << "\t" << " |   " << "Item Keys" << std::endl;
    for(int i = 0; i != static_cast<int>(priorityConstraintNames.size()); ++i){
        std::cout << i << "\t" << " |   " ;
        for(auto item : priorityConstraintNames.at(i)){
            std::cout << item << "(" << constraints.find(item)->second->dim << ")" << " | ";
        }
        std::cout << std::endl;
    }
    std::cout << "###############################################################" << std::endl;

    return true;
}

bool Wbc::displayResultInformation() const{
    std::cout << "Nothing to be displayed, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::setParametersInt(const std::vector<int> &parameters){
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::setParametersDouble(const std::vector<double> &parameters){
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::getAuxiliaryDataInt(std::vector<int> &auxiliaryData){
    auxiliaryData.clear();
    std::cout << "Nothing to be output, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Wbc::getAuxiliaryDataDouble(std::vector<double> &auxiliaryData){
    auxiliaryData.clear();
    std::cout << "Nothing to be output, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

// ======================================== protected Functions ====================================================

bool Wbc::check(const Eigen::MatrixXd & M, int row, int col){
    #ifdef USE_ERROR
        if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [Wbc::check], matrix dimensions do not match" );
                }
        }
    #else
        try {
            if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [Wbc::check], matrix dimensions do not match" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        } 
    #endif
    return true;
}

bool Wbc::check(const Eigen::VectorXd & v, int row){
    #ifdef USE_ERROR
        if(v.rows() != row) {
            throw InvalidDimension(
                "Error : In [Wbc::check], vector dimensions do not match!" );
            }
    #else

        try {
            if(v.rows() != row) {
                throw InvalidDimension(
                    "In [Wbc::check], vector dimensions do not match!" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        }
    #endif
    return true;
}

} // namespace TAICHI
