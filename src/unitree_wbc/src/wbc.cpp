#include "wbc.h"

namespace AGIROBOT {

Wbc::Wbc(int dimVar, RobotDynamics * roDy){
    nV = dimVar;
    robot = roDy;
    tasks.clear();
    constraints.clear();
    priorityTaskNames.clear();
    priorityConstraintNames.clear();
}

bool Wbc::addTask(Task * const taskPtr, int priority, bool mandatory){

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
        // std::cout << iter->second->name << " ref: " << iter->second->updateRefence(ref) << std::endl;
        // std::cout << iter->second->name << " wei: " << iter->second->updateWeight(wei) << std::endl;

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

int Wbc::getNlevel(){
    std::cout << "Nothing to be output, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

} // namespace AGIROBOT
