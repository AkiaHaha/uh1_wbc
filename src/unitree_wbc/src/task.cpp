#include "task.h"
#include <ErrorMsg.h>

namespace HUMANOID {

Task::Task(const std::string & taskName, int taskDim, int varDim){
    name = taskName;
    dim = taskDim;
    varDof = varDim;
    taskMatA = Eigen::MatrixXd::Zero(dim, varDof);
    taskVecB = Eigen::VectorXd::Zero(dim);
    wei = Eigen::VectorXd::Zero(dim);
    ref = Eigen::VectorXd::Zero(dim);
}

bool Task::setParameter(const std::vector<double> & params){
    // TODO {throw}
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}

bool Task::updateRefence(const Eigen::VectorXd & newRef){
    if (check(newRef, dim)){
        ref = newRef;
    }else{
        return false;
    }
    return true;
}

bool Task::updateWeight(const Eigen::VectorXd & newWei){

    #ifdef USE_ERROR

        for(int i=0; i<newWei.size(); ++i) {
            if(newWei(i) < 0.) {
                throw NotPositive(
                    "Error: in [Task::updateWeight] the elements must be positive!"
                );
                break;
            }
        }
    #else

        for(int i=0; i<newWei.size(); ++i) {
            try {
                if(newWei(i) < 0.) {
                    throw NotPositive(
                        "Error: in [Task::updateWeight] the elements must be positive!"
                    );
                }
            } catch (NotPositive notPositive) {
                std::cout << notPositive.what() << std::endl;
                return false;
            }
        }
    #endif

    if (check(newWei, dim)){
        wei = newWei;
    }else{
        return false;
    }
    return true;
}

bool Task::check(const Eigen::MatrixXd & M, int row, int col){
    
    #ifdef USE_ERROR
        if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [Task::check], Matrix dimensions do not match" );
                }
        }
    #else
        try {
            if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [Task::check], matrix dimensions do not match" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        } 
    #endif
    return true;
}

bool Task::check(const Eigen::VectorXd & v, int row){
    #ifdef USE_ERROR
        if(v.rows() != row) {
            throw InvalidDimension(
                "Error : In [Task::check], vector dimensions do not match!" );
            }
    #else

        try {
            if(v.rows() != row) {
                throw InvalidDimension(
                    "`" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        }
    #endif

    return true;
}

} // namespace HUMANOID
