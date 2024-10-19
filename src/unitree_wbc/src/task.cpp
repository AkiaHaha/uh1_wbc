#include "task.h"
#include <ErrorMsg.h>

namespace AGIROBOT {

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
    ref = newRef;
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

    wei = newWei;
    return true;
}

} // namespace AGIROBOT
