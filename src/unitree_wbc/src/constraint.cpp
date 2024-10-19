#include "constraint.h"

namespace AGIROBOT {

Constraint::Constraint(const std::string & constrName, int constrDim, int varDim){
    name = constrName;
    dim = constrDim;
    varDof = varDim;
    cstrMatC = Eigen::MatrixXd::Zero(dim, varDof);
    lbC = Eigen::VectorXd::Zero(dim);
    ubC = Eigen::VectorXd::Zero(dim);
}

bool Constraint::setParameter(const std::vector<double> & params){
    std::cout << "Nothing to be done, you should rewrite this function, if you want to use it."<< std::endl;
    return true;
}
} // namespace AGIROBOT
