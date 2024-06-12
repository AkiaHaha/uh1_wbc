#include "constraint.h"

namespace TAICHI {

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

bool Constraint::check(const Eigen::MatrixXd & M, int row, int col){

    #ifdef USE_ERROR
        if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [Constraint::check], matrix dimensions do not match" );
                }
        }
    #else
        try {
            if(M.rows() != row || M.cols() != col) {
                throw InvalidDimension(
                    "Error: In [Constraint::check], matrix dimensions do not match" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        } 
    #endif

    return true;
}

bool Constraint::check(const Eigen::VectorXd & v, int row){

    #ifdef USE_ERROR
        if(v.rows() != row) {
            throw InvalidDimension(
                "Error: In [Constraint::check], vector dimensions do not match!" );
            }
    #else

        try {
            if(v.rows() != row) {
                throw InvalidDimension(
                    "Error: In [Constraint::check], vector dimensions do not match!" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        }
    #endif

    return true;
}

} // namespace TAICHI
