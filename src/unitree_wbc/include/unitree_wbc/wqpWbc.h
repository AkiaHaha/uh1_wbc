#ifndef TAICHI_WQPWBCSOLVER_H
#define TAICHI_WQPWBCSOLVER_H

#include <qpOASES.hpp>

#include "wbc.h"

namespace TAICHI {

/**
 * @class WqpWbc
 * @brief The WqpWbc class, a subclass derived from the abstract class Wbc, using the weighted QP algorithm.
 * @note The QP solver is qpOASES
 * @see {https://github.com/coin-or/qpOASES}
 */
class WqpWbc : public Wbc {
public:

    /**
     * @brief WqpWbc Constructor
     * @param dimVar Dimension of Variables in the WBC optimziation problem
     * @param roDy Control object of WBC problem
     */
    WqpWbc(int dimVar, RobotDynamics * roDy);

    /**
     * @brief Copy constructor of WqpWbc
     * @param wbcInstance the wbc instance to be copied from
     */
    WqpWbc(const Wbc &wbcInstance);

    // ---------------------- rewrite virtual functions ------------------------

    ~WqpWbc();

    bool wbcInit();

    bool displayResultInformation() const;

    /**
     * @brief Set the parameters of the integer type
     * @param parameters Input, the value of nWsrRes
     * @return
     */
    bool setParametersInt(const std::vector<int> & parameters);

    /**
     * @brief Set the parameters of the double type
     * @param parameters Input, the value of cpuTimeDes
     * @return
     */
    bool setParametersDouble(const std::vector<double> & parameters);

    /**
     * @brief Get Auxiliary Data of the integer type
     * @param auxiliaryData Output, the value of nWSR, simpleStatus
     * @return
     */
    bool getAuxiliaryDataInt(std::vector<int> & auxiliaryData);

    /**
     * @brief Get Auxiliary Data of the double type
     * @param auxiliaryData Output, the value of costOpt, cpuTimePtr
     * @return
     */
    bool getAuxiliaryDataDouble(std::vector<double> & auxiliaryData);


    // ---------------------- implement pure virtual functions -----------------

    bool updateBound(const Eigen::VectorXd & lb, const Eigen::VectorXd & ub);

    bool updateRobotDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & qDot);

    bool wbcSolve();

    bool getResultOpt(Eigen::VectorXd & resultOpt);


private:

    qpOASES::SQProblem * QP = nullptr;
    qpOASES::Options qpOption;

    qpOASES::int_t nWSR;
    qpOASES::real_t * cpuTimePtr = nullptr;

    qpOASES::returnValue statusCodeSolving;
    qpOASES::returnValue statusCodeGetSolution;

    qpOASES::real_t * primalOptPtr = nullptr;
    qpOASES::real_t * dualOptPtr = nullptr;

    bool initDone{false};
    int nWSRDes{100};
    double cpuTimeDes{10};

    // ---------------------------- Costs/Objects --------------------------------------

    /// Hessian matrix, H = A^T * A
    Eigen::MatrixXd hessianMat;                             ///< nV*nV, a Real symmetric matrix
    /// gradient vector, g = - A^T * b
    Eigen::VectorXd gradientVec;                            ///< nV*1
    /// weight
    Eigen::VectorXd weiVecAll;                              ///< nO*1
    /// A
    Eigen::MatrixXd taskMatAll;                             ///< nO*nV
    /// b
    Eigen::VectorXd taskVecAll;                             ///< nO*1

    // ------------------------ Bounds & Constraints -----------------------------------

    /// lb,ub
    Eigen::VectorXd lowerBoundVec;                          ///< nV*1
    Eigen::VectorXd upperBoundVec;                          ///< nV*1
    /// C
    Eigen::MatrixXd cstrMatAll;                             ///< nC*nV
    Eigen::MatrixXd cstrMatAllTrans;                        ///< nV*nC, the transpose of cstrMatAll
    /// lbC,ubC
    Eigen::VectorXd lbCstrAll;                              ///< nC*1
    Eigen::VectorXd ubCstrAll;                              ///< nC*1

    // ------------------------ private Functions -----------------------------------
    bool createNewQP();
    bool resizeQPMatrixVector();

    /**
     * @brief qpUpdate Calculate H,g,C,... for qpOASES
     * @return
     */
    bool qpUpdate();
    bool calcHessianGradient();
    bool calcConstraintCoefficient();
    bool qpSolve();
    bool qpReset();

    bool getPrimalOpt(Eigen::VectorXd & primalOpt);
    bool getDualOpt(Eigen::VectorXd & dualOpt);

    bool getNwsr(int & nWsrRes);
    bool getSimpleStatusInt(int & simpleStatus);
    bool getOptCost(double & costOpt);

};

} // namespace TAICHI

#endif // TAICHI_WQPWBCSOLVER_H
