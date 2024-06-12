#ifndef TAICHI_HQPWBCSOLVER_H
#define TAICHI_HQPWBCSOLVER_H

#include <qpOASES.hpp>

#include "wbc.h"

namespace TAICHI {

/**
 * @class HqpWbc
 * @brief The HqpWbc class, a subclass derived from the abstract class Wbc, using the hierarchial QP algorithm.
 * @note The QP solver is qpOASES
 * @see {https://github.com/coin-or/qpOASES}
 */
class HqpWbc : public Wbc {
public:

    /**
     * @brief HqpWbc Constructor
     * @param dimVar Dimension of Variables in the WBC optimziation problem
     * @param roDy Control object of WBC problem
     */
    HqpWbc(int dimVar, RobotDynamics * roDy);

    /**
     * @brief Copy constructor of HqpWbc
     * @param wbcInstance the wbc instance to be copied from
     */
    HqpWbc(const Wbc &wbcInstance);

    // ---------------------- rewrite virtual functions ------------------------

    ~HqpWbc();

    bool wbcInit();

    bool displayResultInformation() const;
    /**
     * @brief Set the parameters of the integer type
     * @param parameters Input, the value of nWsrRes
     * @return
     */
    bool setParametersInt(const std::vector<int> & parameters);                 ///< nWSR
    /**
     * @brief Set the parameters of the double type
     * @param parameters Input, the value of cpuTimeDes
     * @return
     */
    bool setParametersDouble(const std::vector<double> & parameters);           ///< _cpu_time
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

    std::vector<qpOASES::SQProblem *> QP;
    std::vector<qpOASES::Options> qpOption;

    std::vector<qpOASES::int_t> nWSR;
    std::vector<qpOASES::real_t *> cpuTimePtr;

    std::vector<qpOASES::returnValue> statusCodeSolving;
    std::vector<qpOASES::returnValue> statusCodeGetSolution;

    std::vector<qpOASES::real_t *> primalOptPtr;
    std::vector<qpOASES::real_t *> dualOptPtr;

    std::vector<bool> initDone;

    std::vector<int> nWSRDes;
    std::vector<double> cpuTimeDes;

    // ---------------------------- Costs/Objects --------------------------------------

    /// Hessian matrix, H = A^T * A
    std::vector<Eigen::MatrixXd> hessianMat;                            ///< nVi*nVi, a Real symmetric matrix
    /// gradient vector, g = - A^T * b
    std::vector<Eigen::VectorXd> gradientVec;                           ///< nVi*1
    /// weight
    std::vector<Eigen::VectorXd> weiVecAll;                             ///< nOi*1
    /// A
    std::vector<Eigen::MatrixXd> taskMatAll;                            ///< nOi*nVi
    /// b
    std::vector<Eigen::VectorXd> taskVecAll;                            ///< nOi*1

    // ------------------------ Bounds & Constraints -----------------------------------

    /// lb,ub
    std::vector<Eigen::VectorXd> lowerBoundVec;                         ///< nVi*1
    std::vector<Eigen::VectorXd> upperBoundVec;                         ///< nVi*1
    /// C
    std::vector<Eigen::MatrixXd> cstrMatAll;                            ///< nCi*nVi
    std::vector<Eigen::MatrixXd> cstrMatAllTrans;                       ///< nVi*nCi, the transpose of cstrMatAll
    /// lbC,ubC
    std::vector<Eigen::VectorXd> lbCstrAll;                             ///< nCi*1
    std::vector<Eigen::VectorXd> ubCstrAll;                             ///< nCi*1

    // ------------------------ private Functions -----------------------------------
    bool createNewQP();
    bool resizeQPMatrixVector();

    /**
     * @brief qpUpdate Calculate H,g,C,... for qpOASES
     * @return
     */
    bool hqpUpdate();
    bool calcHessianGradient();
    bool calcConstraintCoefficient();

    bool nspSolve();
    bool hqpUpdateLeveln(const int & iLevel);
    bool hqpSolveLeveln(const int & iLevel);
    bool qpResetLeveln(const int & iLevel);

    bool getResultOptLeveln(Eigen::VectorXd & resultOpt, const int & iLevel);
    bool getPrimalOptLeveln(Eigen::VectorXd & primalOpt, const int & iLevel);
    bool getDualOptLeveln(Eigen::VectorXd & dualOpt, const int & iLevel);

    bool getNwsr(std::vector<int> & nWsrRes);
    bool getSimpleStatusInt(std::vector<int> & simpleStatus);
    bool getOptCost(std::vector<double> & costOpt);

    int nLevel{0};
    int nLevelTask{0};
    int nLevelConstraint{0};

    std::vector<int> nVLevel;
    std::vector<int> nOLevel;
    std::vector<int> nCLevel;

    std::vector<Eigen::MatrixXd> nspMat;                                ///< nVi*nVi, Null Space Projection Matrix
    std::vector<Eigen::VectorXd> optVarStar;                            ///< nVi*1, Optimized variables labeled 'Star', x_star

    /// wei_level
    std::vector<Eigen::VectorXd> weiVecLevel;                           ///< nOi*1
    /// A_level
    std::vector<Eigen::MatrixXd> taskMatLevel;                          ///< nOi*nVi
    /// AHat_level
    std::vector<Eigen::MatrixXd> taskMatHatLevel;                       ///< nOi*nVi
    /// AHatInv_level
    std::vector<Eigen::MatrixXd> taskMatHatInvLevel;                    ///< nVi*nOi
    /// b_level
    std::vector<Eigen::VectorXd> taskVecLevel;                          ///< nOi*1
    /// C_level
    std::vector<Eigen::MatrixXd> cstrMatLevel;                          ///< nCi*nVi
    std::vector<Eigen::MatrixXd> cstrMatLevelTrans;                     ///< nVi*nCi, the transpose of cstrMatLevel
    /// lbC_level,ubC_level
    std::vector<Eigen::VectorXd> lbCstrLevel;                           ///< nCi*1
    std::vector<Eigen::VectorXd> ubCstrLevel;                           ///< nCi*1
};

} // namespace TAICHI

#endif // TAICHI_HQPWBCSOLVER_H
