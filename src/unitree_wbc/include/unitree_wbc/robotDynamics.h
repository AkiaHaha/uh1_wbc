#ifndef TAICHI_ROBOTDYNAMICS_H
#define TAICHI_ROBOTDYNAMICS_H

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <rbdl/rbdl.h>

namespace TAICHI {

/**
 * @brief The JacobianTc struct: Jacobian structure in TAICHI.
 *  This structure is used to characterize the task coefficient matrix of acceleration-level tasks:
 *      xDDot = J * qDDot + JDot * qDot,
 *  here JdotQdot denotes JDot * qDot.
 *  For speed-level tasks, it is sufficient to determine J, because:
 *      xDot = J * qDot
 */
struct JacobianTc{
      Eigen::MatrixXd J;
      Eigen::VectorXd JdotQdot;
};

/**
 * @class RobotDynamics
 * @brief The RobotDynamics class
 * @details Dynamics calculation depends on the library 'RBDL'.
 * @see {https://rbdl.github.io/index.html}
 * @note
 * - This is an abstract class.
 * - The public members 'cstrSet, nlBiasWithInternalCstr, nspInternalCstrJacobian' are designed for parallel robots, they typically have no meaning for serial robots.
 */
class RobotDynamics {

public:

    RobotDynamics() = default;
    virtual ~RobotDynamics() = default;

    /**
     * @brief Set value of jntPositions and jntVelocities
     * @param q Generalized position
     * @param qdot Generalized velocity
     * @return
     * @note Always call this function first! As "jntPositions, jntVelocities" are the independent variables of all the RBDL funtions.
     */
    virtual bool setJntStates(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot);

    /**
     * @brief Display some necessary information
     * @return
     */
    virtual bool displayDynamicInformation();

    /**
     * @brief Get a boolean flag indicating whether the function calcWbcDependence() was executed successfully.
     * @return A boolean value
     */
    virtual bool isCalcWbcDependenceDone();

    /**
     * @brief Calculate WBC Dependence
     * @details The main content of the function is to calculate the robot dynamics dependencies for WBC
     * @note
     *  - Must call this function after setJntStates();
     *  - In the end of this function, set calcWbcDependenceDone to be true.
     */
    virtual bool calcWbcDependence() = 0;

    /**
     * @brief Get the total mass of the robot
     * @return totalMass of the model
     */
    virtual double getTotalMass() = 0;

    int NB;                                                 ///< Number of moving Bodys
    int NJG;                                                ///< Number of Joints of Generalized coordinates (q, qdot, qddot, tau). NJG = NJF + NJJ, normally.
    int NJF;                                                ///< Number of Joints Free-floating
    int NJJ;                                                ///< Number of non-floating-base Joints, including actuated & underactuated(paissive) joints. NJJ = NJA + NJP
    int NJA;                                                ///< Number of Joints Actuated (torque_actuated)
    int NJP;                                                ///< Number of Passive joints that do not contain the DoFs of floating base
    int NFC;                                                ///< Number of Forces describing Contact

    RigidBodyDynamics::Model * model  = nullptr;                        ///< Class that contains all information about the rigid body model. Such as gravity.

    RigidBodyDynamics::ConstraintSet cstrSet;                           ///< Structure that contains both constraint information and workspace memory. For parallel robot

    RigidBodyDynamics::Math::VectorNd jntPositions;                     ///< NJG*1, Generalized position
    RigidBodyDynamics::Math::VectorNd jntVelocities;                    ///< NJG*1, Generalized velocity

    RigidBodyDynamics::Math::MatrixNd inertiaMat;                       ///< NJG*NJG, Mass matrix
    RigidBodyDynamics::Math::MatrixNd invInertiaMat;                    ///< NJG*NJG, inverse of Mass matrix
    RigidBodyDynamics::Math::VectorNd nonlinearBias;                    ///< NJG*1, non-linear effects term caused by Coriolis and Gravity
    RigidBodyDynamics::Math::VectorNd coriolisBias;                     ///< NJG*1, non-linear effects term caused by Coriolis
    RigidBodyDynamics::Math::VectorNd gravityBias;                      ///< NJG*1, non-linear effects term caused by Gravity

    RigidBodyDynamics::Math::VectorNd nlBiasWithInternalCstr;           ///< NJG*1, non-linear effects term when considering internal-constraints. for parallel robot
    RigidBodyDynamics::Math::MatrixNd nspInternalCstrJacobian;          ///< NJG*NJG, Dynamically Consistent Null Space Projection matrix of Jacobian_InternalConstraint. for parallel robot

    RigidBodyDynamics::Math::MatrixNd selMatFloatingBase;               ///< NJF*NJG, Selection matrix of floating-base joints
    RigidBodyDynamics::Math::MatrixNd selMatNonFloatingBase;            ///< NJJ*NJG, Selection matrix of non-floating-base joints
    RigidBodyDynamics::Math::MatrixNd selMatActuated;                   ///< NJA*NJG, Selection matrix of actuated joints or Actuation matrix
    RigidBodyDynamics::Math::MatrixNd selMatPassive;                    ///< NJP*NJG, Selection matrix of passive joints that do not contain floating bases

    RigidBodyDynamics::Math::MatrixNd centroidalMomentumMatrix;         ///< NJF*NJG, Centroidal Momentum Matrix (CMM)
    RigidBodyDynamics::Math::VectorNd centroidalMomentumBiased;         ///< NJF*1, centroidal momentum bias force

    RigidBodyDynamics::Math::MatrixNd eqCstrMatTau;                     ///< NJA*?, equality constraints : TauActuated = eqCstrMatTau * x^T + eqCstrMatTauBias, x is the generalized variables, e.g. NJA*(NJG+NFC), x = [Qddot, f_c]'
    RigidBodyDynamics::Math::VectorNd eqCstrMatTauBias;                 ///< NJA*1, equality constraints : TauActuated = eqCstrMatTau * x^T + eqCstrMatTauBias

    JacobianTc biContactJacoTc;                                          ///< JacobianTc of contact point(s)
    JacobianTc quadContactJacoTc;                                          ///< JacobianTc of contact point(s)

    JacobianTc floatBaseJacoTc;                                        ///< JacobianTc of floating-base.
    JacobianTc upTorsoJacoTc;                                        ///< JacobianTc of floating-base.

    std::vector<JacobianTc> tasksJacoTc;                              ///< tasks JacobianTc, e.g. knee, elbow, etc.


protected:

    bool calcWbcDependenceDone{false};

    bool check(const Eigen::MatrixXd & M, int row, int col);
    bool check(const Eigen::VectorXd & v, int row);

};

} // namespace TAICHI

#endif // TAICHI_ROBOTDYNAMICS_H
