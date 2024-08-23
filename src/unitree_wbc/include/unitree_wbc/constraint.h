#ifndef HUMANOID_CONSTRAINT_H
#define HUMANOID_CONSTRAINT_H

#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include <ErrorMsg.h>

#include "robotDynamics.h"

namespace HUMANOID {

/**
 * @class Constraint
 * @brief The Constraint class, expression: lbC <= C*x <= ubC
 * @note this is an abstract class
 */
class Constraint{

public:

    /**
     * @brief Constraint Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The dimension of variables in the WBC problem
     */
    Constraint(const std::string & constrName, int constrDim, int varDim);

    virtual ~Constraint() = default;

    std::string name;       ///< the unique identification of the constraint
    int priority{0};        ///< the level of priority among constraints
    int dim{0};             ///< the dimension of constraint, determining the row of C, lbC, ubC
    int varDof{0};          ///< the dimension of variables in the optimization, determining the column of C
    Eigen::MatrixXd cstrMatC;       ///< constraint expression: lbC <= C*x <= ubC
    Eigen::VectorXd lbC;            ///< Constraint expression: lbC <= C*x <= ubC
    Eigen::VectorXd ubC;            ///< Constraint expression: lbC <= C*x <= ubC

    /**
     * @brief Set parameters
     *  Set value to some necessary paprameters (private members) in the derived classes.
     * @param params Vector used to pass parameters
     * @return
     * @note If necessary, you should rewrite this virtual function to set value to constraint parameters 
     */
    virtual bool setParameter(const std::vector<double> & params);

    /**
     * @brief Calculate and Update Constraint expression components: 'cstrMatC', 'lbC', 'ubC'
     * @param robot Class that contains necessary information about the rigid body model
     * @return
     */
    virtual bool update(const RobotDynamics & robot) = 0;

protected:

    bool check(const Eigen::MatrixXd & M, int row, int col);
    bool check(const Eigen::VectorXd & v, int row);

};

} // namespace HUMANOID

#endif // HUMANOID_CONSTRAINT_H
