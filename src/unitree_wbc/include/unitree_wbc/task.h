#ifndef AGIROBOT_TASK_H
#define AGIROBOT_TASK_H

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include "robotDynamics.h"

namespace AGIROBOT {

/**
 * @class Task
 * @brief The Task class, expression: A*x = b, and we will minimize the l2-norm of A*x - b in the QPs
 * @note this is an abstract class
 */
class Task{

public:

    /**
     * @brief Task Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    Task(const std::string & taskName, int taskDim, int varDim);

    virtual ~Task() = default;

    std::string name;       ///< the unique identification of the task
    int priority{0};        ///< the level of priority among tasks
    int dim{0};             ///< the dimension of task, determining the row of A, b, w, ref
    int varDof{0};          ///< the DoF of variables in the WBC problem, determining the column of A
    Eigen::MatrixXd taskMatA;     ///< equality Task expression: A*x = b
    Eigen::VectorXd taskVecB;     ///< equality Task expression: A*x = b
    Eigen::VectorXd wei;       ///< weight: W*A*x = W*b, W is the diagonal matrix generated by the vector w
    Eigen::VectorXd ref;            ///< reference, contributing to the composition of b

    /**
     * @brief Set parameters
     *  If necessary, set value to some necessary paprameters (private members) in the derived classes.
     * @param params Vector used to pass parameters
     * @return
     */
    virtual bool setParameter(const std::vector<double> & params);

    /**
     * @brief Calculate and Update Task expression components: 'A', 'b'
     * @param robot Class that contains necessary information about the rigid body model
     * @return
     */
    virtual bool update(const RobotDynamics & robot) = 0;

    /**
     * @brief Update refence of Task
     * @param newRef Assign to vector ref
     * @return
     */
    bool updateRefence(const Eigen::VectorXd & newRef);

    /**
     * @brief Update weight of Task
     * @param newWei Assign to vector w
     * @return
     */
    bool updateWeight(const Eigen::VectorXd & newWei);

};

} // namespace AGIROBOT

#endif // AGIROBOT_TASK_H
