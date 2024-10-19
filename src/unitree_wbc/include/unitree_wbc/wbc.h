#ifndef AGIROBOT_WBC_H
#define AGIROBOT_WBC_H

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>

#include <Eigen/Dense>

#include "task.h"
#include "constraint.h"
#include "robotDynamics.h"

namespace AGIROBOT {

/**
 * @class Wbc
 * @brief The Wbc class
 * @note this is an abstract class
 */
class Wbc{
public:

    /**
     * @brief Wbc Constructor
     * @param dimVar Dimension of Variables in the WBC problem
     * @param roDy Control object of WBC problem
     */
    Wbc(int dimVar, RobotDynamics * roDy);

    /**
     * @brief Copy constructor of Wbc
     * @param wbcInstance the wbc instance to be copied from
     */
    Wbc(const Wbc &wbcInstance);

    // ================================================== virtual funcions ====================================================

    virtual ~Wbc() = default;

    /**
     * @brief Initialization function of Wbc
     * @details Generally, this function is called only once, immediately after its instantiation.
     * @return
     */
    virtual bool wbcInit();

    /**
     * @brief Add new element to tasks
     * @param taskPtr Pointer to Task class
     * @param priority The level of task priority
     * @param mandatory Whether to replace, if the same key already exists. false by default
     * @return
     */
    virtual bool addTask(Task * const taskPtr, int priority, bool mandatory = false);

    /**
     * @brief Add new element to constraints
     * @param cstrPtr Pointer to Constraint class
     * @param priority The level of priority
     * @param mandatory Whether to replace, if the same key already exists. false by default
     * @return
     */
    virtual bool addConstraint(Constraint * const cstrPtr, int priority, bool mandatory = false);

    /**
     * @brief Remove an element from tasks
     * @param taskName The key of the element to be removed
     * @return
     */
    virtual bool removeTask(const std::string & taskName);

    /**
     * @brief Remove an element from constraints
     * @param cstrName The key of the element to be removed
     * @return
     */
    virtual bool removeConstraint(const std::string & cstrName);

    /**
     * @brief Adjust Task Priority
     * @param taskName The key of the element to be adjusted
     * @param priority New level value
     * @return
     */
    virtual bool adjustTaskPriority(const std::string & taskName, int priority);

    /**
     * @brief Adjust Constraint Priority
     * @param cstrName The key of the element to be adjusted
     * @param priority New level value
     * @return
     */
    virtual bool adjustConstraintPriority(const std::string & cstrName, int priority);

    /**
     * @brief Remove all elements from tasks
     * @return
     */
    virtual bool clearTask();

    /**
     * @brief Remove all elements from constraints
     * @return
     */
    virtual bool clearConstraint();

    /**
     * @brief Update the memeber value in the named Task
     * @param taskName The key of element
     * @param ref Reference Vector
     * @param wei Weight Vector
     * @param params Parameters if necessary
     * @return
     */
    virtual bool updateTask(const std::string & taskName,
                            const Eigen::VectorXd & ref,
                            const Eigen::VectorXd & wei,
                            const std::vector<double> * params = nullptr);

    /**
     * @brief Update the memeber value in the named Task
     * @param taskName The key of element
     * @param ref Reference Vector
     * @param params Parameters if necessary
     * @return
     */
    virtual bool updateTask(const std::string & taskName,
                            const Eigen::VectorXd & ref,
                            const std::vector<double> * params = nullptr);

    /**
     * @brief Update the memeber value in the named Constraint
     * @param constrName The key of element
     * @param params Parameters if necessary
     * @return
     */
    virtual bool updateConstraint(const std::string & cstrName,
                                  const std::vector<double> * params = nullptr);

    /**
     * @brief get Dimension of certain members
     * @param varDim dimension of Variables
     * @param objDim dimension of Tasks/Objects/Costs
     * @param cstrDim dimension of Constraints
     * @return
     */
    virtual bool getDimension(int & varDim, int & objDim, int & cstrDim) const;

    /**
     * @brief get the Pointer of RobotDynamics, i.e. the control object of WBC problem
     * @param roDy the variable to be assigned
     * @return
     */
    virtual bool getRobotPointer(RobotDynamics * roDy) const;

    /**
     * @brief get the main containers of class WBC
     * @param wbcTasks dictionary of tasks
     * @param wbcConstraints dictionary of constraints
     * @param wbcPriorityTaskNames Two-dimensional table of task priority
     * @param wbcPriorityConstraintNames Two-dimensional table of constraints priority
     * @return
     */
    virtual bool getContainers(std::unordered_map<std::string, Task *> & wbcTasks,
                               std::unordered_map<std::string, Constraint *> & wbcConstraints,
                               std::vector<std::vector<std::string>> & wbcPriorityTaskNames,
                               std::vector<std::vector<std::string>> & wbcPriorityConstraintNames) const;

    /**
     * @brief copy from another wbc
     * @param wbcInstance the wbc instance to be copied from
     * @return
     */
    virtual bool copyFromWbc(const Wbc & wbcInstance);

    /**
     * @brief Display WBC Information
     * @return
     */
    virtual bool displayWbcInformation() const;

    /**
     * @brief display Result Information
     * @return
     */
    virtual bool displayResultInformation() const;

    /**
     * @brief Set the parameters of the integer type
     * @param parameters
     * @return
     */
    virtual bool setParametersInt(const std::vector<int> & parameters);

    /**
     * @brief Set the parameters of the double type
     * @param parameters
     * @return
     */
    virtual bool setParametersDouble(const std::vector<double> & parameters);

    /**
     * @brief Get Auxiliary Data of the integer type
     * @param auxiliaryData
     * @return
     */
    virtual bool getAuxiliaryDataInt(std::vector<int> & auxiliaryData);

    /**
     * @brief Get Auxiliary Data of the double type
     * @param auxiliaryData
     * @return
     */
    virtual bool getAuxiliaryDataDouble(std::vector<double> & auxiliaryData);

    virtual int getNlevel();



    // ================================================== pure virtual funcions =================================================

    /**
     * @brief Update the bound of optimization variables
     * @param lb    Lower bound of optimization variables
     * @param ub    Upper bound of optimization variables
     * @return
     */
    virtual bool updateBound(const Eigen::VectorXd & lb, const Eigen::VectorXd & ub) = 0;

    /**
     * @brief update RobotDynamics
     * @details Update the robot member, perform some necessary dynamic calculations
     *          to prepare for the update of tasks and constraints.
     * @param q     Generalized position
     * @param qDot  Generalized velocity
     * @return Whether the function is executed successfully
     *  @retval true     execution succeed
     *  @retval false    execution failed
     * @note You must call this function before update Task and Constraint
     */
    virtual bool updateRobotDynamics(const Eigen::VectorXd & q, const Eigen::VectorXd & qDot) = 0;

    /**
     * @brief Solve the WBC problem using any kind of algorithm
     * @return
     */
    virtual bool wbcSolve() = 0;

    /**
     * @brief Get optimized variable results
     * @param resultOpt Output
     * @return
     */
    virtual bool getResultOpt(Eigen::VectorXd & resultOpt) = 0;


protected:

    RobotDynamics * robot = nullptr;                                ///< control object of WBC

    std::unordered_map<std::string, Task *> tasks;                  ///< dictionary of tasks
    std::unordered_map<std::string, Constraint *> constraints;      ///< dictionary of constraints

    std::vector<std::vector<std::string>> priorityTaskNames;        ///< Two-dimensional table of task priority
    std::vector<std::vector<std::string>> priorityConstraintNames;  ///< Two-dimensional table of constraints priority

    int nV{0};              ///< dimension of Variables
    int nO{0};              ///< dimension of Tasks/Objects/Costs
    int nC{0};              ///< dimension of Constraints

    bool nOChange{false};
    bool nVChange{false};
    bool nCChange{false};
};

} // namespace AGIROBOT

#endif // AGIROBOT_WBC_H
