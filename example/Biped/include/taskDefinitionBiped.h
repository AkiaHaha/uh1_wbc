/**
 *	This file is part of TAICHI.
 *
 *	TAICHI -- Task Arrangement In Control HIerarchy.
 *	Copyright (C) 2015-2021 Beijing Research Institute of UBTECH Robotics.
 *	All rights reserved.
 *
 *	Licensed under the Apache License 2.0. See LICENSE for more details.
 */

/**
 * @file taskDefinitionBiped.h
 * @brief Declaration of the subclasses of the abstract class Task
 * @author Yijie Guo, Jiajun Wang
 * @date 2021
 * @version alpha
 */

#ifndef TAICHI_EXAMPLE_TASKDEFINITION_BIPED_H
#define TAICHI_EXAMPLE_TASKDEFINITION_BIPED_H

//#include "taichi/task.h"
#include "task.h"

class BipedFloatingBaseDynamics: public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFloatingBaseDynamics(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFloatingBaseDynamics() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedCentroidalMomentum : public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedCentroidalMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedCentroidalMomentum() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedLinearMomentum : public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedLinearMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedLinearMomentum() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedAngularMomentum : public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedAngularMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedAngularMomentum() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedTorsoPosition: public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoPosition() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedTorsoPosRpy: public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoPosRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoPosRpy() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedTorsoPosXyz: public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoPosXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoPosXyz() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedFootPosition : public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootPosition() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};


class BipedFootForce : public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootForce() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedFootForceChange : public TAICHI::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootForceChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootForceChange() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedArmPosition : public TAICHI::Task{//Daniel 24.5.28
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedArmPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedArmPosition() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

#endif // TAICHI_EXAMPLE_TASKDEFINITION_BIPED_H
