#ifndef AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H
#define AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H

//#include "AGIROBOT/task.h"
#include "task.h"
#include "operation.h"

class FloatingBaseDynamics: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    FloatingBaseDynamics(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~FloatingBaseDynamics() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class CentroidalMomentum : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    CentroidalMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~CentroidalMomentum() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class LinearMomentum : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    LinearMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~LinearMomentum() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class AngularMomentum : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    AngularMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~AngularMomentum() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class PelvisPosition: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    PelvisPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~PelvisPosition() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class PelvisPosRpy: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    PelvisPosRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~PelvisPosRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class PelvisPosXyz: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    PelvisPosXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~PelvisPosXyz() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class TorsoPosRpy: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    TorsoPosRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~TorsoPosRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class TorsoPosXyz: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    TorsoPosXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~TorsoPosXyz() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class QuadSolePosition : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    QuadSolePosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~QuadSolePosition() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};


class QuadSoleForce : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    QuadSoleForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~QuadSoleForce() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};


class FootPosition : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    FootPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~FootPosition() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};


class FootForce : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    FootForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~FootForce() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class FootForceChange : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    FootForceChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~FootForceChange() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class GVLimitation : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    GVLimitation(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~GVLimitation() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

#endif // AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H
