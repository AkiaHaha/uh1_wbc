#ifndef AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H
#define AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H

//#include "AGIROBOT/task.h"
#include "task.h"
#include "operation.h"

class BipedFloatingBaseDynamics: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFloatingBaseDynamics(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFloatingBaseDynamics() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedCentroidalMomentum : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedCentroidalMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedCentroidalMomentum() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedLinearMomentum : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedLinearMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedLinearMomentum() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedAngularMomentum : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedAngularMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedAngularMomentum() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedPelvisPosition: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedPelvisPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedPelvisPosition() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedPelvisRpy: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedPelvisRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedPelvisRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedPelvisXyz: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedPelvisXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedPelvisXyz() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedTorsoRpy: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedTorsoXyz: public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoXyz() = default;
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

class QuadSoleForceChange : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    QuadSoleForceChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~QuadSoleForceChange() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedFootPosition : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootPosition() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};


class BipedFootForce : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootForce() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class BipedFootForceChange : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootForceChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootForceChange() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class GlobalVelocityLimitation : public AGIROBOT::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    GlobalVelocityLimitation(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~GlobalVelocityLimitation() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

#endif // AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H
