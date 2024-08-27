#ifndef HUMANOID_EXAMPLE_TASKDEFINITION_BIPED_H
#define HUMANOID_EXAMPLE_TASKDEFINITION_BIPED_H

//#include "taichi/task.h"
#include "task.h"
#include "operation.h"

class BipedFloatingBaseDynamics: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFloatingBaseDynamics(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFloatingBaseDynamics() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedCentroidalMomentum : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedCentroidalMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedCentroidalMomentum() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedLinearMomentum : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedLinearMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedLinearMomentum() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedAngularMomentum : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedAngularMomentum(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedAngularMomentum() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedTorsoPosition: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoPosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoPosition() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedTorsoRpy: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoRpy() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedTorsoXyz: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedTorsoXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedTorsoXyz() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedComXyz: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedComXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedComXyz() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedComRpy: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedComRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedComRpy() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedUpTorsoRpy: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedUpTorsoRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedUpTorsoRpy() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};


class BipedUpTorsoXyz: public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedUpTorsoXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedUpTorsoXyz() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class QuadSolePosition : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    QuadSolePosition(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~QuadSolePosition() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class QuadSoleForce : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    QuadSoleForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~QuadSoleForce() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class QuadSoleForceChange : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    QuadSoleForceChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~QuadSoleForceChange() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedFootPose : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootPose(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootPose() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedArmPose : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedArmPose(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedArmPose() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedArmPoseStatic : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedArmPoseStatic(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedArmPoseStatic() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedFootForce : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootForce() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedArmForce : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedArmForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedArmForce() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class BipedFootForceChange : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFootForceChange(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~BipedFootForceChange() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

class GlobalVelocityLimitation : public HUMANOID::Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    GlobalVelocityLimitation(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~GlobalVelocityLimitation() = default;
    bool update(const HUMANOID::RobotDynamics &robot) override;
};

#endif // HUMANOID_EXAMPLE_TASKDEFINITION_BIPED_H
