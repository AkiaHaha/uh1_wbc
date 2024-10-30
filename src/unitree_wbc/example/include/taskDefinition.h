#ifndef AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H
#define AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H

//#include "AGIROBOT/task.h"
#include "task.h"
#include "operation.h"

class Task{

public:
    Task(const std::string & taskName, int taskDim, int varDim);
    virtual ~Task() = default;

    std::string name;      
    int priority{0};    
    int dim{0};           
    int varDof{0};         
    Eigen::MatrixXd taskMatA;    
    Eigen::VectorXd taskVecB;   
    Eigen::VectorXd wei;     
    Eigen::VectorXd ref;   

    virtual bool setParameter(const std::vector<double> & params);
    virtual bool update(const RobotDynamics & robot) = 0;
    bool updateRefence(const Eigen::VectorXd & newRef);
    bool updateWeight(const Eigen::VectorXd & newWei);

};


class FloatingBaseDynamics: public Task{
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

class CentroidalMomentum : public Task{
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

class LinearMomentum : public Task{
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

class AngularMomentum : public Task{
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

class PelvisPosition: public Task{
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

class PelvisRpy: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    PelvisRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~PelvisRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class PelvisXyz: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    PelvisXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~PelvisXyz() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class TorsoRpy: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    TorsoRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~TorsoRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class TorsoXyz: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    TorsoXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~TorsoXyz() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class FootXyz: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    FootXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~FootXyz() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
}

class FootRpy: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    FootRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~FootRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
}

class ArmXyz: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    ArmXyz(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}   
    ~ArmXyz() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
}

class ArmRpy: public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     */
    ArmRpy(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}   
    ~ArmRpy() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
}

class FootForce : public Task{
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

class ArmForce : public Task{
public:
    /**
     * @brief Constructor
     * @param taskName The unique identification of the Task: name
     * @param taskDim The dimension of Task
     * @param varDim The DoF of variables in the WBC problem
     *  */
    ArmForce(const std::string & taskName, int taskDim, int varDim) : Task(taskName, taskDim, varDim){}
    ~ArmForce() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
}

class GVLimitation : public Task{
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

class QuadSolePosition : public Task{
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


class QuadSoleForce : public Task{
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

#endif // AGIROBOT_EXAMPLE_TASKDEFINITION_BIPED_H
