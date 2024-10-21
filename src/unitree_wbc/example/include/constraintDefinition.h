#ifndef AGIROBOT_EXAMPLE_CONSTRAINTDEFINITION_BIPED_H
#define AGIROBOT_EXAMPLE_CONSTRAINTDEFINITION_BIPED_H

#include "constraint.h"
#include "operation.h"
//#include "AGIROBOT/constraint.h"


class DynamicConsistency : public AGIROBOT::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    DynamicConsistency(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~DynamicConsistency() = default;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
};

class FrictionCone : public AGIROBOT::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    FrictionCone(const std::string & constrName, int constrDim, int varDim);
    ~FrictionCone() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of muStaticFriction and the second item is the value of myInfinity
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
private:
    double muStaticFriction{0.6};
    double myInfinity{1e9};
    Eigen::MatrixXd fricMat;
};

class CenterOfPressure : public AGIROBOT::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    CenterOfPressure(const std::string & constrName, int constrDim, int varDim);
    ~CenterOfPressure() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of sole2Front,
     *  the second item is the value of sole2Back,
     *  the third item is the value of sole2Left,
     *  the fourth item is the value of sole2Right,
     *  the fifth item is the value of copFactor,
     *  the sixth item is the value of myInfinity
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
private:
    double sole2Front{0.1};
    double sole2Back{0.1};
    double sole2Left{0.1};
    double sole2Right{ 0.1};
    double copFactor{0.9};
    double myInfinity{1e9};
    Eigen::MatrixXd copMat;
};

class JointTorqueSaturation : public AGIROBOT::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    JointTorqueSaturation(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~JointTorqueSaturation() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of jointTauLimit
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const AGIROBOT::RobotDynamics &robot) override;
private:
    double jointTauLimit{100.};
};

#endif //AGIROBOT_EXAMPLE_CONSTRAINTDEFINITION_BIPED_H