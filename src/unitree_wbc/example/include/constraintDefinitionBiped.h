#ifndef TAICHI_EXAMPLE_CONSTRAINTDEFINITION_BIPED_H
#define TAICHI_EXAMPLE_CONSTRAINTDEFINITION_BIPED_H

#include "constraint.h"
//#include "taichi/constraint.h"


class BipedDynamicConsistency : public TAICHI::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedDynamicConsistency(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~BipedDynamicConsistency() = default;
    bool update(const TAICHI::RobotDynamics &robot) override;
};

class BipedFrictionCone : public TAICHI::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedFrictionCone(const std::string & constrName, int constrDim, int varDim);
    ~BipedFrictionCone() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of muStaticFriction and the second item is the value of myInfinity
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const TAICHI::RobotDynamics &robot) override;
private:
    double muStaticFriction{0.6};
    double myInfinity{1e9};
    Eigen::MatrixXd fricMat;
};

class BipedCenterOfPressure : public TAICHI::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedCenterOfPressure(const std::string & constrName, int constrDim, int varDim);
    ~BipedCenterOfPressure() = default;
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
    bool update(const TAICHI::RobotDynamics &robot) override;
private:
    double sole2Front{0.1};
    double sole2Back{0.1};
    double sole2Left{0.1};
    double sole2Right{ 0.1};
    double copFactor{0.9};
    double myInfinity{1e9};
    Eigen::MatrixXd copMat;
};

class BipedJointTorqueSaturation : public TAICHI::Constraint{
public:
    /**
     * @brief Constructor
     * @param constrName The unique identification of the Constraint: name
     * @param constrDim The dimension of Constraint
     * @param varDim The DoF of variables in the WBC problem
     */
    BipedJointTorqueSaturation(const std::string & constrName, int constrDim, int varDim) : Constraint(constrName, constrDim, varDim){}
    ~BipedJointTorqueSaturation() = default;
    /**
     * @brief setParameter
     * @param params
     *  a std::vector, whose first item is the value of jointTauLimit
     * @return
     */
    bool setParameter(const std::vector<double> & params) override;
    bool update(const TAICHI::RobotDynamics &robot) override;
private:
    double jointTauLimit{100.};
};

#endif //TAICHI_EXAMPLE_CONSTRAINTDEFINITION_BIPED_H
