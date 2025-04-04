#ifndef AGIROBOT_EXAMPLE_ROBOTDYNAMICS_BIPED_H
#define AGIROBOT_EXAMPLE_ROBOTDYNAMICS_BIPED_H

#define TYPELEFTSOLE 0
#define TYPERIGHTSOLE 1
#define GRAVITY_CONST 9.81

//#include "AGIROBOT/robotDynamics.h"
#include "dynamics.h"
#include "operation.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/**
 * @brief The RobotDynamics class
 */

class RobotDynamics : public AGIROBOT::RobotDynamics{
public:

    /**
     * @brief Constructor function
     */
    RobotDynamics();
    ~RobotDynamics();

    /**
     * @brief setJntStates set value of joint states and some flags
     * @param q generalized position
     * @param qdot generalized velocity
     * @return
     */
    bool setJntStates(const Eigen::VectorXd & q, const Eigen::VectorXd & qdot) override;

    /**
     * @brief calcWbcDependence
     * @note Must call this function after setJntStates()
     */
    bool calcWbcDependence() override;

    /**
     * @brief getTotalMass
     *  fetch: totalMass
     * @note Must call this function after setJntStates()
     */
    double getTotalMass() override;

    /**
     * @brief estPelvisPosVelInWorld
     *          estimate the pelvis state in world frame (origin at the stance sole)
     * @param jointPos  generalized joint position
     * @param jointVel  generalized joint velocity
     * @param footType  stance sole label
     * @return the [posXYZ, velXYZ] of pelvis in Vector(6,1)
     */
    VectorNd estPelvisPosVelInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& footType);

    /**
     * @brief estBodyTwistInWorld
     *          estimate the swing foot state in world frame (origin at the stance sole)
     * @param jointPos  generalized joint position
     * @param jointVel  generalized joint velocity
     * @param footType  stance sole label
     * @return the motion twist as sequence: [posRPY, posXYZ, velRPY, velXYZ] 
     */
    VectorNd estBodyTwistInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& pointType);

    bool estComPosVel(Eigen::Vector3d &comPos, Eigen::Vector3d &comVel);


    VectorNd estRootXyzRpy(const Eigen::VectorXd & q);///<Danny@240526

    VectorNd estRootXyzRpyDot(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot);///<Danny@240621

    VectorNd estBodyPosInWorldAkia(const VectorNd& jointPos, const VectorNd& jointVel, const unsigned int& bodyId);//<Danny@240526

private:

    bool isPosVelUpdated{false};

    unsigned int idPelvis, idTorso;
    unsigned int idLeftSole, idRightSole;
    unsigned int idLeftFixedAnkle, idRightFixedAnkle;
    unsigned int idRightSoleGround, idLeftSoleGround;
    unsigned int idLeftArmEnd, idRightArmEnd;
    unsigned int idLeftLegLink[5],idLeftArmLink[5];
    unsigned int idRightLegLink[5],idRightArmLink[5];
    double massAll;

    MatrixNd centroidAG;                        // nJF*nJG, Centroidal Momentum Matrix (CMM) in G CS
    MatrixNd centroidAICS;                      // nJF*nJG, Centroidal Momentum Matrix (CMM) in I CS
    SpatialVector centroidAGDotQDot;            // nJF*1, Centroidal momentum bias force in G CS
    SpatialVector centroidAICSDotQDot;          // nJF*1, Centroidal momentum bias force in I CS
    SpatialVector centroidMomentum;             // nJF*1, Centroidal momentum  in G CS
    SpatialVector centroidMomentumICS;          // nJF*1, Centroidal momentum  in I CS
    SpatialMatrix compositeInertia;             // nJF*nJF, Composite-rigid-body inertia in G CS
    SpatialMatrix spatialTransformG2ICS;        // nJF*nJF, Transform matrix from GCS to ICS

    Vector3d comPos2Pelvis;
    Vector3d comPos2World;
    Vector3d comVel2World;
    Vector3d linearMomentum;
    Vector3d angularMomentum;

    MatrixNd pelvisJacob;
    VectorNd pelvisJDotQDot;

    MatrixNd torsoJacob;
    VectorNd torsoJDotQDot;

    MatrixNd leftArmSoleJacob;
    MatrixNd rightArmSoleJacob;
    MatrixNd leftLegSoleJacob;
    MatrixNd rightLegSoleJacob;
    MatrixNd dualSoleJacob;
    MatrixNd quadSoleJacob;

    VectorNd leftArmSoleJDotQDot;
    VectorNd rightArmSoleJDotQDot;
    VectorNd leftLegSoleJDotQDot;
    VectorNd rightLegSoleJDotQDot;
    VectorNd dualSoleJDotQDot;
    VectorNd quadSoleJDotQDot;


    bool updateKinematicsPosVel();
    bool updateKinematicsAcc();

    bool calcPelvisJacob();
    bool calcPelvisJDotQDot();
    bool calcTorsoJacob();
    bool calcTorsoJDotQDot();
    bool calcSoleJacob();
    bool calcSoleJDotQDot();

    bool calcMassMatrixCRBA();
    bool calcNonlinearBiasRNEA();
    bool calcRigidBodyDynamicsDescriptors();
    bool calcCentroidalDynamicsDescriptors();
    bool calcSoleTask();
    bool calcPelvisTask();

    bool calcCOMPosVel();
    bool calcCOMState();
    bool calcCentroidalMomentum();

    Vector3d rotaitonMatrix2EulerRPY(const Matrix3d& rotMat);
    Vector3d angleDot2EulerDot(const Vector3d& angleDot, const Vector3d& rpy);
    Matrix3d skew(const Vector3d& omg);
};

#endif //AGIROBOT_EXAMPLE_ROBOTDYNAMICS_BIPED_H
