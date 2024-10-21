#ifndef AGIROBOT_EXAMPLE_ROBOTDYNAMICS_BIPED_H
#define AGIROBOT_EXAMPLE_ROBOTDYNAMICS_BIPED_H

#define TYPELEFTSOLE 0
#define TYPERIGHTSOLE 1
#define GRAVITY_CONST 9.81

//#include "AGIROBOT/robotDynamics.h"
#include "robotDynamics.h"
#include "operation.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

/**
 * @brief The RobotDynamicsBiped class
 */

class RobotDynamicsBiped : public AGIROBOT::RobotDynamics{
public:

    /**
     * @brief Constructor function
     */
    RobotDynamicsBiped();
    ~RobotDynamicsBiped();

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
     * @brief estWaistPosVelInWorld
     *          estimate the waist state in world frame (origin at the stance sole)
     * @param jointPos  generalized joint position
     * @param jointVel  generalized joint velocity
     * @param footType  stance sole label
     * @return the [posXYZ, velXYZ] of waist in Vector(6,1)
     */
    VectorNd estWaistPosVelInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& footType);

    /**
     * @brief estFootArmPosVelInWorld
     *          estimate the swing foot state in world frame (origin at the stance sole)
     * @param jointPos  generalized joint position
     * @param jointVel  generalized joint velocity
     * @param footType  stance sole label
     * @return the [posRPY, posXYZ, velRPY, velXYZ] of swing foot in Vector(12,1)
     */
    VectorNd estFootArmPosVelInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& footType);

    VectorNd getRootXyzRpy(const Eigen::VectorXd & q);//Daniel 5.26

    VectorNd getRootXyzRpyDot(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot);//Daniel 5.27

    VectorNd estBodyPosInWorldAkia(const VectorNd& jointPos, const VectorNd& jointVel, const unsigned int& bodyId);//Daniel 5.27

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

    // -------- for Centroidal Dynamics -------
    MatrixNd centroidAG;                        // nJF*nJG, Centroidal Momentum Matrix (CMM) in G CS
    MatrixNd centroidAICS;                      // nJF*nJG, Centroidal Momentum Matrix (CMM) in I CS
    SpatialVector centroidAGDotQDot;            // nJF*1, Centroidal momentum bias force in G CS
    SpatialVector centroidAICSDotQDot;          // nJF*1, Centroidal momentum bias force in I CS
    SpatialVector centroidMomentum;             // nJF*1, Centroidal momentum  in G CS
    SpatialVector centroidMomentumICS;          // nJF*1, Centroidal momentum  in I CS
    SpatialMatrix compositeInertia;             // nJF*nJF, Composite-rigid-body inertia in G CS
    SpatialMatrix spatialTransformG2ICS;        // nJF*nJF, Transform matrix from GCS to ICS
    // -------- for Centroidal Dynamics -------

    Vector3d comPos2Waist;
    Vector3d comPos2World;
    Vector3d comVel2World;
    Vector3d linearMomentum;
    Vector3d angularMomentum;

    MatrixNd waistJacob;
    VectorNd waistJDotQDot;

    MatrixNd trunkJacob;
    VectorNd trunkJDotQDot;

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

    bool calcWaistJacob();
    bool calcWaistJDotQDot();
    bool calcTrunkJacob();
    bool calcTrunkJDotQDot();
    bool calcSoleJacob();
    bool calcSoleJDotQDot();

    bool calcMassMatrixCRBA();
    bool calcNonlinearBiasRNEA();
    bool calcRigidBodyDynamicsDescriptors();
    bool calcCentroidalDynamicsDescriptors();
    bool calcSoleTask();
    bool calcWaistTask();

    bool calcCOMPosVel();
    bool calcCOMState();
    bool calcCentroidalMomentum();

    // Math function
    Vector3d rotaitonMatrix2EulerRPY(const Matrix3d& rotMat);
    Vector3d angleDot2EulerDot(const Vector3d& angleDot, const Vector3d& rpy);
    Matrix3d skew(const Vector3d& omg);
};

#endif //AGIROBOT_EXAMPLE_ROBOTDYNAMICS_BIPED_H
