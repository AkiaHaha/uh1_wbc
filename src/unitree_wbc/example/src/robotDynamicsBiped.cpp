#include "robotDynamicsBiped.h"
using namespace std;


// ======================================== Public Functions ==========================================

RobotDynamicsBiped::RobotDynamicsBiped() {
    // ------------------------- public members of Base class -------------------
    NJG = 16;
    NJF = 6;  
    NJJ = 10; 
    NJA = 10; 
    NFC = 12;  
 
    jntPositions = VectorNd :: Zero(NJG);
    jntVelocities = VectorNd :: Zero(NJG);

    inertiaMat = MatrixNd :: Zero(NJG, NJG);
    invInertiaMat = MatrixNd :: Zero(NJG, NJG);
    nonlinearBias = VectorNd :: Zero(NJG);
    coriolisBias = VectorNd::Zero(NJG);
    gravityBias = VectorNd::Zero(NJG);

    selMatFloatingBase = MatrixNd :: Zero(NJF, NJG);
    selMatFloatingBase.leftCols(NJF) = MatrixNd::Identity(NJF, NJF);

    selMatActuated = MatrixNd :: Zero(NJA, NJG);
    selMatActuated.rightCols(NJA) = MatrixNd::Identity(NJA, NJA);

    selMatNonFloatingBase = MatrixNd :: Zero(NJJ, NJG);

    centroidalMomentumMatrix = MatrixNd :: Zero(NJF, NJG);
    centroidalMomentumBiased = VectorNd :: Zero(NJF);

    floatBaseJacoTc.J = MatrixNd :: Zero(NJF, NJG);
    floatBaseJacoTc.JdotQdot = VectorNd :: Zero(NJF);

    contactJacoTc.J = MatrixNd :: Zero(NFC, NJG);
    contactJacoTc.JdotQdot = VectorNd :: Zero(NFC); 

    eqCstrMatTau = MatrixNd :: Zero(NJA, NJG+NFC);
    eqCstrMatTauBias = VectorNd :: Zero(NJA);

    model = new RigidBodyDynamics::Model();
    model->gravity = Vector3d (0., 0., -GRAVITY_CONST);
    // ------------------------- public members of Base class -------------------

    // ------------------------- private members of Derived class -------------------
    Body floatingWaistLink, fixedAnkleLink, fixedSoleLink;
    Body leftLegLink[5], rightLegLink[5];
    Joint floatingWaistJoint, fixedSoleJoint, fixedAnkleJoint;

    // ------------------------------------------------------ Biped parameters------------------------------------------------------
    floatingWaistLink = Body(6.0, Vector3d(0.0, 0.0, 0.05),
                            Matrix3d(0.05, 0.0,  0.0
                                    ,0.0,  0.05, 0.0
                                    ,0.0,  0.0,  0.05));//pelvis
    leftLegLink[0] = Body(1.0, Vector3d(0.0, 0.0, 0.0),
                            Matrix3d(0.002, 0.0,   0.0
                                    ,0.0,   0.002, 0.0
                                    ,0.0,   0.0,   0.002));//yaw
    leftLegLink[1] = Body(1.0, Vector3d(0.0, 0.05, 0.01),
                            Matrix3d(0.005, 0.0,   0.0
                                    ,0.0,   0.005, 0.0
                                    ,0.0,   0.0,   0.005));//row
    leftLegLink[2] = Body(2.0, Vector3d(0.0,  0.0, -0.2),
                            Matrix3d(0.04, 0.0,  0.0
                                    ,0.0,  0.04, 0.0
                                    ,0.0,  0.0,  0.01));//pitch
    leftLegLink[3] = Body(2.0, Vector3d(0.0,  0.0, -0.2),
                            Matrix3d(0.04, 0.0,  0.0
                                    ,0.0,  0.04, 0.0
                                    ,0.0,  0.0,  0.01));//knee
    leftLegLink[4] = Body(0.5,  Vector3d(0.0, 0.0, -0.03),
                            Matrix3d(0.0003, 0.0,   0.0
                                    ,0.0,    0.001, 0.0
                                    ,0.0,    0.0,   0.001));//ankle
    rightLegLink[0] = Body(1.0, Vector3d(0.0, 0.0, 0.0),
                            Matrix3d(0.002, 0.0,   0.0
                                    ,0.0,   0.002, 0.0
                                    ,0.0,   0.0,   0.002));
    rightLegLink[1] = Body(1.0, Vector3d(0.0, -0.05, 0.01),
                            Matrix3d(0.005, 0.0,   0.0
                                    ,0.0,   0.005, 0.0
                                    ,0.0,   0.0,   0.005));
    rightLegLink[2] = Body(2.0, Vector3d(0.0,  0.0, -0.2),
                            Matrix3d(0.04, 0.0,  0.0
                                    ,0.0,  0.04, 0.0
                                    ,0.0,  0.0,  0.01));
    rightLegLink[3] = Body(2.0, Vector3d(0.0,  0.0, -0.2),
                            Matrix3d(0.04, 0.0,  0.0
                                    ,0.0,  0.04, 0.0
                                    ,0.0,  0.0,  0.01));
    rightLegLink[4] = Body(0.5,  Vector3d(0.0, 0.0, -0.03),
                            Matrix3d(0.0003, 0.0,   0.0
                                    ,0.0,    0.001, 0.0
                                    ,0.0,    0.0,   0.001));
    fixedAnkleLink = Body(0.0, Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
    fixedSoleLink = Body(0.0, Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
    // ------------------------------------------------------Biped parameters------------------------------------------------------

    // ------------------------------------------------------Biped RBDL Model------------------------------------------------------
    floatingWaistJoint = Joint(SpatialVector(0.,0.,0., 1.,0.,0.),  //                ^ Z
                                SpatialVector(0.,0.,0., 0.,1.,0.), //                |
                                SpatialVector(0.,0.,0., 0.,0.,1.), //                |___Pelvis, Torso (1.1) 
                                SpatialVector(1.,0.,0., 0.,0.,0.), //                |
                                SpatialVector(0.,1.,0., 0.,0.,0.), //                |
                                SpatialVector(0.,0.,1., 0.,0.,0.));//                ------------> Y
    Joint joint_Rx = Joint(SpatialVector(1.,0.,0., 0.,0.,0.));     //               /
    Joint joint_Ry = Joint(SpatialVector(0.,1.,0., 0.,0.,0.));     //              / 
    Joint joint_Rz = Joint(SpatialVector(0.,0.,1., 0.,0.,0.));     //           X /
    fixedSoleJoint = Joint(JointTypeFixed);

    //Pelvis relates the world coordinate system
    idPelvis = model->AddBody(0, Xtrans(Vector3d(0., 0. , 0.)), floatingWaistJoint, floatingWaistLink,"pelvis");

    idLeftLegLink[0] = model->AddBody(idPelvis, Xtrans(Vector3d(0, 0.0875, -0.1742)), joint_Rz, leftLegLink[0], "lhy");
    idLeftLegLink[1] = model->AppendBody(Xtrans(Vector3d(0.039468, 0, 0)), joint_Rx, leftLegLink[1], "lhr");
    idLeftLegLink[2] = model->AppendBody(Xtrans(Vector3d(0, 0.11536, 0)), joint_Ry, leftLegLink[2], "lhp");
    idLeftLegLink[3] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, leftLegLink[3], "lk");
    idLeftLegLink[4] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, fixedAnkleLink, "la");
    idLeftSole = model->AppendBody(Xtrans(Vector3d(0.0556, 0, -0.05)), fixedSoleJoint, leftLegLink[4], "lsole");//Daniel5.1
    idLeftSoleGround = model->AppendBody(Xtrans(Vector3d(0, 0, -0.012)), fixedSoleJoint, fixedSoleLink, "lsoleg");

    idRightLegLink[0] = model->AddBody(idPelvis, Xtrans(Vector3d(0, -0.0875, -0.1742)), joint_Rz, rightLegLink[0], "rhy");
    idRightLegLink[1] = model->AppendBody(Xtrans(Vector3d(0.039468, 0, 0)), joint_Rx, rightLegLink[1], "rhr");
    idRightLegLink[2] = model->AppendBody(Xtrans(Vector3d(0, -0.11536, 0)), joint_Ry, rightLegLink[2], "rhp");
    idRightLegLink[3] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, rightLegLink[3], "rk");
    idRightLegLink[4] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, fixedAnkleLink, "ra");
    idRightSole = model->AppendBody(Xtrans(Vector3d(0.0556, 0, -0.05)), fixedSoleJoint, rightLegLink[4], "rsole");//5.31
    idRightSoleGround = model->AppendBody(Xtrans(Vector3d(0, 0, -0.012)), fixedSoleJoint, fixedSoleLink, "rsoleg");

    massAll = floatingWaistLink.mMass;
    for (int i = 0; i < 5; i++){massAll += leftLegLink[i].mMass + rightLegLink[i].mMass;}

    // -------- for Centroidal Dynamics ------- //
    centroidAG  = MatrixNd::Zero(NJF,NJG);
    centroidAICS = MatrixNd::Zero(NJF, NJG);
    compositeInertia = SpatialMatrix::Zero(NJF,NJF);
    centroidAGDotQDot = SpatialVector::Zero(NJF);
    centroidAICSDotQDot = SpatialVector::Zero(NJF);
    centroidMomentum = SpatialVector::Zero(NJF);
    centroidMomentumICS = SpatialVector::Zero(NJF);
    spatialTransformG2ICS = SpatialMatrix::Zero(NJF,NJF);
    // -------- for Centroidal Dynamics ------- //

    comPos2Waist = Vector3d::Zero();
    comPos2World = Vector3d::Zero();
    comVel2World = Vector3d::Zero();
    linearMomentum = Vector3d::Zero();
    angularMomentum = Vector3d::Zero();
  
    waistJacob = MatrixNd::Zero(NJF, NJG);
    waistJDotQDot = VectorNd::Zero(NJF);

    leftArmSoleJacob = MatrixNd::Zero(NJF, NJG);
    rightArmSoleJacob = MatrixNd::Zero(NJF, NJG);
    leftLegSoleJacob = MatrixNd::Zero(NJF, NJG);
    rightLegSoleJacob = MatrixNd::Zero(NJF, NJG);
    dualSoleJacob = MatrixNd::Zero(2*NJF, NJG);
    quadSoleJacob = MatrixNd::Zero(4*NJF, NJG);

    leftArmSoleJDotQDot = VectorNd::Zero(NJF);
    rightArmSoleJDotQDot = VectorNd::Zero(NJF);
    leftLegSoleJDotQDot = VectorNd::Zero(NJF);
    rightLegSoleJDotQDot = VectorNd::Zero(NJF);
    dualSoleJDotQDot = VectorNd::Zero(2*NJF);
    quadSoleJDotQDot = VectorNd::Zero(4*NJF);
    // ------------------------- private members of Derived class -------------------
}

RobotDynamicsBiped::~RobotDynamicsBiped(){
    delete model;
    model = nullptr;
}

bool RobotDynamicsBiped::setJntStates(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot){
    TAICHI::RobotDynamics::setJntStates(q, qdot);
    isPosVelUpdated = false;
    calcWbcDependenceDone = false;
    return true;
}

bool RobotDynamicsBiped::calcWbcDependence(){
    if (!isPosVelUpdated){
        updateKinematicsPosVel();
    }
    calcRigidBodyDynamicsDescriptors();
    updateKinematicsAcc();
    calcSoleTask();
    calcWaistTask();
    calcWbcDependenceDone = true;
    // ------------------------- public members of Base class -------------------
    contactJacoTc.J = dualSoleJacob;
    contactJacoTc.JdotQdot = dualSoleJDotQDot;
    floatBaseJacoTc.J = waistJacob;
    floatBaseJacoTc.JdotQdot = waistJDotQDot;
    eqCstrMatTau << selMatActuated * inertiaMat,
                -selMatActuated * leftLegSoleJacob.transpose(),
                -selMatActuated * rightLegSoleJacob.transpose();
    eqCstrMatTauBias = selMatActuated * nonlinearBias;
    // ------------------------- public members of Base class -------------------
    return true;
}

double RobotDynamicsBiped::getTotalMass(){
    return massAll;
}
// ======================================== Public Functions ==========================================

VectorNd RobotDynamicsBiped::estWaistPosVelInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& footType) {
    VectorNd qTemp = VectorNd::Zero(NJG);
    VectorNd qDotTemp = VectorNd::Zero(NJG);
    Vector3d sole2WaistPos = Vector3d::Zero();
    Vector3d sole2WaistVel = Vector3d::Zero();
    VectorNd posVelRes = VectorNd::Zero(6,1);
    qTemp = jointPos;
    qDotTemp = jointVel;
    UpdateKinematicsCustom(*model, & qTemp, & qDotTemp, NULL);
    switch (footType)
    {
        case TYPELEFTSOLE: // Left Sole
            sole2WaistPos = CalcBodyToBaseCoordinates(*model, qTemp, idLeftSoleGround, Math::Vector3d::Zero(), false);
            sole2WaistVel = CalcPointVelocity(*model, qTemp, qDotTemp, idLeftSoleGround, Math::Vector3d::Zero(), false);
            break;
        case TYPERIGHTSOLE: // Right Sole
            sole2WaistPos = CalcBodyToBaseCoordinates(*model, qTemp, idRightSoleGround, Math::Vector3d::Zero(), false);
            sole2WaistVel = CalcPointVelocity(*model, qTemp, qDotTemp, idRightSoleGround, Math::Vector3d::Zero(), false);
            break;    
        default:
            break;
    }
    posVelRes.head(3) = qTemp.head(3)-sole2WaistPos;
    posVelRes.tail(3) = qDotTemp.head(3)-sole2WaistVel;
    calcWbcDependenceDone = false;
    return posVelRes;
}

VectorNd RobotDynamicsBiped::estBodyPosInWorldAkia(const VectorNd& jointPos, const VectorNd& jointVel, const unsigned int& bodyId) {//Daniel 5.27
    VectorNd qTemp = VectorNd::Zero(NJG);                                                                                           //0: pelvis; 1:LS; 2:RS; 3:LF; 4:RF
    VectorNd qDotTemp = VectorNd::Zero(NJG);
    Vector3d bodyPos = Vector3d::Zero(3);

    qTemp = jointPos;
    qDotTemp = jointVel;
    UpdateKinematicsCustom(*model, & qTemp, & qDotTemp, NULL);

    switch (bodyId)
    {
        case 0:
            bodyPos = CalcBodyToBaseCoordinates(*model, qTemp, idPelvis, Math::Vector3d::Zero(), false);
            break;
        default:
            break;
    }
    calcWbcDependenceDone = false;
    return bodyPos;
}

VectorNd RobotDynamicsBiped::estFootArmPosVelInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& footType) {
    VectorNd qTemp = VectorNd::Zero(NJG);
    VectorNd qDotTemp = VectorNd::Zero(NJG);
    Vector3d sole2WorldPos = Vector3d::Zero();
    Vector3d sole2WorldRPY = Vector3d::Zero();
    VectorNd sole2WorldVel = VectorNd::Zero(6);
    Matrix3d sole2WorldMatR = Matrix3d::Identity();
    VectorNd posVelRes = VectorNd::Zero(12,1);
    qTemp = jointPos;
    qDotTemp = jointVel;
    UpdateKinematicsCustom(*model, & qTemp, & qDotTemp, NULL);
    switch (footType)
    {
        case 1: // Left Sole
            sole2WorldPos = CalcBodyToBaseCoordinates(*model, qTemp, idLeftSoleGround, Math::Vector3d::Zero(), false);
            sole2WorldMatR = CalcBodyWorldOrientation(*model, qTemp, idLeftSoleGround, false);
            sole2WorldVel = CalcPointVelocity6D(*model, qTemp, qDotTemp, idLeftSoleGround, Math::Vector3d::Zero(), false);
            break;   
        case 2: // Right Sole 
            sole2WorldPos = CalcBodyToBaseCoordinates(*model, qTemp, idRightSoleGround, Math::Vector3d::Zero(), false);
            sole2WorldMatR = CalcBodyWorldOrientation(*model, qTemp, idRightSoleGround, false);
            sole2WorldVel = CalcPointVelocity6D(*model, qTemp, qDotTemp, idRightSoleGround, Math::Vector3d::Zero(), false);
            break;
        default:
            break;
    }
    sole2WorldRPY = -rotaitonMatrix2EulerRPY(sole2WorldMatR);
    posVelRes.head(3) = sole2WorldRPY;
    posVelRes.segment(3,3) = sole2WorldPos;
    posVelRes.segment(6,3) = angleDot2EulerDot(sole2WorldVel.head(3), sole2WorldRPY);
    posVelRes.tail(3) = sole2WorldVel.tail(3);
    calcWbcDependenceDone = false;
    return posVelRes;
}


VectorNd RobotDynamicsBiped::getRootXyzRpy(const Eigen::VectorXd & q){//Daniel 5.26
     Eigen::VectorXd rootPose(6);

    // 获取根节点的位置
    Vector3d position = CalcBodyToBaseCoordinates(*model, q, idPelvis, Vector3d(0.0, 0.0, 0.0), false);
    rootPose.segment<3>(0) = position;

    // 获取根节点的姿态
    Matrix3d orientation = CalcBodyWorldOrientation(*model, q, idPelvis, false);
    Eigen::Vector3d eulerAngles = orientation.eulerAngles(0, 1, 2); 
    rootPose.segment<3>(3) = eulerAngles;

    return rootPose;
}

VectorNd RobotDynamicsBiped::getRootXyzRpyDot(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot) {//Daniel 5.27
    Eigen::VectorXd rootPoseDot(6);

    // 获取根节点的速度
    VectorNd velocity = CalcPointVelocity6D(*model, q, qDot, idPelvis, Vector3d(0.0, 0.0, 0.0), false);

    // 分离线速度和角速度
    Vector3d linearVelocity = velocity.segment<3>(3);  // 后三个分量是线速度
    Vector3d angularVelocity = velocity.segment<3>(0); // 前三个分量是角速度

    rootPoseDot.segment<3>(0) = linearVelocity;
    rootPoseDot.segment<3>(3) = angularVelocity;

    return rootPoseDot;
}

// Update fuction
bool RobotDynamicsBiped::updateKinematicsPosVel() {
    UpdateKinematicsCustom(*model, &jntPositions, &jntVelocities, NULL);
    isPosVelUpdated = true;
    return true;
}
bool RobotDynamicsBiped::updateKinematicsAcc() {
    VectorNd qDDotZero = VectorNd::Zero(NJG);
    UpdateKinematicsCustom(*model, NULL, NULL, &qDDotZero);
    return true;
}

bool RobotDynamicsBiped::calcWaistJacob() {
    CalcPointJacobian6D(*model, jntPositions, idPelvis, Vector3d::Zero(), waistJacob, false);
    return true;
}

bool RobotDynamicsBiped::calcWaistJDotQDot() {
    waistJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idPelvis, Vector3d::Zero(), false);
    return true;
}

bool RobotDynamicsBiped::calcSoleJacob() {
    CalcPointJacobian6D(*model, jntPositions, idLeftSoleGround, Vector3d::Zero(), leftLegSoleJacob, false);
    CalcPointJacobian6D(*model, jntPositions, idRightSoleGround, Vector3d::Zero(), rightLegSoleJacob, false);
    dualSoleJacob.block(0, 0, 6, NJG) = leftLegSoleJacob;
    dualSoleJacob.block(6, 0, 6, NJG) = rightLegSoleJacob;
    return true;
}

bool RobotDynamicsBiped::calcSoleJDotQDot() {
    leftLegSoleJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idLeftSoleGround, Vector3d::Zero(), false);
    rightLegSoleJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idRightSoleGround, Vector3d::Zero(), false);
    dualSoleJDotQDot.head(6) = leftLegSoleJDotQDot;//D 24.5.22
    dualSoleJDotQDot.tail(6) = rightLegSoleJDotQDot;
    return true;
}

bool RobotDynamicsBiped::calcMassMatrixCRBA() {
    inertiaMat.setZero();
    CompositeRigidBodyAlgorithm(*model, jntPositions, inertiaMat, false);
    return true;
}

bool RobotDynamicsBiped::calcNonlinearBiasRNEA() {
    gravityBias.setZero();
    VectorNd qDotZero = VectorNd::Zero(NJG);
    NonlinearEffects(*model, jntPositions, qDotZero, gravityBias);
    nonlinearBias.setZero();
    NonlinearEffects(*model, jntPositions, jntVelocities, nonlinearBias);
    coriolisBias = nonlinearBias - gravityBias;
    return true;
}

bool RobotDynamicsBiped::calcRigidBodyDynamicsDescriptors() {
    calcMassMatrixCRBA();
    calcNonlinearBiasRNEA();
    return true;
}

bool RobotDynamicsBiped::calcCentroidalDynamicsDescriptors() {
    SpatialMatrix phiR = SpatialMatrix::Zero(6,6);
    SpatialMatrix phi = SpatialMatrix::Zero(6,6);
    SpatialMatrix phiT = SpatialMatrix::Zero(6,6);
    SpatialMatrix psi = SpatialMatrix::Zero(6,6);
    SpatialMatrix psiT = SpatialMatrix::Zero(6,6);

    SpatialMatrix spatialTransformWaist2G = SpatialMatrix::Zero(6,6);
    Matrix3d rotMatformWaist2G = Matrix3d::Zero();

    Vector3d waistPosXYZ = Vector3d::Zero();
    Matrix3d waistRotm = Matrix3d::Zero();
    MatrixNd waistJacob = MatrixNd::Zero(NJF, NJG);
    MatrixNd spatialInertia1C = MatrixNd::Zero(NJF, NJF);

    //step: 1  Solve H Matrix
    //step: 2  Solve C Bias

    //step: 3  Solve pG Vector
    //0R1(3x3)  and waistJacob
    waistPosXYZ = CalcBodyToBaseCoordinates(*model, jntPositions, idPelvis, Vector3d::Zero(), false);
    waistRotm = CalcBodyWorldOrientation(*model, jntPositions, idPelvis, false).transpose();
    CalcPointJacobian6D(*model, jntPositions, idPelvis, Vector3d::Zero(), waistJacob, false);
    //phiR is constructed like:
    // |    0R1(3x3),   0(3x3)      |
    // |    0(3x3),     0R1(3x3)    |
    phiR.block(0, 0, 3, 3) = waistRotm.transpose();
    phiR.block(3, 3, 3, 3) = waistRotm.transpose();
    // psi = phi ^ T;   phi = phiR * waistJacob
    phi = phiR * waistJacob.block(0,0,NJF,NJF);
    phiT = phi.transpose();
    psi = phi.inverse();
    psiT = psi.transpose();
    spatialInertia1C = psiT * inertiaMat.block(0,0,NJF,NJF)  * psi;
    massAll = spatialInertia1C(5,5);
    comPos2Waist << spatialInertia1C(2,4)/massAll, spatialInertia1C(0,5)/massAll, spatialInertia1C(1,3)/massAll;
    comPos2World = waistPosXYZ + waistRotm * comPos2Waist;
    // step: 4  Solve Transform Matrix
    rotMatformWaist2G = waistRotm;
    spatialTransformWaist2G.block(0,0,3,3) = rotMatformWaist2G;
    spatialTransformWaist2G.block(3,3,3,3) = rotMatformWaist2G;
    spatialTransformWaist2G.block(0,3,3,3) = rotMatformWaist2G * (skew(comPos2Waist).transpose());
    spatialTransformG2ICS.block(0,0,3,3) = Matrix3d::Identity();
    spatialTransformG2ICS.block(3,3,3,3) = Matrix3d::Identity();
    spatialTransformG2ICS.block(3,0,3,3) = skew(-comPos2World).transpose();
    // step: 5 Solve AG and AGDotQDot
    // AG = iXG^T * psi^T * H11
    // AICS = G_X_ICS^T x AG
    centroidAG = spatialTransformWaist2G * psiT * inertiaMat.block(0,0,NJF,NJG);
    centroidAICS = spatialTransformG2ICS.transpose() * centroidAG;
    // AGDotQDot = iXG^T * psi^T * C1
    centroidAGDotQDot = spatialTransformWaist2G * psiT * coriolisBias.block(0,0,NJF,1);
    centroidAICSDotQDot = spatialTransformG2ICS.transpose() * centroidAGDotQDot;
    // IG = iXG^T * psi^T * H11 * psi * iXG
    compositeInertia = spatialTransformWaist2G * psiT * inertiaMat.block(0,0,NJF,NJF) * psi * spatialTransformWaist2G.transpose();
    return true;
}

bool RobotDynamicsBiped::calcSoleTask() {
    calcSoleJacob();
    calcSoleJDotQDot();
    return true;
}

bool RobotDynamicsBiped::calcWaistTask() {
    calcWaistJacob();
    calcWaistJDotQDot();
    return true;
}

bool RobotDynamicsBiped::calcCOMPosVel() {
    Utils::CalcCenterOfMass(*model, jntPositions, jntVelocities, NULL, massAll, comPos2World, &comVel2World, NULL, NULL, NULL, false);
    return true;
}
bool RobotDynamicsBiped::calcCOMState() {
    Utils::CalcCenterOfMass(*model, jntPositions, jntVelocities, NULL, massAll, comPos2World, &comVel2World, NULL, &angularMomentum, NULL, false);
    return true;
}
bool RobotDynamicsBiped::calcCentroidalMomentum() {
    centroidMomentum = centroidAG * jntVelocities;
    centroidMomentumICS = centroidAICS * jntVelocities;
    return true;
}

// ------------------------- Math function ----------------------------------------
Vector3d RobotDynamicsBiped::rotaitonMatrix2EulerRPY(const Matrix3d& rotMat) {
    Vector3d rpy = Vector3d::Zero();
    rpy(0) = std::atan2(rotMat(2,1), rotMat(2,2));
    rpy(1) = std::atan2(-rotMat(2,0), std::sqrt(rotMat(2,1) * rotMat(2,1) + rotMat(2,2) * rotMat(2,2)));
    rpy(2) = std::atan2(rotMat(1,0), rotMat(0,0));
    return rpy;
}

Vector3d RobotDynamicsBiped::angleDot2EulerDot(const Vector3d& angleDot, const Vector3d& rpy) {
    // rollDot       | cos(yaw)/cos(pitch) sin(yaw)/cos(pitch) 0 |
    // pitchDot   =  | -sin(yaw)           cos(yaw)            0 | *  angleDot
    // yawDOt        | cos(yaw)tan(pitch)  sin(yaw)tan(pitch)  1 |
    Vector3d rpyEulerDot;
    Matrix3d mat;
    double pitch = rpy(1);
    double yaw = rpy(2);
    mat << std::cos(yaw)/std::cos(pitch), std::sin(yaw)/std::cos(pitch), 0.0,
         -std::sin(yaw), std::cos(yaw), 0.0,
         std::cos(yaw)*std::tan(pitch),  std::sin(yaw)*std::tan(pitch),  1.0;
    rpyEulerDot = mat*angleDot;
    return rpyEulerDot;
}

Matrix3d RobotDynamicsBiped::skew(const Vector3d& omg) {
    Matrix3d matRes;
    matRes << 0, -omg(2), omg(1),
            omg(2), 0, -omg(0),
            -omg(1), omg(0), 0;
    return matRes;
}
// ------------------------- Math function ----------------------------------------
