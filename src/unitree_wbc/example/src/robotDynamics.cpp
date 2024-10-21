#include "robotDynamics.h"
using namespace std;

//===========================================================================
// Public functions for invocation
//===========================================================================
RobotDynamics::RobotDynamics() {
    // Basic elements
    NJG = NG;///< NJG = NJF + NJJ
    NJF = 6;  //< Number of Joints Free-floating
    NJJ = NJ;  ///< Number of non-floating-base Joints, including actuated & underactuated(paissive) joints. NJJ = NJA + NJP
    NJA = NJ;   ///< Number of Joints Actuated (torque_actuated)
    NJP = 0;  ///< Number of Passive joints that do not contain the DoFs of floating base
    NFC = NFCC4;  ///< Number of Forces describing Contact
 
    jntPositions = VectorNd :: Zero(NJG);
    jntVelocities = VectorNd :: Zero(NJG);

    inertiaMat = MatrixNd :: Zero(NJG, NJG);//Mass matrix
    invInertiaMat = MatrixNd :: Zero(NJG, NJG);//inverse of Mass matrix
    nonlinearBias = VectorNd :: Zero(NJG);//Coriolis and Gravity
    coriolisBias = VectorNd::Zero(NJG);//Coriolis
    gravityBias = VectorNd::Zero(NJG);//Gravity

    //nlBiasWithInternalCstr = VectorNd :: Zero(4);// ///< NJG*1, non-linear effects term when considering internal-constraints. for parallel robot
    //nspInternalCstrJacobian = MatrixNd :: Zero(2, NJG + NFC);//Dynamically Consistent Null Space Projection matrix of Jacobian_InternalConstraint. for parallel robot

    selMatFloatingBase = MatrixNd :: Zero(NJF, NJG);///< NJF*NJG, Selection matrix of floating-base joints
    selMatFloatingBase.leftCols(NJF) = MatrixNd::Identity(NJF, NJF);

    selMatActuated = MatrixNd :: Zero(NJA, NJG);///< NJA*NJG, Selection matrix of actuated joints or Actuation matrix
    selMatActuated.rightCols(NJA) = MatrixNd::Identity(NJA, NJA);

    selMatNonFloatingBase = MatrixNd :: Zero(NJJ, NJG);///< NJJ*NJG, Selection matrix of non-floating-base joints
    //selMatPassive = MatrixNd :: Zero(18, NJG + NFC);///< NJP+6+6*NJG+6+6, Selection matrix of passive joints that do not contain floating bases


    centroidalMomentumMatrix = MatrixNd :: Zero(NJF, NJG);///< NJF*NJG, Centroidal Momentum Matrix (CMM)质心动量矩阵
    centroidalMomentumBiased = VectorNd :: Zero(NJF);///< NJF*1, centroidal momentum bias force

    floatBaseJacoTc.J = MatrixNd :: Zero(NJF, NJG);///< JacobianTc of floating-base.
    floatBaseJacoTc.JdotQdot = VectorNd :: Zero(NJF);

    torsoJacoTc.J = MatrixNd :: Zero(NJF, NJG);///< JacobianTc of floating-base.
    torsoJacoTc.JdotQdot = VectorNd :: Zero(NJF);

    biContactJacoTc.J = MatrixNd :: Zero(12, NJG);///< JacobianTc of contact point(s)
    biContactJacoTc.JdotQdot = VectorNd :: Zero(12); 

    quadContactJacoTc.J = MatrixNd :: Zero(24, NJG);///< JacobianTc of contact point(s)
    quadContactJacoTc.JdotQdot = VectorNd :: Zero(24); 

    eqCstrMatTau = MatrixNd :: Zero(NJA, NJG+NFC);///< NJA*?, equality constraints : TauActuated = eqCstrMatTau * x^T + eqCstrMatTauBias, x is the generalized variables, e.g. NJA*(NJG+NFC), x = [Qddot, f_c]'
    eqCstrMatTauBias = VectorNd :: Zero(NJA);///< NJA*1, equality constraints : TauActuated = eqCstrMatTau * x^T + eqCstrMatTauBias

    model = new RigidBodyDynamics::Model();
    model->gravity = Vector3d (0., 0., -GRAVITY_CONST);
    
    Body floatingPelvisLink, torsoLink, fixedAnkleLink, fixedSoleLink, fixedArmEndLink;
    Body leftLegLink[5], leftArmLink[5];
    Body rightLegLink[5], rightArmLink[5];
    Joint floatingPelvisJoint, fixedSoleJoint, fixedAnkleJoint, fixedArmEndJoint;

    // Params of Unitree H1 humanoid robot
    floatingPelvisLink = BodyAkia(5.39, Vector3d(-0.0002, 0.0, -0.04522), 0.044582, 0.0082464, 0.049021, 0, 0, 0);

    leftLegLink[0] = BodyAkia(2.244, Vector3d(-0.04923, 0.0001, 0.0072), 0.0025731, 0.0030444, 0.0022883, 0, 0, 0); // hip yaw
    leftLegLink[1] = BodyAkia(2.232, Vector3d(-0.0058, -0.00319, -9e-05), 0.0020603, 0.0022482, 0.0024323, 0, 0, 0); // hip row
    leftLegLink[2] = BodyAkia(4.152, Vector3d(0.00746, -0.02346, -0.08193), 0.082618, 0.081579, 0.0060081, 0, 0, 0); // hip pitch
    leftLegLink[3] = BodyAkia(1.721, Vector3d(-0.00136, -0.00512, -0.1384), 0.012205, 0.012509, 0.0020629, 0, 0, 0); // knee
    leftLegLink[4] = BodyAkia(0.474, Vector3d(0.042575, 0, -0.044672), 0.000159668, 0.002900286, 0.002805438, 0, 0, 0); // ankle

    rightLegLink[0] = BodyAkia(2.244, Vector3d(-0.04923, -0.0001, 0.0072), 0.0025731, 0.0030444, 0.0022883, 0, 0, 0); // hip yaw
    rightLegLink[1] = BodyAkia(2.232, Vector3d(-0.0058, 0.00319, -9e-05), 0.0020603, 0.0022482, 0.0024323, 0, 0, 0); // hip row
    rightLegLink[2] = BodyAkia(4.152, Vector3d(0.00746, 0.02346, -0.08193), 0.082618, 0.081579, 0.0060081, 0, 0, 0); // hip pitch
    rightLegLink[3] = BodyAkia(1.721, Vector3d(-0.00136, 0.00512, -0.1384), 0.012205, 0.012509, 0.0020629, 0, 0, 0); // knee
    rightLegLink[4] = BodyAkia(0.474, Vector3d(0.042575, 0, -0.044672), 0.000159668, 0.002900286, 0.002805438, 0, 0, 0); // ankle

    leftArmLink[0] = BodyAkia(1.033, Vector3d(0.005045, 0.053657, -0.015715), 0.0012985, 0.00087279, 0.00097338, 0, 0, 0); // pitch
    leftArmLink[1] = BodyAkia(0.793, Vector3d(0.000679, 0.00115, -0.094076), 0.0015742, 0.0016973, 0.0010183, 0, 0, 0); // row
    leftArmLink[2] = BodyAkia(0.839, Vector3d(0.01365, 0.002767, -0.16266), 0.003664, 0.0040789, 0.00066383, 0, 0, 0); // yaw
    leftArmLink[3] = BodyAkia(0.723, Vector3d(0.164862, 0.000118, -0.015734), 0.00042388, 0.0060062, 0.0060023, 0, 0, 0); // elbow

    rightArmLink[0] = BodyAkia(1.033, Vector3d(0.005045, -0.053657, -0.015715), 0.0012985, 0.00087279, 0.00097338, 0, 0, 0); // pitch
    rightArmLink[1] = BodyAkia(0.793, Vector3d(0.000679, -0.00115, -0.094076), 0.0015742, 0.0016973, 0.0010183, 0, 0, 0); // row
    rightArmLink[2] = BodyAkia(0.839, Vector3d(0.01365, -0.002767, -0.16266), 0.003664, 0.0040789, 0.00066383, 0, 0, 0); // yaw
    rightArmLink[3] = BodyAkia(0.723, Vector3d(0.164862, 0.000118, -0.015734), 0.00042388, 0.0060062, 0.0060023, 0, 0, 0); // elbow

    torsoLink = BodyAkia(17.789, Vector3d(0.000489, 0.002797, 0.20484), 0.4873,0.40963,0.12785,0, 0, 0);

    fixedAnkleLink = Body(0.0, Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
    fixedSoleLink = Body(0.0, Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
    fixedArmEndLink = Body(0.0, Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));

    // Construction of robot use joints, body and transformation matrix;
    floatingPelvisJoint = Joint(SpatialVector(0.,0.,0., 1.,0.,0.),  //                ^ Z
                                SpatialVector(0.,0.,0., 0.,1.,0.), //                |
                                SpatialVector(0.,0.,0., 0.,0.,1.), //                |___Pelvis, Torso (1.1) 
                                SpatialVector(1.,0.,0., 0.,0.,0.), //                |
                                SpatialVector(0.,1.,0., 0.,0.,0.), //                |
                                SpatialVector(0.,0.,1., 0.,0.,0.));//                ------------> Y
    Joint joint_Rx = Joint(SpatialVector(1.,0.,0., 0.,0.,0.));     //               /
    Joint joint_Ry = Joint(SpatialVector(0.,1.,0., 0.,0.,0.));     //              / 
    Joint joint_Rz = Joint(SpatialVector(0.,0.,1., 0.,0.,0.));     //           X /
    fixedSoleJoint = Joint(JointTypeFixed);
    fixedArmEndJoint = Joint(JointTypeFixed);

    SpatialTransform rotation_lsp = Xrotx(0.43633);
    SpatialTransform translatin_lsp = Xtrans(Vector3d(0.0055, 0.15535, 0.42999));
    SpatialTransform transform_lsp = rotation_lsp * translatin_lsp;
    
    SpatialTransform rotation_lsr = Xrotx(-0.43633);
    SpatialTransform translatin_lsr = Xtrans(Vector3d(-0.0055, 0.0565, -0.0165));
    SpatialTransform transform_lsr = rotation_lsr * translatin_lsr;

    SpatialTransform rotation_rsp = Xrotx(-0.43633);
    SpatialTransform translatin_rsp = Xtrans(Vector3d(0.0055, -0.15535, 0.42999));
    SpatialTransform transform_rsp = rotation_rsp * translatin_rsp;

    SpatialTransform rotation_rsr = Xrotx(0.43633);
    SpatialTransform translatin_rsr = Xtrans(Vector3d(-0.0055, -0.0565, -0.0165));
    SpatialTransform transform_rsr = rotation_rsr * translatin_rsr;

    idPelvis = model->AddBody(0, Xtrans(Vector3d(0., 0. , 0.)), floatingPelvisJoint, floatingPelvisLink,"pelvis"); ///< Pelvis of World sys.

    idLeftLegLink[0] = model->AddBody(idPelvis, Xtrans(Vector3d(0, 0.0875, -0.1742)), joint_Rz, leftLegLink[0], "lhy");
    idLeftLegLink[1] = model->AppendBody(Xtrans(Vector3d(0.039468, 0, 0)), joint_Rx, leftLegLink[1], "lhr");
    idLeftLegLink[2] = model->AppendBody(Xtrans(Vector3d(0, 0.11536, 0)), joint_Ry, leftLegLink[2], "lhp");
    idLeftLegLink[3] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, leftLegLink[3], "lk");
    idLeftLegLink[4] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, fixedAnkleLink, "la");
    idLeftSole = model->AppendBody(Xtrans(Vector3d(0.0556, 0, -0.05)), fixedSoleJoint, leftLegLink[4], "lsole");//@Danny240531
    idLeftSoleGround = model->AppendBody(Xtrans(Vector3d(0, 0, -0.012)), fixedSoleJoint, fixedSoleLink, "lsoleg");

    idRightLegLink[0] = model->AddBody(idPelvis, Xtrans(Vector3d(0, -0.0875, -0.1742)), joint_Rz, rightLegLink[0], "rhy");
    idRightLegLink[1] = model->AppendBody(Xtrans(Vector3d(0.039468, 0, 0)), joint_Rx, rightLegLink[1], "rhr");
    idRightLegLink[2] = model->AppendBody(Xtrans(Vector3d(0, -0.11536, 0)), joint_Ry, rightLegLink[2], "rhp");
    idRightLegLink[3] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, rightLegLink[3], "rk");
    idRightLegLink[4] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.4)), joint_Ry, fixedAnkleLink, "ra");
    idRightSole = model->AppendBody(Xtrans(Vector3d(0.0556, 0, -0.05)), fixedSoleJoint, rightLegLink[4], "rsole");//@Danny240531
    idRightSoleGround = model->AppendBody(Xtrans(Vector3d(0, 0, -0.012)), fixedSoleJoint, fixedSoleLink, "rsoleg");

    idTorso = model->AddBody(idPelvis, Xtrans(Vector3d(0, 0.0, 0.0)), joint_Rz, torsoLink, "torso");///< Torso related to world sys. by pelvis
    // idTorso = model->AddBody(idPelvis, Xtrans(Vector3d(0, 0.0, 0.1)), fixedSoleJoint, torsoLink, "torso");

    // idLeftArmLink[0] = model->AddBody(idTorso, Xtrans(Vector3d(0.0055, 0.15535, 0.42999)), joint_Ry, leftArmLink[0], "lsp");
    // idLeftArmLink[1] = model->AppendBody(Xtrans(Vector3d(-0.0055, 0.0565, -0.0165)), joint_Rx, leftArmLink[1], "lsr");
    // idLeftArmLink[2] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.1343)), joint_Rz, leftArmLink[2], "lsy");
    // idLeftArmLink[3] = model->AppendBody(Xtrans(Vector3d(0.0185, 0, -0.198)), joint_Ry, leftArmLink[3], "ls");
    // idLeftArmEnd = model->AppendBody(Xtrans(Vector3d(0.27, 0.0, -0.02)), fixedArmEndJoint, fixedArmEndLink, "left_arm_end");

    // idRightArmLink[0] = model->AddBody(idTorso, Xtrans(Vector3d(0.0055, -0.15535, 0.42999)), joint_Ry, rightLegLink[0], "rsp");
    // idRightArmLink[1] = model->AppendBody(Xtrans(Vector3d(-0.0055, -0.0565, -0.0165)), joint_Rx, rightLegLink[1], "rsr");
    // idRightArmLink[2] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.1343)), joint_Rz, rightLegLink[2], "rsy");
    // idRightArmLink[3] = model->AppendBody(Xtrans(Vector3d(0.0185, 0, -0.198)), joint_Ry, rightLegLink[3], "rs");
    // idRightArmEnd = model->AppendBody(Xtrans(Vector3d(0.27, 0.0, -0.02)), fixedArmEndJoint, fixedArmEndLink, "right_arm_end");

    idLeftArmLink[0] = model->AddBody(idTorso, transform_lsp, joint_Ry, leftArmLink[0], "lsp");
    idLeftArmLink[1] = model->AppendBody(transform_lsr , joint_Rx, leftArmLink[1], "lsr");
    idLeftArmLink[2] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.1343)), joint_Rz, leftArmLink[2], "lsy");
    idLeftArmLink[3] = model->AppendBody(Xtrans(Vector3d(0.0185, 0, -0.198)), joint_Ry, leftArmLink[3], "ls");
    idLeftArmEnd = model->AppendBody(Xtrans(Vector3d(0.27, 0.0, -0.02)), fixedArmEndJoint, fixedArmEndLink, "left_arm_end");

    idRightArmLink[0] = model->AddBody(idTorso, transform_rsp, joint_Ry, rightLegLink[0], "rsp");
    idRightArmLink[1] = model->AppendBody(transform_rsr, joint_Rx, rightLegLink[1], "rsr");
    idRightArmLink[2] = model->AppendBody(Xtrans(Vector3d(0, 0, -0.1343)), joint_Rz, rightLegLink[2], "rsy");
    idRightArmLink[3] = model->AppendBody(Xtrans(Vector3d(0.0185, 0, -0.198)), joint_Ry, rightLegLink[3], "rs");
    idRightArmEnd = model->AppendBody(Xtrans(Vector3d(0.27, 0.0, -0.02)), fixedArmEndJoint, fixedArmEndLink, "right_arm_end");

    massAll = floatingPelvisLink.mMass + torsoLink.mMass;
    for (int i = 0; i < 5; i++){massAll += leftLegLink[i].mMass + rightLegLink[i].mMass;}
    for (int i = 0; i < 4; i++){massAll += leftArmLink[i].mMass + rightArmLink[i].mMass;}

    centroidAG  = MatrixNd::Zero(NJF,NJG);
    centroidAICS = MatrixNd::Zero(NJF, NJG);
    compositeInertia = SpatialMatrix::Zero(NJF,NJF);
    centroidAGDotQDot = SpatialVector::Zero(NJF);
    centroidAICSDotQDot = SpatialVector::Zero(NJF);
    centroidMomentum = SpatialVector::Zero(NJF);
    centroidMomentumICS = SpatialVector::Zero(NJF);
    spatialTransformG2ICS = SpatialMatrix::Zero(6,6);

    comPos2Pelvis = Vector3d::Zero();
    comPos2World = Vector3d::Zero();
    comVel2World = Vector3d::Zero();
    linearMomentum = Vector3d::Zero();
    angularMomentum = Vector3d::Zero();
  
    pelvisJacob = MatrixNd::Zero(NJF, NJG);
    pelvisJDotQDot = VectorNd::Zero(NJF);
    torsoJacob = MatrixNd::Zero(NJF, NJG);
    torsoJDotQDot = VectorNd::Zero(NJF);

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
}

RobotDynamics::~RobotDynamics(){
    delete model;
    model = nullptr;
}

bool RobotDynamics::setJntStates(const Eigen::VectorXd &q, const Eigen::VectorXd &qdot){
    AGIROBOT::RobotDynamics::setJntStates(q, qdot);
    isPosVelUpdated = false;
    return true;
}

bool RobotDynamics::calcWbcDependence(){
    if (!isPosVelUpdated){
        updateKinematicsPosVel();
    }
    calcRigidBodyDynamicsDescriptors();
//  calcCentroidalDynamicsDescriptors();
    updateKinematicsAcc();
    calcSoleTask();
    calcPelvisTask();
    quadContactJacoTc.J = quadSoleJacob;
    quadContactJacoTc.JdotQdot = quadSoleJDotQDot;
    biContactJacoTc.J = dualSoleJacob;
    biContactJacoTc.JdotQdot = dualSoleJDotQDot;
    floatBaseJacoTc.J = pelvisJacob;
    floatBaseJacoTc.JdotQdot = pelvisJDotQDot;
    torsoJacoTc.J = torsoJacob;
    torsoJacoTc.JdotQdot = torsoJDotQDot;
    eqCstrMatTau << selMatActuated * inertiaMat, //A*G * G*G
                -selMatActuated * leftLegSoleJacob.transpose(),//A*F * G*F()
                -selMatActuated * rightLegSoleJacob.transpose(),//A*F * G*F   ====>A*(G+2F)   {TauActuated = eqCstrMatTau * x^T + eqCstrMatTauBias}  ===> X = (G+2F) * 1  = (g+fc)*1=nv*1
                -selMatActuated * leftArmSoleJacob.transpose(),
                -selMatActuated * rightArmSoleJacob.transpose();//Force @@
    eqCstrMatTauBias = selMatActuated * nonlinearBias;//A*G * G*1 = A*1
    return true;
}

double RobotDynamics::getTotalMass(){
    return massAll;
}

//===========================================================================
// Update and calculate dynamics params
//===========================================================================
VectorNd RobotDynamics::estPelvisPosVelInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& footType) {
    VectorNd qTemp = VectorNd::Zero(NJG);
    VectorNd qDotTemp = VectorNd::Zero(NJG);
    Vector3d sole2PelvisPos = Vector3d::Zero();
    Vector3d sole2PelvisVel = Vector3d::Zero();
    VectorNd posVelRes = VectorNd::Zero(6,1);
    qTemp = jointPos;
    qDotTemp = jointVel;
    UpdateKinematicsCustom(*model, & qTemp, & qDotTemp, NULL);
    switch (footType)
    {
        case TYPELEFTSOLE: // Left Sole
            sole2PelvisPos = CalcBodyToBaseCoordinates(*model, qTemp, idLeftSoleGround, Math::Vector3d::Zero(), false);
            sole2PelvisVel = CalcPointVelocity(*model, qTemp, qDotTemp, idLeftSoleGround, Math::Vector3d::Zero(), false);
            break;
        case TYPERIGHTSOLE: // Right Sole
            sole2PelvisPos = CalcBodyToBaseCoordinates(*model, qTemp, idRightSoleGround, Math::Vector3d::Zero(), false);
            sole2PelvisVel = CalcPointVelocity(*model, qTemp, qDotTemp, idRightSoleGround, Math::Vector3d::Zero(), false);
            break;    
        default:
            break;
    }
    posVelRes.head(3) = qTemp.head(3)-sole2PelvisPos;
    posVelRes.tail(3) = qDotTemp.head(3)-sole2PelvisVel;
    return posVelRes;
}

VectorNd RobotDynamics::estBodyPosInWorldAkia(const VectorNd& jointPos, const VectorNd& jointVel, const unsigned int& bodyId) {//Daniel 5.27
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
        case 1:
            bodyPos = CalcBodyToBaseCoordinates(*model, qTemp, idLeftArmEnd, Math::Vector3d::Zero(), false);
            break;
        case 2:
            bodyPos = CalcBodyToBaseCoordinates(*model, qTemp, idRightArmEnd, Math::Vector3d::Zero(), false);
            break;
        case 3:
            bodyPos = CalcBodyToBaseCoordinates(*model, qTemp, idLeftSoleGround, Math::Vector3d::Zero(), false);
            break;
        case 4:
            bodyPos = CalcBodyToBaseCoordinates(*model, qTemp,idRightSoleGround, Math::Vector3d::Zero(), false);
            break;
        default:
            break;
    }
    return bodyPos;
}

VectorNd RobotDynamics::estFootArmPosVelInWorld(const VectorNd& jointPos, const VectorNd& jointVel, const int& footType) {
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
        case 0: // Torso
            sole2WorldPos = CalcBodyToBaseCoordinates(*model, qTemp, idTorso, Math::Vector3d::Zero(), false);
            sole2WorldMatR = CalcBodyWorldOrientation(*model, qTemp, idTorso, false);
            sole2WorldVel = CalcPointVelocity6D(*model, qTemp, qDotTemp, idTorso, Math::Vector3d::Zero(), false);
            break; 

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
        case 3: // Left Arm
            sole2WorldPos = CalcBodyToBaseCoordinates(*model, qTemp, idLeftArmEnd, Math::Vector3d::Zero(), false);
            sole2WorldMatR = CalcBodyWorldOrientation(*model, qTemp, idLeftArmEnd, false);
            sole2WorldVel = CalcPointVelocity6D(*model, qTemp, qDotTemp, idLeftArmEnd, Math::Vector3d::Zero(), false);
            break;
        case 4: // Right Arm
            sole2WorldPos = CalcBodyToBaseCoordinates(*model, qTemp, idRightArmEnd, Math::Vector3d::Zero(), false);
            sole2WorldMatR = CalcBodyWorldOrientation(*model, qTemp, idRightArmEnd, false);
            sole2WorldVel = CalcPointVelocity6D(*model, qTemp, qDotTemp, idRightArmEnd, Math::Vector3d::Zero(), false);
            break;
        default:
            break;
    }
    sole2WorldRPY = -rotaitonMatrix2EulerRPY(sole2WorldMatR);
    posVelRes.head(3) = sole2WorldRPY;
    posVelRes.segment(3,3) = sole2WorldPos;
    posVelRes.segment(6,3) = angleDot2EulerDot(sole2WorldVel.head(3), sole2WorldRPY);
    posVelRes.tail(3) = sole2WorldVel.tail(3);
    return posVelRes;
}


VectorNd RobotDynamics::getRootXyzRpy(const Eigen::VectorXd & q){//Daniel 5.26
     Eigen::VectorXd rootPose(6);

    // Obtain root node pos.
    Vector3d position = CalcBodyToBaseCoordinates(*model, q, idPelvis, Vector3d(0.0, 0.0, 0.0), false);
    rootPose.segment<3>(0) = position;

    // Obtain root node rpy.
    Matrix3d orientation = CalcBodyWorldOrientation(*model, q, idPelvis, false);
    Eigen::Vector3d eulerAngles = orientation.eulerAngles(0, 1, 2); 
    rootPose.segment<3>(3) = eulerAngles;

    return rootPose;
}

VectorNd RobotDynamics::getRootXyzRpyDot(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot) {//Daniel 5.27
    Eigen::VectorXd rootPoseDot(6);

    // Obtain root node Velocity screw  
    VectorNd velocity = CalcPointVelocity6D(*model, q, qDot, idPelvis, Vector3d(0.0, 0.0, 0.0), false);

    // Split it
    Vector3d linearVelocity = velocity.segment<3>(3);  // 后三个分量是线速度
    Vector3d angularVelocity = velocity.segment<3>(0); // 前三个分量是角速度

    rootPoseDot.segment<3>(0) = linearVelocity;
    rootPoseDot.segment<3>(3) = angularVelocity;

    return rootPoseDot;
}

bool RobotDynamics::updateKinematicsPosVel() {
    UpdateKinematicsCustom(*model, &jntPositions, &jntVelocities, NULL);
    isPosVelUpdated = true;
    return true;
}

bool RobotDynamics::updateKinematicsAcc() {
    VectorNd qDDotZero = VectorNd::Zero(NJG);
    UpdateKinematicsCustom(*model, NULL, NULL, &qDDotZero);
    return true;
}

bool RobotDynamics::calcPelvisJacob() {
    CalcPointJacobian6D(*model, jntPositions, idPelvis, Vector3d::Zero(), pelvisJacob, false);
    return true;
}

bool RobotDynamics::calcPelvisJDotQDot() {
    pelvisJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idPelvis, Vector3d::Zero(), false);
    return true;
}

bool RobotDynamics::calcTorsoJacob() {
    CalcPointJacobian6D(*model, jntPositions, idTorso, Vector3d::Zero(), torsoJacob, false);
    return true;
}

bool RobotDynamics::calcTorsoJDotQDot() {
    torsoJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idTorso, Vector3d::Zero(), false);
    return true;
}

bool RobotDynamics::calcSoleJacob() {
    CalcPointJacobian6D(*model, jntPositions, idLeftSoleGround, Vector3d::Zero(), leftLegSoleJacob, false);
    CalcPointJacobian6D(*model, jntPositions, idRightSoleGround, Vector3d::Zero(), rightLegSoleJacob, false);
    CalcPointJacobian6D(*model, jntPositions, idLeftArmEnd, Vector3d::Zero(), leftArmSoleJacob, false);
    CalcPointJacobian6D(*model, jntPositions, idRightArmEnd, Vector3d::Zero(), rightArmSoleJacob, false);
    dualSoleJacob.block(0, 0, 6, NJG) = leftLegSoleJacob;
    dualSoleJacob.block(6, 0, 6, NJG) = rightLegSoleJacob;

    quadSoleJacob.block(0, 0, 6, NJG) = leftLegSoleJacob;
    quadSoleJacob.block(6, 0, 6, NJG) = rightLegSoleJacob;
    quadSoleJacob.block(12, 0, 6, NJG) = leftArmSoleJacob;
    quadSoleJacob.block(18, 0, 6, NJG) = rightArmSoleJacob;
    return true;
}

bool RobotDynamics::calcSoleJDotQDot() {
    leftLegSoleJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idLeftSoleGround, Vector3d::Zero(), false);
    rightLegSoleJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idRightSoleGround, Vector3d::Zero(), false);
    leftArmSoleJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idLeftArmEnd, Vector3d::Zero(), false);
    rightArmSoleJDotQDot = CalcPointAcceleration6D(*model, jntPositions, jntVelocities, VectorNd::Zero(NJG), idRightArmEnd, Vector3d::Zero(), false);
    dualSoleJDotQDot.head(6) = leftLegSoleJDotQDot;//D 24.5.22
    dualSoleJDotQDot.tail(6) = rightLegSoleJDotQDot;

    quadSoleJDotQDot.head(6) = leftLegSoleJDotQDot;
    quadSoleJDotQDot.segment(6,6) = rightLegSoleJDotQDot;
    quadSoleJDotQDot.segment(12,6) = leftArmSoleJDotQDot;
    quadSoleJDotQDot.tail(6) = rightArmSoleJDotQDot;
    return true;
}

bool RobotDynamics::calcMassMatrixCRBA() {
    inertiaMat.setZero();
    CompositeRigidBodyAlgorithm(*model, jntPositions, inertiaMat, false);
    return true;
}

bool RobotDynamics::calcNonlinearBiasRNEA() {
    gravityBias.setZero();
    VectorNd qDotZero = VectorNd::Zero(NJG);
    NonlinearEffects(*model, jntPositions, qDotZero, gravityBias);
    nonlinearBias.setZero();
    NonlinearEffects(*model, jntPositions, jntVelocities, nonlinearBias);
    coriolisBias = nonlinearBias - gravityBias;
    return true;
}

bool RobotDynamics::calcRigidBodyDynamicsDescriptors() {
    calcMassMatrixCRBA();
    calcNonlinearBiasRNEA();
    return true;
}

bool RobotDynamics::calcCentroidalDynamicsDescriptors() {
    SpatialMatrix phiR = SpatialMatrix::Zero(6,6);
    SpatialMatrix phi = SpatialMatrix::Zero(6,6);
    SpatialMatrix phiT = SpatialMatrix::Zero(6,6);
    SpatialMatrix psi = SpatialMatrix::Zero(6,6);
    SpatialMatrix psiT = SpatialMatrix::Zero(6,6);

    SpatialMatrix spatialTransformPelvis2G = SpatialMatrix::Zero(6,6);
    Matrix3d rotMatformPelvis2G = Matrix3d::Zero();

    Vector3d pelvisPosXYZ = Vector3d::Zero();
    Matrix3d pelvisRotm = Matrix3d::Zero();
    MatrixNd pelvisJacob = MatrixNd::Zero(NJF, NJG);
    MatrixNd spatialInertia1C = MatrixNd::Zero(NJF, NJF);

    //step: 1  Solve H Matrix
    //step: 2  Solve C Bias

    //step: 3  Solve pG Vector
    //0R1(3x3)  and pelvisJacob
    pelvisPosXYZ = CalcBodyToBaseCoordinates(*model, jntPositions, idPelvis, Vector3d::Zero(), false);
    pelvisRotm = CalcBodyWorldOrientation(*model, jntPositions, idPelvis, false).transpose();
    CalcPointJacobian6D(*model, jntPositions, idPelvis, Vector3d::Zero(), pelvisJacob, false);
    //phiR is constructed like:
    // |    0R1(3x3),   0(3x3)      |
    // |    0(3x3),     0R1(3x3)    |
    phiR.block(0, 0, 3, 3) = pelvisRotm.transpose();
    phiR.block(3, 3, 3, 3) = pelvisRotm.transpose();
    // psi = phi ^ T;   phi = phiR * pelvisJacob
    phi = phiR * pelvisJacob.block(0,0,NJF,NJF); //transformation from world 2 body;
    phiT = phi.transpose();
    psi = phi.inverse(); //transformation from body to world;
    psiT = psi.transpose();
    spatialInertia1C = psiT * inertiaMat.block(0,0,NJF,NJF)  * psi;//inertia matrix in B;
    massAll = spatialInertia1C(5,5);
    comPos2Pelvis << spatialInertia1C(2,4)/massAll, spatialInertia1C(0,5)/massAll, spatialInertia1C(1,3)/massAll;
    comPos2World = pelvisPosXYZ + pelvisRotm * comPos2Pelvis;
    // step: 4  Solve Transform Matrix
    rotMatformPelvis2G = pelvisRotm;

    spatialTransformPelvis2G.block(0,0,3,3) = rotMatformPelvis2G;
    spatialTransformPelvis2G.block(3,3,3,3) = rotMatformPelvis2G;
    spatialTransformPelvis2G.block(0,3,3,3) = rotMatformPelvis2G * (skew(comPos2Pelvis).transpose());
    spatialTransformG2ICS.block(0,0,3,3) = Matrix3d::Identity();
    spatialTransformG2ICS.block(3,3,3,3) = Matrix3d::Identity();
    spatialTransformG2ICS.block(3,0,3,3) = skew(-comPos2World).transpose();
    // step: 5 Solve AG and AGDotQDot
    // AG = iXG^T * psi^T * H11
    // AICS = G_X_ICS^T x AG
    centroidAG = spatialTransformPelvis2G * psiT * inertiaMat.block(0,0,NJF,NJG);
    centroidAICS = spatialTransformG2ICS.transpose() * centroidAG;
    // AGDotQDot = iXG^T * psi^T * C1
    centroidAGDotQDot = spatialTransformPelvis2G * psiT * coriolisBias.block(0,0,NJF,1);
    centroidAICSDotQDot = spatialTransformG2ICS.transpose() * centroidAGDotQDot;
    // IG = iXG^T * psi^T * H11 * psi * iXG
    compositeInertia = spatialTransformPelvis2G * psiT * inertiaMat.block(0,0,NJF,NJF) * psi * spatialTransformPelvis2G.transpose();
    return true;
}

bool RobotDynamics::calcSoleTask() {
    calcSoleJacob();
    calcSoleJDotQDot();
    return true;
}

bool RobotDynamics::calcPelvisTask() {
    calcTorsoJacob();
    calcTorsoJDotQDot();  
    calcPelvisJacob();
    calcPelvisJDotQDot(); 
    return true;
}

bool RobotDynamics::calcCOMPosVel() {
    Utils::CalcCenterOfMass(*model, jntPositions, jntVelocities, NULL, massAll, comPos2World, &comVel2World, NULL, NULL, NULL, false);
    return true;
}
bool RobotDynamics::calcCOMState() {
    Utils::CalcCenterOfMass(*model, jntPositions, jntVelocities, NULL, massAll, comPos2World, &comVel2World, NULL, &angularMomentum, NULL, false);
    return true;
}
bool RobotDynamics::calcCentroidalMomentum() {
    centroidMomentum = centroidAG * jntVelocities;
    centroidMomentumICS = centroidAICS * jntVelocities;
    return true;
}

//===========================================================================
// Math operations
//===========================================================================
Vector3d RobotDynamics::rotaitonMatrix2EulerRPY(const Matrix3d& rotMat) {
    Vector3d rpy = Vector3d::Zero();
    rpy(0) = std::atan2(rotMat(2,1), rotMat(2,2));
    rpy(1) = std::atan2(-rotMat(2,0), std::sqrt(rotMat(2,1) * rotMat(2,1) + rotMat(2,2) * rotMat(2,2)));
    rpy(2) = std::atan2(rotMat(1,0), rotMat(0,0));
    return rpy;
}

Vector3d RobotDynamics::angleDot2EulerDot(const Vector3d& angleDot, const Vector3d& rpy) {
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

Matrix3d RobotDynamics::skew(const Vector3d& omg) {
    Matrix3d matRes;
    matRes << 0, -omg(2), omg(1),
            omg(2), 0, -omg(0),
            -omg(1), omg(0), 0;
    return matRes;
}
