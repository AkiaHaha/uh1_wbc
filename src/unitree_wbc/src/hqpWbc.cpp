#include "hqpWbc.h"

namespace TAICHI {

// ======================================== public Functions ====================================================

HqpWbc::HqpWbc(int dimVar, RobotDynamics * roDy):Wbc(dimVar, roDy){
    createNewQP();
    resizeQPMatrixVector();
}

HqpWbc::HqpWbc(const Wbc & wbcInstance):Wbc(wbcInstance){
    createNewQP();
    resizeQPMatrixVector();
}

// ---------------------- rewrite virtual functions ------------------------

HqpWbc::~HqpWbc(){
    for (int i = 0; i != nLevel; i++) {
        delete QP.at(i);
        delete cpuTimePtr.at(i);
        delete [] primalOptPtr.at(i);
        delete [] dualOptPtr.at(i);
        QP.at(i) = nullptr;
        cpuTimePtr.at(i) = nullptr;
        primalOptPtr.at(i) = nullptr;
        dualOptPtr.at(i) = nullptr;
    }
}

bool HqpWbc::wbcInit(){
    createNewQP();
    resizeQPMatrixVector();
    return true;
}

bool HqpWbc::displayResultInformation() const{
    for (int i = 0; i != nLevel; i++){
        std::cout << "-------------------------   Level " << i << " Information   ----------------------" << std::endl;
        std::cout << "Numbers: " << std::endl
                << "#################   qpOASES  --  QP Basic Numbers   #################" << std::endl
                << "number of variables :                                     " << QP.at(i)->getNV() << std::endl
                << "number of free variables :                                " << QP.at(i)->getNFR() << std::endl
                << "number of fixed variables :                               " << QP.at(i)->getNFX() << std::endl
                << "number of constraints :                                   " << QP.at(i)->getNC() << std::endl
                << "number of (implicitly defined) equality constraints :     " << QP.at(i)->getNEC() << std::endl
                << "number of active constraints :                            " << QP.at(i)->getNAC() << std::endl
                << "number of inactive constraints :                          " << QP.at(i)->getNIAC() << std::endl;

        /// If use the follow, set qpOption.printLevel not to be PL_NONE !
        std::cout << "Properties : " << qpOASES::MessageHandling::getErrorCodeMessage(QP.at(i)->printProperties()) << std::endl;
        std::cout << "Options : " << qpOASES::MessageHandling::getErrorCodeMessage(QP.at(i)->printOptions()) << std::endl;
    }
    return true;
}


bool HqpWbc::setParametersInt(const std::vector<int> &parameters){

    #ifdef USE_ERROR
        if(parameters.empty() || parameters.size() < static_cast<unsigned int>(nLevel)) {
            throw InvalidDimension(
                "Error : In [HqpWbc::setParametersInt], vector dimensions do not match!" );
            }
    #else

        try {
            if(parameters.empty() || parameters.size() < static_cast<unsigned int>(nLevel)) {
                throw InvalidDimension(
                    "Error : In [HqpWbc::setParametersInt], vector dimensions do not match!" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        }
    #endif

    for (int i = 0; i != nLevel; i++){
        nWSRDes.at(i) = parameters.at(0);
    }

    return true;
}

bool HqpWbc::setParametersDouble(const std::vector<double> &parameters){

    #ifdef USE_ERROR
        if(parameters.empty() || parameters.size() < static_cast<unsigned int>(nLevel)) {
            throw InvalidDimension(
                "Error : In [HqpWbc::setParametersDouble], vector dimensions do not match!" );
            }
    #else

        try {
            if(parameters.empty() || parameters.size() < static_cast<unsigned int>(nLevel)) {
                throw InvalidDimension(
                    "Error : In [HqpWbc::setParametersDouble], vector dimensions do not match!" );
                }
        } catch ( InvalidDimension invalidDimensionObj ) {
            std::cout << invalidDimensionObj.what() << std::endl;
            return false;
        }
    #endif

    for (int i = 0; i != nLevel; i++){
        cpuTimeDes.at(i) = parameters.at(0);
    }

    return true;
}

bool HqpWbc::getAuxiliaryDataInt(std::vector<int> &auxiliaryData){

    auxiliaryData.resize(2 * nLevel);
    std::vector<int> NwsrData(nLevel, -1);
    std::vector<int> SimpleStatusData(nLevel, -1);
    getNwsr(NwsrData);
    getSimpleStatusInt(SimpleStatusData);
    for (int i = 0; i != nLevel; i++){
        auxiliaryData.at(i) = NwsrData.at(i);
        auxiliaryData.at(i + nLevel) = SimpleStatusData.at(i);
    }
    return true;
}

bool HqpWbc::getAuxiliaryDataDouble(std::vector<double> &auxiliaryData){

    auxiliaryData.resize(2 * nLevel);
    std::vector<double> CostData(nLevel, -1);
    getOptCost(CostData);
    for (int i = 0; i != nLevel; i++){
        auxiliaryData.at(i) = CostData.at(i);
        auxiliaryData.at(i + nLevel) = static_cast<double>(* cpuTimePtr.at(i));
    }
    return true;
}

// ---------------------- implement pure virtual functions -----------------

bool HqpWbc::updateBound(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub){

    for (int i = 0; i != nLevel; i++){
        if(check(lb, nVLevel.at(i))){
            lowerBoundVec.at(i) = lb;
        }else{
            return false;
        }
        if(check(ub, nVLevel.at(i))){
            upperBoundVec.at(i) = ub;
        }else{
            return false;
        }
    }
    return true;
}

bool HqpWbc::updateRobotDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot){
    /// Provide opportunities for multi-threaded computing
    if (!robot->isCalcWbcDependenceDone()){
        robot->setJntStates(q, qDot);
        robot->calcWbcDependence();
    }
    return true;
}

bool HqpWbc::wbcSolve(){
  
    hqpUpdate();     // update QP solver
    nspSolve();
    for (int i = 0; i !=nLevel; i++){
        if (nOLevel.at(i)  == 0){
            if ( i == 0){        
                optVarStar.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));
            }else{
                optVarStar.at(i) = optVarStar.at(i - 1);
            }
        }else{
            hqpUpdateLeveln(i);
            hqpSolveLeveln(i);      // solve QP
            getResultOptLeveln(optVarStar.at(i), i);

            std::cout << "********* resulted primal opt star in level " << i << "******************" << std::endl;
            std::cout << optVarStar.at(i).transpose() << std::endl;            
            
            if ( i != 0){        
                optVarStar.at(i) = optVarStar.at(i - 1) + nspMat.at(i - 1) *  optVarStar.at(i);
            }           
            std::cout << "********* optVarStar at level " << i << "************" << std::endl;
            std::cout << optVarStar.at(i).transpose() << std::endl;
        }
    }
    return true;
}
bool HqpWbc::getResultOpt(Eigen::VectorXd &resultOpt){
    // TODO: 
//    for(int i = 0; i != nLevel; i++) {
//         resultOpt += optVarStar.at(i);
//    }
//    std::cout << resultOpt.transpose();
   resultOpt = optVarStar.at(nLevel-1);
    return true;
}



// ======================================== private Functions ====================================================

bool HqpWbc::createNewQP(){

    nLevelTask = priorityTaskNames.size();
    nLevelConstraint = priorityConstraintNames.size();
    nLevel = (nLevelTask > nLevelConstraint) ? nLevelTask : nLevelConstraint;

    nVLevel.resize(nLevel);
    nOLevel.resize(nLevel);
    nCLevel.resize(nLevel);

    int iLevel = 0;
    for(auto level : priorityTaskNames){
        for(auto item : level){
            auto iter = tasks.find(item);
            nVLevel.at(iLevel) = iter->second->varDof;
            nOLevel.at(iLevel) += iter->second->dim;
        }
    iLevel++;
    }    

    iLevel = 0;
    for(auto level : priorityConstraintNames){
        for(auto item : level){
            auto iter = constraints.find(item);
            nCLevel.at(iLevel) += iter->second->dim;
        }
    iLevel++;
    }    

    QP.resize(nLevel);
    qpOption.resize(nLevel);

    nWSR.resize(nLevel);
    cpuTimePtr.resize(nLevel);

    primalOptPtr.resize(nLevel);
    dualOptPtr.resize(nLevel);

    statusCodeSolving.resize(nLevel);
    statusCodeGetSolution.resize(nLevel);

    initDone.resize(nLevel);
    nWSRDes.resize(nLevel);
    cpuTimeDes.resize(nLevel);

    int sumNCLevel = 0;
    for (int i = 0; i != nLevel; i++){
        if (nVLevel.at(i) == 0){
            nVLevel.at(i) = *std::max_element(nVLevel.begin(),nVLevel.end());
            std::cout << "Warning : QP empty in level " << i << "!" << std::endl;
        } 

        sumNCLevel += nCLevel.at(i);
       
        qpOption.at(i).setToFast();
        qpOption.at(i).printLevel = qpOASES::PL_NONE;
        qpOption.at(i).terminationTolerance = 0.00001;
        qpOption.at(i).enableRegularisation = qpOASES::BT_TRUE;

        QP.at(i) = new qpOASES::SQProblem(nVLevel.at(i), sumNCLevel);
        QP.at(i)->setOptions(qpOption.at(i));

        primalOptPtr.at(i) = new qpOASES::real_t[nVLevel.at(i)];
        dualOptPtr.at(i) = new qpOASES::real_t[nVLevel.at(i) + sumNCLevel];

        initDone.at(i) = false;
        nWSRDes.at(i) = 100;
        cpuTimeDes.at(i) = 10;

        nWSR.at(i) = nWSRDes.at(i);
        cpuTimePtr.at(i) = new double(cpuTimeDes.at(i));
    }

    return true;
}


bool HqpWbc::resizeQPMatrixVector(){
    // ---------------------------- Costs/Objects --------------------------------------
    hessianMat.resize(nLevel);
    gradientVec.resize(nLevel);
    weiVecAll.resize(nLevel);
    taskMatAll.resize(nLevel);
    taskVecAll.resize(nLevel);

    lowerBoundVec.resize(nLevel);
    upperBoundVec.resize(nLevel);
    cstrMatAll.resize(nLevel);
    cstrMatAllTrans.resize(nLevel);
    lbCstrAll.resize(nLevel);
    ubCstrAll.resize(nLevel);

    nspMat.resize(nLevel);
    optVarStar.resize(nLevel);

    weiVecLevel.resize(nLevel);
    taskMatLevel.resize(nLevel);
    taskMatHatLevel.resize(nLevel);
    taskMatHatInvLevel.resize(nLevel);
    taskVecLevel.resize(nLevel);
    cstrMatLevel.resize(nLevel);
    cstrMatLevelTrans.resize(nLevel);
    lbCstrLevel.resize(nLevel);
    ubCstrLevel.resize(nLevel);


    int sumNCLevel = 0;
    for (int i = 0; i != nLevel; i++){
        sumNCLevel += nCLevel.at(i);
        hessianMat.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),nVLevel.at(i));                  // nVi*nVi
        gradientVec.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                               // nVi*1
        weiVecAll.at(i) = Eigen::VectorXd::Zero(nOLevel.at(i));                                 // nOi*1
        taskMatAll.at(i) = Eigen::MatrixXd::Zero(nOLevel.at(i),nVLevel.at(i));                  // nOi*nVi
        taskVecAll.at(i) = Eigen::VectorXd::Zero(nOLevel.at(i));                                // nOi*1

        // ------------------------ Bounds & Constraints -----------------------------------
        lowerBoundVec.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                             // nVi*1
        upperBoundVec.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                             // nVi*1
        cstrMatAll.at(i) = Eigen::MatrixXd::Zero(sumNCLevel,nVLevel.at(i));                    // (nC0 + ... +nCi)*nVi
        cstrMatAllTrans.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),sumNCLevel);               // nVi*(nC0 + ... +nCi)
        lbCstrAll.at(i) = Eigen::VectorXd::Zero(sumNCLevel);                                   // (nC0 + ... +nCi)*1
        ubCstrAll.at(i) = Eigen::VectorXd::Zero(sumNCLevel);                                   // (nC0 + ... +nCi)*1

        // ------------------------ Bounds & Constraints -----------------------------------
        nspMat.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i), nVLevel.at(i));                     // nVi*nVi
        optVarStar.at(i) = Eigen::VectorXd::Zero(nVLevel.at(i));                                // nVi*1
        weiVecLevel.at(i) = Eigen::VectorXd::Ones(nOLevel.at(i));                               // nOi*1
        taskMatLevel.at(i) = Eigen::MatrixXd::Zero(nOLevel.at(i),nVLevel.at(i));                // nOi*nVi
        taskMatHatLevel.at(i) = Eigen::MatrixXd::Zero(nOLevel.at(i),nVLevel.at(i));;            // nOi*nVi
        taskMatHatInvLevel.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),nOLevel.at(i));          // nVi*nOi
        taskVecLevel.at(i) = Eigen::VectorXd::Zero(nOLevel.at(i));                              // nOi*1
        cstrMatLevel.at(i) = Eigen::MatrixXd::Zero(nCLevel.at(i),nVLevel.at(i));                // nCi*nVi
        cstrMatLevelTrans.at(i) = Eigen::MatrixXd::Zero(nVLevel.at(i),nCLevel.at(i));           // nVi*nCi
        lbCstrLevel.at(i) = Eigen::VectorXd::Zero(nCLevel.at(i));                               // nCi*1
        ubCstrLevel.at(i) = Eigen::VectorXd::Zero(nCLevel.at(i));                               // nCi*1
    }

    return true;
}

bool HqpWbc::hqpUpdate(){
    if(nCChange || nVChange){
        // TODO:
        // createNewQP();
        // resizeQPMatrixVector();

        // delete QP;
        // delete cpuTimePtr;
        // delete [] primalOptPtr;
        // delete [] dualOptPtr;
        // QP = nullptr;
        // cpuTimePtr = nullptr;
        // primalOptPtr = nullptr;
        // dualOptPtr = nullptr;

        createNewQP();
        resizeQPMatrixVector();//directly copied from wqp at 7.6//
    }
    calcHessianGradient();          ///< tasks
    calcConstraintCoefficient();    ///< constraints
    return true;
}

bool HqpWbc::calcHessianGradient(){
    int startRow = 0;
    int iLevel = 0;
    
    for(auto level : priorityTaskNames){
        for(auto item : level){
            auto iter = tasks.find(item);
            taskMatLevel.at(iLevel).block(startRow, 0, iter->second->dim, nVLevel.at(iLevel)) = iter->second->taskMatA;
            taskVecLevel.at(iLevel).segment(startRow, iter->second->dim) = iter->second->taskVecB;
            weiVecLevel.at(iLevel).segment(startRow, iter->second->dim) = iter->second->wei;
            startRow += iter->second->dim;
        }
        startRow = 0;
        iLevel++;
    }
    return true;
}

bool HqpWbc::calcConstraintCoefficient(){
    int startRow = 0;
    int iLevel = 0;
    for(auto level : priorityConstraintNames){
        for(auto item : level){
            auto iter = constraints.find(item);
            cstrMatLevel.at(iLevel).block(startRow, 0, iter->second->dim, nVLevel.at(iLevel)) = iter->second->cstrMatC;
            lbCstrLevel.at(iLevel).segment(startRow, iter->second->dim) = iter->second->lbC;
            ubCstrLevel.at(iLevel).segment(startRow, iter->second->dim) = iter->second->ubC;
            startRow += iter->second->dim;
        }
        startRow = 0;
        iLevel++;
    }
    return true;
}

bool HqpWbc::nspSolve(){//calculate nsp matrix;
    for (int i = 0; i != nLevel; i++){
        if (nOLevel.at(i) == 0){
            if (i == 0){
                nspMat.at(i) = Eigen::MatrixXd::Identity(nVLevel.at(i), nVLevel.at(i));
            }else{
                nspMat.at(i) = nspMat.at(i-1);
            }
        }else{
            if (i == 0){
                taskMatHatInvLevel.at(i) = taskMatLevel.at(i).transpose() * (taskMatLevel.at(i) * taskMatLevel.at(i).transpose()).inverse();//this step takes lots of time;
                nspMat.at(i) = Eigen::MatrixXd::Identity(nVLevel.at(i), nVLevel.at(i)) - taskMatHatInvLevel.at(i) * taskMatLevel.at(i);
            }else{
                taskMatHatLevel.at(i) = taskMatLevel.at(i) * nspMat.at(i - 1);
                taskMatHatInvLevel.at(i) =  taskMatHatLevel.at(i).transpose() * (taskMatHatLevel.at(i) * taskMatHatLevel.at(i).transpose()).inverse();
                nspMat.at(i) = nspMat.at(i -1) * (Eigen::MatrixXd::Identity(nVLevel.at(i), nVLevel.at(i)) - taskMatHatInvLevel.at(i) * taskMatHatLevel.at(i));
            }
        }

    }
    return true;
}

bool HqpWbc::hqpUpdateLeveln(const int & iLevel){//update task and constraint in each level;
    // ---------------------------- Costs/Objects --------------------------------------
    if (iLevel == 0){
        taskMatAll.at(iLevel) = taskMatLevel.at(iLevel);
        taskVecAll.at(iLevel) = taskVecLevel.at(iLevel);
    }else{
        taskMatAll.at(iLevel) = taskMatLevel.at(iLevel) * nspMat.at(iLevel -1);
        taskVecAll.at(iLevel) = taskVecLevel.at(iLevel) - taskMatLevel.at(iLevel) * optVarStar.at(iLevel -1);
    }

    weiVecAll.at(iLevel) = weiVecLevel.at(iLevel);
    taskMatAll.at(iLevel) = weiVecAll.at(iLevel).asDiagonal() * taskMatAll.at(iLevel);
    taskVecAll.at(iLevel) = weiVecAll.at(iLevel).asDiagonal() * taskVecAll.at(iLevel);
         
    /// Hessian matrix, H = A^T * A
    hessianMat.at(iLevel) = taskMatAll.at(iLevel).transpose() * taskMatAll.at(iLevel);
    /// gradient vector, g = - A^T * b
    gradientVec.at(iLevel) = - taskMatAll.at(iLevel).transpose() * taskVecAll.at(iLevel);

    // ------------------------ Bounds & Constraints -----------------------------------
    int  startRow = 0;
    for (int i = 0; i != iLevel + 1; i++){
        cstrMatAll.at(iLevel).block(startRow, 0, nCLevel.at(i), nVLevel.at(i)) = cstrMatLevel.at(i);
        lbCstrAll.at(iLevel).segment(startRow, nCLevel.at(i)) = lbCstrLevel.at(i);
        ubCstrAll.at(iLevel).segment(startRow, nCLevel.at(i)) = ubCstrLevel.at(i);
        startRow += nCLevel.at(i);
    }

    if (iLevel == 0){
        lbCstrAll.at(iLevel) = lbCstrAll.at(iLevel);
        ubCstrAll.at(iLevel) = ubCstrAll.at(iLevel);
        cstrMatAll.at(iLevel)  = cstrMatAll.at(iLevel);
    }else{
        lbCstrAll.at(iLevel) = lbCstrAll.at(iLevel) - cstrMatAll.at(iLevel) * optVarStar.at(iLevel - 1);
        ubCstrAll.at(iLevel) = ubCstrAll.at(iLevel) - cstrMatAll.at(iLevel) * optVarStar.at(iLevel - 1);
        cstrMatAll.at(iLevel)  = cstrMatAll.at(iLevel) * nspMat.at(iLevel -1);//?? the sequence of cstrMat or cstrVec;
    }
    /// the transpose of cstrMatAll
    cstrMatAllTrans.at(iLevel) = cstrMatAll.at(iLevel).transpose();
    return true;
}

bool HqpWbc::hqpSolveLeveln(const int & iLevel){
    if(!initDone.at(iLevel)){
        nWSR.at(iLevel) = nWSRDes.at(iLevel);
        * cpuTimePtr.at(iLevel) = cpuTimeDes.at(iLevel);
        statusCodeSolving.at(iLevel) = QP.at(iLevel)->init(hessianMat.at(iLevel).data(), gradientVec.at(iLevel).data(),
                                      cstrMatAllTrans.at(iLevel).data(),
                                      lowerBoundVec.at(iLevel).data(), upperBoundVec.at(iLevel).data(),
                                      lbCstrAll.at(iLevel).data(), ubCstrAll.at(iLevel).data(),
                                      nWSR.at(iLevel), cpuTimePtr.at(iLevel));
        if(statusCodeSolving.at(iLevel) > 0){
            std::cout << "init QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCodeSolving.at(iLevel)) << std::endl;
            qpResetLeveln(iLevel);
        }else {
            initDone.at(iLevel) = true;
        }
    }else {
        nWSR.at(iLevel) = nWSRDes.at(iLevel);
        * cpuTimePtr.at(iLevel) = cpuTimeDes.at(iLevel);
        statusCodeSolving.at(iLevel) = QP.at(iLevel)->hotstart(hessianMat.at(iLevel).data(), gradientVec.at(iLevel).data(),
                                          cstrMatAllTrans.at(iLevel).data(),
                                          lowerBoundVec.at(iLevel).data(), upperBoundVec.at(iLevel).data(),
                                          lbCstrAll.at(iLevel).data(), ubCstrAll.at(iLevel).data(),
                                          nWSR.at(iLevel), cpuTimePtr.at(iLevel));
        if(statusCodeSolving.at(iLevel) > 0){
            std::cout << "hotstart QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCodeSolving.at(iLevel)) << std::endl;
            qpResetLeveln(iLevel);
        }
    }
    return true;
}

bool HqpWbc::qpResetLeveln(const int & iLevel){
    QP.at(iLevel)->reset();
    initDone.at(iLevel) = false;

    return true;
}

bool HqpWbc::getPrimalOptLeveln(Eigen::VectorXd & primalOpt, const int & iLevel){
    if (primalOpt.size() < nVLevel.at(iLevel)){
        primalOpt.resize(nVLevel.at(iLevel));
    }
    QP.at(iLevel)->getPrimalSolution(primalOptPtr.at(iLevel));
    for(int i = 0; i != nVLevel.at(iLevel); i++)
    {
        primalOpt(i) = static_cast<double>(primalOptPtr.at(iLevel)[i]);
    }
    std::cout << "********* primal opt in level " << iLevel << "******************" << std::endl;
    std::cout << primalOpt.transpose() << std::endl;
    return true;
}

bool HqpWbc::getResultOptLeveln(Eigen::VectorXd &resultOpt, const int & iLevel){
    getPrimalOptLeveln(resultOpt, iLevel);
    return true;
}


bool HqpWbc::getNwsr(std::vector<int> & nWsrRes){
    for (int i = 0; i != nLevel; i++){
        nWsrRes.at(i) = static_cast<int>(nWSR.at(i));
    }
    return true;
}

bool HqpWbc::getSimpleStatusInt(std::vector<int> & simpleStatus){
    for (int i = 0; i != nLevel; i++){
        if (initDone.at(i)){
            simpleStatus.at(i)= 0;
        }else{
            simpleStatus.at(i) = 1;
        }
    }
    return true;
}

bool HqpWbc::getOptCost(std::vector<double> & costOpt){
    for (int i = 0; i != nLevel; i++){
        costOpt.at(i) = static_cast<double>(QP.at(i)->getObjVal()) + 0.5 * taskVecAll.at(i).transpose() * taskVecAll.at(i);
    }
    return true;
}

} // namespace TAICHI
