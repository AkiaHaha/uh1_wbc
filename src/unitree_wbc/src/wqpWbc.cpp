#include "wqpWbc.h"

namespace HUMANOID {
using namespace std;
// ======================================== public Functions ====================================================

WqpWbc::WqpWbc(int dimVar, RobotDynamics * roDy):Wbc(dimVar, roDy){
    createNewQP();
    resizeQPMatrixVector();
}

WqpWbc::WqpWbc(const Wbc & wbc_foo):Wbc(wbc_foo){
    createNewQP();
    resizeQPMatrixVector();
}

// ---------------------- rewrite virtual functions ------------------------

WqpWbc::~WqpWbc(){
    delete QP;
    delete cpuTimePtr;
    delete [] primalOptPtr;
    delete [] dualOptPtr;
    QP = nullptr;
    cpuTimePtr = nullptr;
    primalOptPtr = nullptr;
    dualOptPtr = nullptr;
}

bool WqpWbc::wbcInit(){
    delete QP;
    delete cpuTimePtr;
    delete [] primalOptPtr;
    delete [] dualOptPtr;
    QP = nullptr;
    cpuTimePtr = nullptr;
    primalOptPtr = nullptr;
    dualOptPtr = nullptr;

    createNewQP();
    resizeQPMatrixVector();
    return true;
}

bool WqpWbc::displayResultInformation() const{

    std::cout << "Numbers: " << std::endl
              << "#################   qpOASES  --  QP Basic Numbers   #################" << std::endl
              << "number of variables :                                     " << QP->getNV() << std::endl
              << "number of free variables :                                " << QP->getNFR() << std::endl
              << "number of fixed variables :                               " << QP->getNFX() << std::endl
              << "number of constraints :                                   " << QP->getNC() << std::endl
              << "number of (implicitly defined) equality constraints :     " << QP->getNEC() << std::endl
              << "number of active constraints :                            " << QP->getNAC() << std::endl
              << "number of inactive constraints :                          " << QP->getNIAC() << std::endl;
              
    // If use the follow, set qpOption.printLevel not to be PL_NONE !
    std::cout << "Properties : " << qpOASES::MessageHandling::getErrorCodeMessage(QP->printProperties()) << std::endl;
    std::cout << "Options : " << qpOASES::MessageHandling::getErrorCodeMessage(QP->printOptions()) << std::endl;

    return true;
}

bool WqpWbc::setParametersInt(const std::vector<int> &Parameters){
    if (Parameters.empty()){
        return false;
    }else{
        nWSRDes = Parameters.at(0);
    }
    return true;
}

bool WqpWbc::setParametersDouble(const std::vector<double> &Parameters){
    if (Parameters.empty()){
        return false;
    }else{
        cpuTimeDes = Parameters.at(0);
    }
    return true;
}

bool WqpWbc::getAuxiliaryDataInt(std::vector<int> &auxiliaryData){
    auxiliaryData.resize(2);
    getNwsr(auxiliaryData.at(0));
    getSimpleStatusInt(auxiliaryData.at(1));
    return true;
}

bool WqpWbc::getAuxiliaryDataDouble(std::vector<double> &auxiliaryData){
    auxiliaryData.resize(2);
    getOptCost(auxiliaryData.at(0));
    auxiliaryData.at(1) = static_cast<double>(* cpuTimePtr);
    return true;
}

// ---------------------- implement pure virtual functions -----------------

bool WqpWbc::updateBound(const Eigen::VectorXd &lb, const Eigen::VectorXd &ub){
    if(check(lb, nV)){
        lowerBoundVec = lb;
    }else{
        return false;
    }
    if(check(ub, nV)){
        upperBoundVec = ub;
    }else{
        return false;
    }
    return true;
}

bool WqpWbc::updateRobotDynamics(const Eigen::VectorXd &q, const Eigen::VectorXd &qDot){
    // Provide opportunities for multi-threaded computing
    if (!robot->isCalcWbcDependenceDone()){
        robot->setJntStates(q, qDot);
        robot->calcWbcDependence();
    }
    return true;
}

bool WqpWbc::wbcSolve(){
    qpUpdate();     // update QP solver
    qpSolve();      // solve QP
    return true;
}

bool WqpWbc::getResultOpt(Eigen::VectorXd &resultOpt){
    getPrimalOpt(resultOpt);
    return true;
}


// ======================================== private Functions ====================================================

bool WqpWbc::createNewQP(){

//    qpOption.setToFast();   // It will cause a decrease in accuracy, so choose carefully.
    qpOption.printLevel = qpOASES::PL_NONE;  // PL_MEDIUM, by default
//    qpOption.initialStatusBounds = ST_INACTIVE; // ST_LOWER, by default

    QP = new qpOASES::SQProblem(nV, nC);
    QP->setOptions(qpOption);

    cpuTimePtr = new double(cpuTimeDes);

    primalOptPtr = new qpOASES::real_t[nV];
    dualOptPtr = new qpOASES::real_t[nV + nC];

    nOChange = false;
    nCChange = false;
    nVChange = false;

    initDone = false;

    return true;
}

bool WqpWbc::resizeQPMatrixVector(){
    // ---------------------------- Costs/Objects --------------------------------------
    hessianMat = Eigen::MatrixXd::Zero(nV,nV);                          // nV*nV
    gradientVec = Eigen::VectorXd::Zero(nV);                            // nV*1
    weiVecAll = Eigen::VectorXd::Zero(nO);                              // nO*1
    taskMatAll = Eigen::MatrixXd::Zero(nO,nV);                          // nO*nV
    taskVecAll = Eigen::VectorXd::Zero(nO);                             // nO*1
    // ------------------------ Bounds & Constraints -----------------------------------
    lowerBoundVec = Eigen::VectorXd::Zero(nV);                          // nV*1
    upperBoundVec = Eigen::VectorXd::Zero(nV);                          // nV*1
    cstrMatAll = Eigen::MatrixXd::Zero(nC,nV);                          // nC*nV
    cstrMatAllTrans = Eigen::MatrixXd::Zero(nV,nC);                     // nV*nC
    lbCstrAll = Eigen::VectorXd::Zero(nC);                              // nC*1
    ubCstrAll = Eigen::VectorXd::Zero(nC);                              // nC*1

    return true;
}

bool WqpWbc::qpUpdate(){
    if(nCChange || nVChange){
        delete QP;
        delete cpuTimePtr;
        delete [] primalOptPtr;
        delete [] dualOptPtr;
        QP = nullptr;
        cpuTimePtr = nullptr;
        primalOptPtr = nullptr;
        dualOptPtr = nullptr;

        createNewQP();
        resizeQPMatrixVector();
    }
    calcHessianGradient();          // tasks
    calcConstraintCoefficient();    // constraints
    return true;
}

bool WqpWbc::calcHessianGradient(){
    int startRow = 0;
    for(auto level : priorityTaskNames){
        for(auto item : level){
            auto iter = tasks.find(item);
            // std::cout << "item : " << item << "  " << iter->second->dim << std::endl;//Daniel
            taskMatAll.block(startRow, 0, iter->second->dim, nV) = iter->second->taskMatA;
            taskVecAll.segment(startRow, iter->second->dim) = iter->second->taskVecB;
            weiVecAll.segment(startRow, iter->second->dim) = iter->second->wei;
            startRow += iter->second->dim;

            // cout << "------taskMatLevel Matrix------ " << "dim: " << iter->second->dim << "  " << iter->second->name << endl;
            // cout << iter->second->taskMatA.transpose() << endl;

            // cout << "------taskVecLevel Matrix------ " << iter->second->dim << endl;
            // cout << iter->second->taskVecB.transpose() << endl;

            // cout << "------weiVecLevel Matrix------ " << iter->second->dim << endl;
            // cout << iter->second->wei.transpose() << endl;            
        }
    }
    taskMatAll = weiVecAll.asDiagonal() * taskMatAll;
    taskVecAll = weiVecAll.asDiagonal() * taskVecAll;
    // Hessian matrix, H = A^T * A
    hessianMat = taskMatAll.transpose() * taskMatAll;
    // gradient vector, g = - A^T * b
    gradientVec = - taskMatAll.transpose() * taskVecAll;

    //<---output test Daniel 5.23---//
    // Displaying Hessian matrix dimensions //
    // std::cout << "hessianMat dimensions: " << hessianMat.rows() << "x" << hessianMat.cols() << std::endl;

    // Display the matrix in the requested format //
    // std::cout << "hessianMat contents:" << std::endl;
    // for (int i = 0; i < hessianMat.rows(); ++i) {
    //     for (int j = 0; j < hessianMat.cols(); ++j) {
    //         std::cout << hessianMat(i, j);
    //         if (j < hessianMat.cols() - 1) std::cout << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    //----------------------------->//
    return true;
}

bool WqpWbc::calcConstraintCoefficient(){
    int startRow = 0;
    for(auto level : priorityConstraintNames){
        for(auto item : level){
            auto iter = constraints.find(item);
            // cout << "--------------dim: " << iter->second->dim << endl;
            // cout << "--------------rows: " << iter->second->cstrMatC.rows() << endl;
            // cout << "--------------cols: " << iter->second->cstrMatC.cols() << endl;
            cstrMatAll.block(startRow, 0, iter->second->dim, nV) = iter->second->cstrMatC;
            lbCstrAll.segment(startRow, iter->second->dim) = iter->second->lbC;
            ubCstrAll.segment(startRow, iter->second->dim) = iter->second->ubC;
            startRow += iter->second->dim;
        }
    }
    // the transpose of cstrMatAll
    cstrMatAllTrans = cstrMatAll.transpose();
    return true;
}

bool WqpWbc::qpSolve(){
    if(!initDone){
        nWSR = nWSRDes;
        * cpuTimePtr = cpuTimeDes;
        statusCodeSolving = QP->init(hessianMat.data(), gradientVec.data(),
                                      cstrMatAllTrans.data(),
                                      lowerBoundVec.data(), upperBoundVec.data(),
                                      lbCstrAll.data(), ubCstrAll.data(),
                                      nWSR, cpuTimePtr);
        // cout << "------Hessian Matrix------" << endl;
        // cout << hessianMat.transpose() << endl;

        // cout << "------gradient Vector------" << endl;
        // cout << gradientVec.transpose() << endl;

        // cout << "------cstr Mat AllTrans------" << endl;
        // cout << cstrMatAllTrans.transpose() << endl;

        // cout << "------lower Bound Vec------" << endl;
        // cout << lowerBoundVec.transpose() << endl;

        // cout << "------upper Bound Vec------" << endl;
        // cout << upperBoundVec.transpose() << endl;

        // cout << "------lb CstrAll------" << endl;
        // cout << lbCstrAll.transpose() << endl;

        // cout << "------ub CstrAll------" << endl;
        // cout << ubCstrAll.transpose() << endl;

        if(statusCodeSolving > 0){
            std::cout << "init QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCodeSolving) << std::endl;
            qpReset();
        }else {
            initDone = true;
        }
    }else {
        nWSR = nWSRDes;
        * cpuTimePtr = cpuTimeDes;
        statusCodeSolving = QP->hotstart(hessianMat.data(), gradientVec.data(),
                                          cstrMatAllTrans.data(),
                                          lowerBoundVec.data(), upperBoundVec.data(),
                                          lbCstrAll.data(), ubCstrAll.data(),
                                          nWSR, cpuTimePtr);
        if(statusCodeSolving > 0){
            std::cout << "hotstart QP : " << qpOASES::MessageHandling::getErrorCodeMessage(statusCodeSolving) << std::endl;
            qpReset();
        }
    }
    return true;
}

bool WqpWbc::qpReset(){
    QP->reset();
    initDone = false;
    return true;
}

bool WqpWbc::getPrimalOpt(Eigen::VectorXd & primalOpt){
    if (primalOpt.size() < nV){
        primalOpt.resize(nV);
    }
    QP->getPrimalSolution(primalOptPtr);
    for(int i = 0; i != nV; i++)
    {
        primalOpt(i) = static_cast<double>(primalOptPtr[i]);
    }
    return true;
}

bool WqpWbc::getDualOpt(Eigen::VectorXd & dualOpt){
    if (dualOpt.size() < nV + nC){
        dualOpt.resize(nV + nC);
    }
    QP->getDualSolution(dualOptPtr);
    for(int i = 0; i != nV + nC; i++)
    {
        dualOpt(i) = static_cast<double>(dualOptPtr[i]);
    }
    return true;
}

bool WqpWbc::getNwsr(int & nWsrRes){
    nWsrRes = static_cast<int>(nWSR);
    return true;
}

bool WqpWbc::getSimpleStatusInt(int & simpleStatus){
    if (initDone){
        simpleStatus = 0;
    }else{
        simpleStatus = 1;
    }
    return true;
}

bool WqpWbc::getOptCost(double & costOpt){
    costOpt = static_cast<double>(QP->getObjVal());
    return true;
}

} // namespace HUMANOID
