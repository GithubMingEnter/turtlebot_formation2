#include"nmpc_ctrl.h"

NmpcPosCtrl::NmpcPosCtrl(int preStep, dataT sampleTime,mpc_param& mpcParam)
{
    mPredictStep = preStep;
    mSampleTime = sampleTime;
    mParas=mpcParam;
    
    // TODO ?
    mPreStage = 0;
    mStates = 3;
    mOutputs = 2;
    mGoalState = Vec3d::Zero();
    // mInitialGuess.resize(mPredictStep*mOutputs, 0); //note
    for(int i=0;i<mPredictStep;i++){
            mInitialGuess.push_back(0);
            mInitialGuess.push_back(0);
        } 
    
}
NmpcPosCtrl::~NmpcPosCtrl()
{
}
void NmpcPosCtrl::SetInput(double vmax,double vmin,double wmax,double wmin)
{
    mParas.vMax = vmax;
    mParas.vMin = vmin;
    mParas.wMax = wmax;
    mParas.wMin = wmin;
    for (int k = 0; k < mPredictStep; ++k)
    {
        mLbx.push_back(mParas.vMin);
        mUbx.push_back(mParas.vMax);
    }
    for (int k = 0; k < mPredictStep; ++k)
    {
        mLbx.push_back(mParas.wMin);
        mUbx.push_back(mParas.wMax);
    }

}
void NmpcPosCtrl::Set_CAM_nmpc_solver(){
     // build model
    // using casadi::SX = casadi::SX;
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::SX theta = casadi::SX::sym("theta");
    casadi::SX v = casadi::SX::sym("v");
    casadi::SX omega = casadi::SX::sym("omega");

    casadi::SX states = casadi::SX::vertcat({x, y, theta});
    casadi::SX outputs = casadi::SX::vertcat({v, omega});

    // kinematic
    casadi::SX rhs = casadi::SX::vertcat({v * casadi::SX::cos(theta)+omega*0.46*casadi::SX::sin(theta),
                                          -v * casadi::SX::sin(theta)+omega*0.46*casadi::SX::cos(theta),
                                          omega});

    // define model function
    casadi::Function modelFunc = casadi::Function("f", {states, outputs}, {rhs});

    // signal expression about solver problem
    casadi::SX U = casadi::SX::sym("U", mOutputs, mPredictStep);    // output
    casadi::SX X = casadi::SX::sym("X", mStates, mPredictStep + 1); // state
    // current states
    casadi::SX current_states = casadi::SX::sym("current_states", mStates);
    // optimal parameters ,需要给出当前运动状态和预测视野内的参考轨迹,先测试定点控制,则参考轨迹可设置为期望终点）
    casadi::SX opt_para = casadi::SX::sym("opt_para", 2 * mStates);
    // optimal variables
    // note u need to be transpose
    casadi::SX opt_var = casadi::SX::reshape(U.T(), -1, 1);
    // predict state
    //  casadi::Slice(int start, int end, int step=1)
    X(casadi::Slice(), 0) = opt_para(casadi::Slice(0, 3, 1)); // state initial value

    for (int i = 0; i < mPredictStep; ++i)
    {
        std::vector<casadi::SX> input_X;
        casadi::SX X_current = X(casadi::Slice(), i);
        casadi::SX U_current = U(casadi::Slice(), i);
        input_X.push_back(X_current);
        input_X.push_back(U_current);
        X(casadi::Slice(), i + 1) = modelFunc(input_X).at(0) *mSampleTime + X_current;
    }
    // predict function
    //"function name" ,in ,out ,
    mPreFun = casadi::Function("mPreFun",
            {casadi::SX::reshape(U, -1, 1), opt_para}, {X});

    // penalty matrix
    
    mQs=casadi::SX::diag(casadi::SX(mParas.Qvec));
    mRo = casadi::SX::diag(casadi::SX(mParas.Rvec));
    mSo = casadi::SX::diag(casadi::SX(mParas.Svec));


    casadi::SX cost_fun = casadi::SX::sym("cost_fun");
    cost_fun = 0;
    int trajPointIdx = 0;
    std::cout<<"cal cost function"<<std::endl;

    for (int k = 0; k < mPredictStep; k++)
    {
        casadi::SX states_err = X(casadi::Slice(), k) - opt_para(casadi::Slice(3, 6, 1));
        casadi::SX controls_err = U(casadi::Slice(), k);
        cost_fun = cost_fun + casadi::SX::mtimes({states_err.T(), mQs, states_err}) +
                   casadi::SX::mtimes({controls_err.T(), mRo, controls_err});
    }
    addInputSmoothnessCost(U);
    cost_fun=cost_fun+mInput_smoothness_cost;
    std::cout<<"build solver"<<std::endl;
    // build solver
    // note map relationship
    casadi::SXDict nlp_prob = {
        {"f", cost_fun},
        {"x", opt_var},
        {"p", opt_para}};
    std::string solverName = "ipopt";
    casadi::Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 6000;
    nlp_opts["ipopt.print_level"] = 0;
    nlp_opts["print_time"] = 0;
    nlp_opts["ipopt.acceptable_tol"] = 1e-6;            // convergence criterion for solution
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4; // change tolerance in the objective function
    std::cout<<"finish "<<std::endl;
    /* solver name , problem structure ,parameters */
    mSolver = nlpsol("nlpsol", solverName, nlp_prob, nlp_opts);

    std::cout<<"cal cost function "<<std::endl;
}

void NmpcPosCtrl::SetSolver()
{
    // build model
    // using casadi::SX = casadi::SX;
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::SX theta = casadi::SX::sym("theta");
    casadi::SX v = casadi::SX::sym("v");
    casadi::SX omega = casadi::SX::sym("omega");

    casadi::SX states = casadi::SX::vertcat({x, y, theta});
    casadi::SX outputs = casadi::SX::vertcat({v, omega});

    // kinematic
    casadi::SX rhs = casadi::SX::vertcat({v * casadi::SX::cos(theta),
                            v * casadi::SX::sin(theta),
                            omega});

    // define model function
    casadi::Function modelFunc = casadi::Function("f", {states, outputs}, {rhs});

    // signal expression about solver problem
    casadi::SX U = casadi::SX::sym("U", mOutputs, mPredictStep);    // output
    casadi::SX X = casadi::SX::sym("X", mStates, mPredictStep + 1); // state
    // current states
    casadi::SX current_states = casadi::SX::sym("current_states", mStates);
    // optimal parameters ,需要给出当前运动状态和预测视野内的参考轨迹,先测试定点控制,则参考轨迹可设置为期望终点）
    casadi::SX opt_para = casadi::SX::sym("opt_para", 2 * mStates);
    // optimal variables
    // note u need to be transpose
    casadi::SX opt_var = casadi::SX::reshape(U.T(), -1, 1);
    // predict state
    //  casadi::Slice(int start, int end, int step=1)
    X(casadi::Slice(), 0) = opt_para(casadi::Slice(0, 3, 1)); // state initial value

    for (int i = 0; i < mPredictStep; ++i)
    {
        std::vector<casadi::SX> input_X;
        casadi::SX X_current = X(casadi::Slice(), i);
        casadi::SX U_current = U(casadi::Slice(), i);
        input_X.push_back(X_current);
        input_X.push_back(U_current);
        X(casadi::Slice(), i + 1) = modelFunc(input_X).at(0) *mSampleTime + X_current;
    }
    // predict function
    //"function name" ,in ,out ,
    mPreFun = casadi::Function("mPreFun",
            {casadi::SX::reshape(U, -1, 1), opt_para}, {X});

    // penalty matrix
    mQs=casadi::SX::diag(casadi::SX(mParas.Qvec));
    mRo = casadi::SX::diag(casadi::SX(mParas.Rvec));
    mSo = casadi::SX::diag(casadi::SX(mParas.Svec));

    casadi::SX cost_fun = casadi::SX::sym("cost_fun");
    cost_fun = 0;
    int trajPointIdx = 0;
    std::cout<<"cal cost function"<<std::endl;

    for (int k = 0; k < mPredictStep; k++)
    {
        casadi::SX states_err = X(casadi::Slice(), k) - opt_para(casadi::Slice(3, 6, 1));
        casadi::SX controls_err = U(casadi::Slice(), k);
        cost_fun = cost_fun + casadi::SX::mtimes({states_err.T(), mQs, states_err}) +
                   casadi::SX::mtimes({controls_err.T(), mRo, controls_err});
    }
    addInputSmoothnessCost(U);
    cost_fun=cost_fun+mInput_smoothness_cost;
    std::cout<<"build solver"<<std::endl;
    // build solver
    // note map relationship
    casadi::SXDict nlp_prob = {
        {"f", cost_fun},
        {"x", opt_var},
        {"p", opt_para}};
    std::string solverName = "ipopt";
    casadi::Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 5000;
    nlp_opts["ipopt.print_level"] = 0;
    nlp_opts["print_time"] = 0;
    nlp_opts["ipopt.acceptable_tol"] = 1e-6;            // convergence criterion for solution
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4; // change tolerance in the objective function
    std::cout<<"finish "<<std::endl;
    /* solver name , problem structure ,parameters */
    mSolver = nlpsol("nlpsol", solverName, nlp_prob, nlp_opts);
    std::cout<<"cal cost function"<<std::endl;

}

void NmpcPosCtrl::addInputSmoothnessCost(casadi::SX& U_)
{
    /*
     * @brief add input smoothness cost
     * cost = sum((u - u_prev)^2 + (v - v_prev)^2)
     */
    mInput_smoothness_cost = 0;

    for (int k = 0; k < mPredictStep - 1; k++)
    {
        //casadi::Slice() all
        mInput_smoothness_cost += casadi::SX::mtimes(casadi::SX::mtimes((U_(casadi::Slice(), k + 1) - U_(casadi::Slice(), k)).T(), mSo),
                                                         U_(casadi::Slice(), k + 1) - U_(casadi::Slice(), k));
    }
}
void NmpcPosCtrl::SetGoalStates(Vec3d goalStates)
{
    mGoalState = goalStates;
}

// optimize
void NmpcPosCtrl::Optimize(Vec3d &curStates)
{
    mPreStage++;
    std::vector<dataT> curGoalStates;
    // set control constraints
    
    for (int j = 0; j < 3; j++)
    {
        curGoalStates.push_back(curStates(j));
    }
    for (int j = 0; j < 3; j++)
    {
        curGoalStates.push_back(mGoalState[j]);
    }

    // set solver parameters
    mArgs["lbx"] = mLbx;
    mArgs["ubx"] = mUbx;
    mArgs["x0"] = mInitialGuess; 
    mArgs["p"] = curGoalStates; //map nlp_prob
    // solver
    mRes = mSolver(mArgs);
    std::cout<<"objective: "<<mRes.at("f")<<std::endl;
    // std::cout<<"solution: " <<mRes.at("x")<<std::endl;
    // get optimal variable
    std::vector<dataT> resControlAll(mRes.at("x"));
    std::vector<dataT> resControlV, resControlW;
    resControlV.assign(resControlAll.begin(), resControlAll.begin() + mPredictStep);
    // note no resControlAll.end()
    resControlW.assign(resControlAll.begin() + mPredictStep, resControlAll.begin() + 2 * mPredictStep);

    // store next time guess solver
    std::vector<dataT> initialGuess;
    
    if(mIsWarmSt)
    {
        for (int j = 0; j < mPredictStep - 1; j++)
        {
            initialGuess.push_back(resControlV.at(j));
        }
        initialGuess.push_back(resControlV.at(mPredictStep-1));
        for (int j = 0; j <= mPredictStep - 1; j++)
        {
            initialGuess.push_back(resControlW.at(j));
        }
    }
    else{
        initialGuess.resize(mPredictStep*mOutputs);
    }


    mInitialGuess = initialGuess;
    // current output
    mCtrlCommand << resControlV.front(), resControlW.front();
    std::cout<<"mCtrlCommand = "<<mCtrlCommand<<std::endl;
    std::vector<Vec3d> predictX;
    predictX.push_back(curStates);
    for (int j = 0; j < mPredictStep; j++)
    {
        Vec3d NextX;
        NextX << mSampleTime * resControlV[j] * std::cos(predictX[j](2)) + predictX[j](0),
            mSampleTime * resControlV[j] * std::sin(predictX[j](2)) + predictX[j](1),
            mSampleTime * resControlW[j] + predictX[j](2);
        predictX.push_back(NextX);
    }
    mPreTraj = predictX;
}

void NmpcPosCtrl::ComputeCommand(Vec2d &command)
{
    command = mCtrlCommand;
}

void NmpcPosCtrl::GetPreTraj(std::vector<Vec3d> &preTraj)
{
    preTraj = mPreTraj;
}
