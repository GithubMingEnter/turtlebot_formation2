#pragma once
#include <casadi/casadi.hpp>
#include<iostream>
#include<eigen3/Eigen/Dense>


using dataT=double;
using Vec3d=Eigen::Matrix<dataT,3,1>;
using Vec2d=Eigen::Matrix<dataT,2,1>;
using Vec2ds=std::vector<Vec2d>;
using Vec3ds=std::vector<Vec3d>;
struct mpc_param{
    dataT vMax;
    dataT vMin;
    dataT wMax;
    dataT wMin;
    std::vector<dataT> Qvec;
    std::vector<dataT> Rvec;
    std::vector<dataT> Svec;
    bool isObs=false;
    dataT obsSoftRatio; 
    
    void operator&=(const mpc_param& rhs )
    {
        vMax=rhs.vMax;
        vMin=rhs.vMin;
        wMax=rhs.wMax;
        wMin=rhs.wMin;
        Qvec=rhs.Qvec;
        Rvec=rhs.Rvec;
        Svec=rhs.Svec;
        isObs=rhs.isObs;
        obsSoftRatio=rhs.obsSoftRatio;
    }

};
class NmpcPosCtrl
{
    
private:
    mpc_param mParas;
    int mPredictStep;
    dataT mSampleTime; //note dataT sample time 
    int mPreStage;//store times that call nmpc
    int mStates; // x,y,theta
    int mOutputs;// v,w
    std::vector<Vec3d> mPreTraj;
    std::vector<dataT> mInitialGuess;//initial solver
    Vec3d mGoalState;
    casadi::Function mSolver; 
    casadi::Function mPreFun; // predict function
    std::map<std::string, casadi::DM> mRes; //solver result
    std::map<std::string , casadi::DM> mArgs;

    Eigen::Matrix<dataT,2,1> mCtrlCommand;
    std::vector<dataT> mLbx;
    std::vector<dataT> mUbx;
    casadi::SX mQs;
    casadi::SX mRo;
    casadi::SX mSo;
    casadi::SX mInput_smoothness_cost;
    casadi::SX mObsSoft_cost;
    double mIgnoreDist;
    Vec3ds obs_list;
    bool mIsWarmSt=true;
    

public: 
    NmpcPosCtrl(int preStep,dataT sampleTime,mpc_param& mpcParam);
    ~NmpcPosCtrl();
    void SetSolver();
    inline void SetObs(Vec3ds& obsl){obs_list=obsl;};
    void SetGoalStates(Vec3d goalStates);
    void SetInput(double vmax,double vmin,double wmax,double wmin);

    void Set_CAM_nmpc_solver();
    //optimize
    void Optimize(Vec3d & curStates);

    void ComputeCommand(Vec2d& command);

    void GetPreTraj(std::vector<Vec3d> & preTraj);
    inline void  SetIsWarmSt(bool pd){mIsWarmSt=pd;}

private:
    void addSoftObsCost(casadi::SX& X_);
    void addInputSmoothnessCost(casadi::SX& U_);
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};





