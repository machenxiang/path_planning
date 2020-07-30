#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);
    //cout<<"1"<<endl;
    Px=closedFormCalCoeff1D(Path.col(0),m,d_order,Time);
    Py=closedFormCalCoeff1D(Path.col(1),m,d_order,Time);
    Pz=closedFormCalCoeff1D(Path.col(2),m,d_order,Time);
    //cout<<"2"<<endl;
    for(int i=0;i<m;i++){
        PolyCoeff.row(i).segment(0,p_num1d)=Px.segment(i*p_num1d,p_num1d);//给该行赋x轴系数
        PolyCoeff.row(i).segment(p_num1d,p_num1d)=Py.segment(i*p_num1d,p_num1d);
        PolyCoeff.row(i).segment(p_num1d*2,p_num1d)=Pz.segment(i*p_num1d,p_num1d);
    }
    cout<<"PolyCoeff"<<endl<<PolyCoeff<<endl;
    cout<<"path"<<Path<<endl;

    return PolyCoeff;
}
//coeff的系数应该是从p0，p7
Eigen::MatrixXd TrajectoryGeneratorWaypoint::getQ(int n_seg,int p_num1d,const Eigen::VectorXd &Time){
    Eigen::MatrixXd res= Eigen::MatrixXd::Zero(8*n_seg,8*n_seg);
    for(int i=0;i<n_seg;i++){
        double t=Time(i);
        Eigen::Matrix<double,8,8> Q;
        Q<<  0,0,0,0,0,0,0,0,
             0,0,0,0,0,0,0,0,
             0,0,0,0,0,0,0,0,
             0,0,0,0,0,0,0,0,
             0,0,0,0, 576*t,          1440*pow(t,2),   2880*pow(t,3),   5040*pow(t,4),
             0,0,0,0, 1440*pow(t,2),  4800*pow(t,3),   10800*pow(t,4),  20160*pow(t,5),
             0,0,0,0, 2880*pow(t,3),  10800*pow(t,4),  25920*pow(t,5),  50400*pow(t,6),
             0,0,0,0, 5040*pow(t,4),  20160*pow(t,5),  50400*pow(t,6),  100800*pow(t,7);
        res.block(i*p_num1d,i*p_num1d,p_num1d,p_num1d)=Q;//block用法前两个为行列的起始位置，后面两个为持续长度
    }
    //cout<<"getQ"<<endl<<res<<endl;
    return res;
}

Eigen::MatrixXd TrajectoryGeneratorWaypoint::getM(int n_seg,int p_num1d, const Eigen::VectorXd &Time){
    Eigen::MatrixXd res= Eigen::MatrixXd::Zero(8*n_seg,8*n_seg);
    for(int i=0;i<n_seg;i++){
        double t=Time(i);
        Eigen::Matrix<double,8,8> M;
        M<<1,0,0,0,0,0,0,0,
           0,1,0,0,0,0,0,0,
           0,0,2,0,0,0,0,0,
           0,0,0,6,0,0,0,0,
           1,       t,       pow(t,2),pow(t,3),  pow(t,4),   pow(t,5),   pow(t,6),    pow(t,7),
           0,       1,       2*t,     3*pow(t,2),4*pow(t,3), 5*pow(t,4), 6*pow(t,5),  7*pow(t,6),
           0,       0,       2,       6*t,       12*pow(t,2),20*pow(t,3),30*pow(t,4), 42*pow(t,5),
           0,       0,       0,       6,         24*t,       60*pow(t,2),120*pow(t,3),210*pow(t,4);
        res.block(i*p_num1d,i*p_num1d,p_num1d,p_num1d)=M;
    }
    //cout<<"getM"<<endl<<res<<endl;
    return res;
}
//输入中间点数量，求导阶数
Eigen::MatrixXd TrajectoryGeneratorWaypoint::getCt(int n_seg,int d_order){//选择矩阵，选择已知量，未知量
    
    int num_mid_point=n_seg-1;// number of mid point
    int num_fix_cons=2*4+n_seg-1;//number of fix constraint
    int num_free=3*(n_seg-1);//number of dp
    int rows=8*n_seg;
    int cols=4*n_seg+4;
    Eigen::MatrixXd res=Eigen::MatrixXd::Zero(rows,cols);
    //ct first segment's p v a j
    res.block(0,0,d_order,d_order)=Eigen::MatrixXd::Identity(d_order,d_order);
    //ct last segment's p v a j
    res.block(rows-d_order,num_fix_cons-d_order,d_order,d_order)=Eigen::MatrixXd::Identity(d_order,d_order);

    for(int i=1;i<=num_mid_point;i++){
        //the pos of the mid point,this place write 2 because the value between the point is same
        res(4+(i-1)*d_order*2,4+i-1)=1;
        res(8+(i-1)*d_order*2,4+i-1)=1;
        //the vel of the two point
        res(5+(i-1)*d_order*2,num_fix_cons+(i-1)*3)=1;
        res(9+(i-1)*d_order*2,num_fix_cons+(i-1)*3)=1;
        //the a of the two point 
        res(6+(i-1)*d_order*2,num_fix_cons+1+(i-1)*3)=1;
        res(10+(i-1)*d_order*2,num_fix_cons+1+(i-1)*3)=1;
        //the j of the two point
        res(7+(i-1)*d_order*2,num_fix_cons+2+(i-1)*3)=1;
        res(11+(i-1)*d_order*2,num_fix_cons+2+(i-1)*3)=1;
    }
    //cout<<"getCt"<<endl<<res<<endl;
    return res;
}
Eigen::VectorXd TrajectoryGeneratorWaypoint::closedFormCalCoeff1D(const Eigen::VectorXd &WayPoints1D,
                                                                  const int n_seg,
                                                                  const int d_order,
                                                                  const Eigen::VectorXd &Time)
{
    Eigen::VectorXd res;
   //计算ct矩阵参数
    int num_mid_point=n_seg-1;// number of mid point
    int num_fix_cons=2*4+n_seg-1;//number of fix constraint
    int num_free=3*(n_seg-1);//number of dp
    int rows=8*n_seg;
    int cols=4*n_seg+4;
    int p_num1d=d_order*2;

    //计算R矩阵
    Eigen::MatrixXd Ct=getCt(n_seg,d_order);
    Eigen::MatrixXd C=Ct.transpose();
    Eigen::MatrixXd M=getM(n_seg,p_num1d,Time);
    Eigen::MatrixXd M_inv=M.inverse();
    Eigen::MatrixXd M_inv_T=M_inv.transpose();
    Eigen::MatrixXd Q=getQ(n_seg,p_num1d,Time);
    Eigen::MatrixXd R=C*M_inv_T*Q*M_inv*Ct;
    //cout<<"R"<<endl<<R<<endl;
    cout<<"ct"<<endl<<Ct<<endl;


    //计算分块矩阵
    Eigen::MatrixXd R_pp=R.block(num_fix_cons,num_fix_cons,num_free,num_free);
    //cout<<"R_pp"<<endl<<R_pp<<endl;

    Eigen::MatrixXd R_fp=R.block(0,num_fix_cons,num_fix_cons,num_free);
    //cout<<"R_fp"<<endl<<R_fp<<endl;
    //计算dF
    Eigen::VectorXd dF(num_fix_cons);
    Eigen::VectorXd start_state(4);
    start_state<<0,0,0,0;

    Eigen::VectorXd end_state(4);
    end_state<<WayPoints1D.row(n_seg),0,0,0;//end state 的位置
    
    dF.head(d_order)=start_state;
    dF.segment(d_order,num_mid_point)=WayPoints1D.segment(1,WayPoints1D.rows()-2);// rows是求行数，row是指特定某一行
    dF.tail(d_order)=end_state;
    cout<<"dF"<<endl<<dF<<endl;
    
    Eigen::VectorXd dP=-R_pp.inverse()*R_fp.transpose()*dF;
    //cout<<"dP"<<endl<<dP<<endl;

    Eigen::VectorXd dF_dP(cols);
    dF_dP<<dF,dP;
    //cout<<"dF_dP"<<endl<<dF_dP<<endl;

    //计算单轴的coeff
    res=M_inv*Ct*dF_dP;
    //cout<<"res"<<endl<<res<<endl;
    return res;
}
        

