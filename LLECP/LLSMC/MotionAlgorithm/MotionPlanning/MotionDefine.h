#include<stdio.h>
#include <stdint.h>  
#include<vector>
#include <algorithm> 
#include <iostream>
#include <fstream>
#include<math.h>
typedef std::vector<double> v_double;
typedef std::vector<double> v_int;


//运动过渡帧
struct ST_MotionTransitionFrame
{
    double pos;   
    double vel;  

    ST_MotionTransitionFrame()
        : pos(0.0), vel(0.0){}


    ST_MotionTransitionFrame(double p, double v)
        : pos(p), vel(v){}
};

struct ST_MotionFrame
{
    double pos;   
    double vel;  
    double acc;  
    double jerk;  

    ST_MotionFrame()
        : pos(0.0), vel(0.0),acc(0.0),jerk(0.0){}


    ST_MotionFrame(double p, double v,double a,double j)
        : pos(p), vel(v),acc(a),jerk(j){}
};

//运动约束
struct ST_KinematicLimit
{
    double Vel;   
    double A_acc_max;  
    double J_acc_max;  
    double A_dec_max;  
    double J_dec_max;  
};

enum EN_Para
{
    en_t1 = 0, 
    en_t2, 
    en_t3, 
    en_t4, 
    en_t5,
    en_t6,
    en_t7,
    en_V_s,
    en_A_acc_max = 11,
    en_J_acc_max, 
    en_A_dec_max, 
    en_J_dec_max, 
};