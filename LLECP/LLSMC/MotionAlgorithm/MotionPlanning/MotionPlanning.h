#include"Luolimath.h"
class MotionPlanning
{
private:
    /* data */
public:
    MotionPlanning(/* args */);
    ~MotionPlanning();

    //在PLCOpen中开始和停止的速度约束是一致的，因此不做区分，如有需要则重新计算
    static int TrajectoryCalculation(double dCycle,ST_KinematicLimit stKinematicLimit,std::vector<ST_MotionTransitionFrame>& v_TransitionMotionFrame,std::vector<ST_MotionFrame>& v_Trajectory);

private:
    //%%%%%%%%%%%%% 基于S型加减速的分段速度校验 //%%%%%%%%%%%%%
    ////by 蒋宇航 2025.7.20 00：30
    // 输入参数:  曲线段端点的速度
    //           分段曲线的弧长
    //           运动约束
    // 输出参数：更新之后的各段端点的速度
    //           更新之后各段曲线能达到的最大速度
    static int SectionalSpeedVerification(v_double v_Segmentation_P,v_double v_Segmentation_V,ST_KinematicLimit stKinematicLimit,v_double& v_Vmax,v_double& v_Vel);

    //%%%%%%%%%%%%% 非对称S型曲线固定长度参数校正 //%%%%%%%%%%%%%
    //by 蒋宇航 2025.7.20 5：30
    // 根据给定的系统参数（A_max,J_max）和轨迹参数（V_s, V_e,V_max，S_seg）
    // 求解S曲线各时间段T1,T2,T3,T4,T5,T6,T7
    // 计算步骤: 1. 通过校验起点速度终点速度可达性与修正（在给定轨迹段长度小于系统从起始点速度运动到终点速度所需最短轨迹段长的情况下
    //			  采用盛金公式修正起点速度和终点速度）；
    //		    2. 校验最大加速度和最大减速度可达性；
    //			3. 校验全程最大速度匀速运行可能性；
    //			4. 校验最大速度可达性；
    //			5. 二分法修正最大速度
    // 输入参数: V_max -- 校验最大速度
    //           运动约束
    //           V_s -- 起点速度
    //           V_e -- 终点速度
    //           S_seg -- 轨迹段长度
    // 输出参数：修正后的V_s，V_e，V_max，A_max，J_max
    //           求解后的时间T1,T2,T3,T4,T5,T6,T7
    static int AsymmetricSCurvesFixedLengthParametersCorrection(double V_s, double V_e,double v_Segmentation_V,ST_KinematicLimit stKinematicLimit, double S_seg,v_double& para);

    //%%%%%%%%%%%%%%% 非对称曲线位移速度加速度加加速度计算 //%%%%%%%%%%%%%%
    //by 蒋宇航 2025.7.20 6：30
    //           根据给定的系统参数（A_max,J_max）和轨迹参数para给到（V_s, V_e,V_max，S_seg，T1、T2、T3、T4、T5、T6、T7）
    //           求解S曲线每个控制周期内的位移、速度、加速度、加加速度
    // 输入参数: t -- 某个离散时间，0 <= t <= Tf
    //           以下参数以para给到
    //              vTs -- S曲线各时间段T1、T2、T3、T4、T5、T6、T7。总时间Tf = T1 + T2 + T3 + T4 + T5 + T6 + T7.
    //              A_acc_max -- 加速运动阶段的最大加速度
    //              J_acc_max -- 加速运动阶段的最大加加速度
    //              A_dec_max -- 减速运动阶段的最大加速度
    //              J_dec_max -- 减速运动阶段的最大加加速度
    //              V_s -- 轨迹初始速度
    // 输出参数：st：位移； vel：速度；acc： 加速度； Jerk：加加速度
    static int AsymmetricSCurveTimeScaling(double t,v_double para,ST_MotionFrame& stMotionFrame);

};


inline int TrajectoryCalculation(double dCycle, ST_KinematicLimit stKinematicLimit,
                                 std::vector<ST_MotionTransitionFrame>& v_TransitionMotionFrame,
                                 std::vector<ST_MotionFrame>& v_Trajectory)
{
    return MotionPlanning::TrajectoryCalculation(dCycle, stKinematicLimit, v_TransitionMotionFrame, v_Trajectory);
}