#pragma once
#include <vector>
#include <stdint.h> 
#include <string.h>
#include<time.h>
#include <cstdlib>
#include <cstdio>
#include <math.h>
struct ST_SMC_PDO_Virtual
{
    uint16_t   Controlword;
    uint16_t   StatusWord;
    uint16_t   ErrorCode;
    int32_t    TargetPosition;
    int32_t    TargetVelocity;
    int16_t    TargetTorque;
    uint8_t    TargetModesOfOperation;
    int32_t    ActualPosition;
    int32_t    ActualVelocity;
    int16_t    ActualTorque;
    uint8_t    ActualModesOfOperation;
    uint32_t   DigitalInputs;
    uint32_t   DigitalOutputs;
};

//该映射遵照CanOpen402协议
struct ST_SMCInitMap
{
    uint16_t*   pControlword;
    uint16_t*   pStatusWord;
    uint16_t*   pErrorCode;
    int32_t*    pTargetPosition;
    int32_t*    pTargetVelocity;
    int16_t*    pTargetTorque;
    uint8_t*    pTargetModesOfOperation;
    int32_t*    pActualPosition;
    int32_t*    pActualVelocity;
    int16_t*    pActualTorque;
    uint8_t*    pActualModesOfOperation;
    uint32_t*   pDigitalInputs;
    uint32_t*   pDigitalOutputs;    
    ST_SMCInitMap()
    {
        pStatusWord = nullptr;
        pErrorCode = nullptr;
        pTargetPosition = nullptr;
        pTargetVelocity = nullptr;
        pTargetTorque = nullptr;
        pTargetModesOfOperation = nullptr;
        pActualPosition = nullptr;
        pActualVelocity = nullptr;
        pActualTorque = nullptr;
        pActualModesOfOperation = nullptr;
        pDigitalInputs = nullptr;
        pDigitalOutputs = nullptr;
    }
};
//规划模式
enum EN_PlanningMode
{
    //0
    enPlanningModeNull = 0,//无模式
    enPosition = 1,//位置模式
    enVelocity = 2,//速度模式
    enAcceleration = 3, //加速度模式

    enStop = 99 //停止模式
};
// 运动方向
enum EN_Direction
{
    enPositive = 0,   // 正方向
    enNegative = 1,   // 负方向
    enCurrent  = 2    // 保持当前方向（有些厂商支持）
};

// 缓冲模式
enum EN_BufferMode
{
    enAborting = 0,   // 立即中断当前运动，执行新指令
    enBuffered = 1,   // 缓冲，等前一运动完成后再执行
    enBlendingLow = 2, // 融合过渡，低优先级平滑衔接
    enBlendingHigh = 3 // 融合过渡，高优先级平滑衔接
};

struct ST_PlanningMotionParam
{
    double pos;
    double vel;
    double acc;
    double dec;
    double jerk;
    EN_PlanningMode PlanningMode;
    EN_Direction  Direction;
    // == 运算符重载
    bool operator==(const ST_PlanningMotionParam& other) const
    {
        auto eq = [](double a, double b) { return std::fabs(a - b) < 1e-9; };

        return eq(pos,  other.pos)  &&
               eq(vel,  other.vel)  &&
               eq(acc,  other.acc)  &&
               eq(dec,  other.dec)  &&
               eq(jerk, other.jerk) &&
               (PlanningMode == other.PlanningMode) &&
               (Direction    == other.Direction);
    }
    bool operator!=(const ST_PlanningMotionParam& other) const
    {
        return !(*this == other);
    }
};

struct ST_MotionUint
{
    ST_PlanningMotionParam  PlanningMotionParam;
    EN_BufferMode BufferMode;
    void* fbID;//功能块指针
    bool bMotion;
    bool bMotionDone;
};



struct ST_SoftMotionData
{
    ST_SMC_PDO_Virtual stSoftMotionPDO;
    std::vector<ST_MotionUint>vMotionUint;
};


struct ST_SMCAxisConfiguration
{
    double dGearRatio;
    uint32_t nEncodeRatio;
    int nEncodeDirection;
    int nEncodeHomePos;
    double dCurrentScales;
    double dTorqueFeedforwardScale;
    double dVelocityScale;
    int nCurrentDirection;
    double dPositive;
    double dNegative;
    ST_SMCAxisConfiguration()
    {
        dGearRatio = 0;
        nEncodeRatio = 0;
        nEncodeDirection = 0;
        nEncodeHomePos = 0;
        dCurrentScales = 0;
        nCurrentDirection = 0;
        dTorqueFeedforwardScale =0;
        dNegative = 0;
        dPositive = 0;
    }
};


enum EN_ModesOfOperation
{
    enModeManufacturerSpecificMin = -128,// 厂商自定义模式起始值

    enModeNoModeAssigned = 0,            // 无模式变化/未指定模式
    enModeProfilePosition = 1,           // 位置模式
    enModeVelocity = 2,                  // 速度模式
    enModeProfileVelocity = 3,           // 简单速度模式
    enModeProfileTorque = 4,             // 扭矩模式
    enModeReserved5 = 5,                 // 保留
    enModeHoming = 6,                    // 回零模式
    enModeInterpolatedPosition = 7,      // 插值位置模式
    enModeCyclicSyncPosition = 8,        // 同步位置模式（周期）
    enModeCyclicSyncVelocity = 9,        // 同步速度模式（周期）
    enModeCyclicSyncTorque = 10,         // 同步扭矩模式（周期）

    enModeManufacturerSpecificMax = 127  // 厂商自定义最大值
    // 11 到 127 保留
};

//OpenPLC标准
enum EN_AxisMotionState
{
    motionState_power_off           = 0,  // Disabled
    motionState_errorstop           = 1,  // Error Stop
    motionState_stopping            = 2,  // Stopping
    motionState_standstill          = 3,  // Standstill
    motionState_discrete_motion     = 4,  // 单步运动
    motionState_continuous_motion   = 5,  // 连续运动
    motionState_synchronized_motion = 6,  // 同步运动
    motionState_homing              = 7   // 回零
};


struct ST_SMCAxisMotionData
{
    int32_t nTarPosCount;
    int32_t nActPosCount;
    int32_t nTarCurCount;
    int32_t nActCurCount;
    double dTarPos;
    double dTarVel;
    //double dTarAcc;
    double dActPos;
    double dActVel;
   // double dActAcc;
    double dTarCur;
    double dActCur;
    //************************** */
    double dTarVel_c;
    double dTarAcc_c;
    double dTarJerk_c;
    double dActVel_c;
    double dActAcc_c;
    double dActJerk_c;
    EN_ModesOfOperation snTarMOO;
    EN_ModesOfOperation snActMOO;
    ST_SMCAxisMotionData()
    {
        dTarVel = 0;
        //dTarAcc = 0;
        dActVel = 0;
        //dActAcc = 0;
        dTarVel_c = 0;
        dTarAcc_c = 0;
        dActVel_c = 0;
        dActAcc_c = 0;
        dActJerk_c = 0;
        dActJerk_c = 0;
        dTarCur = 0;
        dActCur = 0;
    }
};

class SMCTimer 
{
public:
    bool bInit_IN;
    long nInit_PT = 0;  // 以微秒为单位
    struct timespec nInit_ET = {0, 0};

	bool bLast_CLK = false;
public:
	// 定时器
    bool Ton(bool IN, int PT, int& ET) 
    {
        if (!IN)
        {
            ET = 0;
            bInit_IN = false;
            nInit_PT = 0;
            nInit_ET = {0, 0};
            return false;
        }
        // 检测上升沿
        if (IN && !bInit_IN)
        {
            nInit_PT = PT;
            clock_gettime(CLOCK_MONOTONIC, &nInit_ET);
        }
        if (IN)
        {
            bInit_IN = IN;
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            // 计算经过的时间（微秒）
            long elapsed = (now.tv_sec - nInit_ET.tv_sec) * 1000000 + 
                           (now.tv_nsec - nInit_ET.tv_nsec) / 1000;
            ET = elapsed;
            return ET > nInit_PT;
        }
        return false;
    }

	bool Ton(bool IN, int PT) 
    {
		int ET = 0;
        if (!IN)
        {
            bInit_IN = false;
            nInit_PT = 0;
            nInit_ET = {0, 0};
            return false;
        }
        // 检测上升沿
        if (IN && !bInit_IN)
        {
            nInit_PT = PT;
            clock_gettime(CLOCK_MONOTONIC, &nInit_ET);
        }
        if (IN)
        {
            bInit_IN = IN;
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            // 计算经过的时间（微秒）
            long elapsed = (now.tv_sec - nInit_ET.tv_sec) * 1000000 + 
                           (now.tv_nsec - nInit_ET.tv_nsec) / 1000;
            ET = elapsed;
            return ET > nInit_PT;
        }
        return false;
    }
	// 上升沿检测
	bool R_TRIG(bool CLK)
	{
		if (CLK && !bLast_CLK)
		{
			bLast_CLK = CLK;
			return true;
		}
		else
		{
			bLast_CLK = CLK;
			return false;
		}
	}
	// 下降沿检测
    bool F_TRIG(bool CLK)
    {
        if (!CLK && bLast_CLK) // 从 true 变为 false
        {
            bLast_CLK = CLK;
            return true;
        }
        else
        {
            bLast_CLK = CLK;
            return false;
        }
    }
};

