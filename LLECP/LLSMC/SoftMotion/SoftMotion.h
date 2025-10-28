#pragma once
#include<vector>
#include"../CIA402Axis/CIA402Axis.h"
#include"motion_algorithm/motion_planning/motion_planning.h"
#define MaxSnap 20000000

struct SoftMotionPlanParams;


class SoftMotion
{
protected:
    std::vector<CIA402Axis*>m_v_Axis;
    uint16_t nAxisCount;
    double m_dSoftMotionCycle;
    std::vector<SoftMotionPlanParams> m_vSoftMotionPlanParams;
protected:
    void PDOsynchronization();
    void AxisRealTime();
    void SoftMotionPlanner_RT();
    int AxisMotionPlanner(CIA402Axis* pAxis);
public:
    SoftMotion(std::vector<CIA402Axis*> v_Axis);
    ~SoftMotion();
    void SoftMotionRun();
    int SetSoftMotionCycle(double dCycle);

// ===== 写入（RxPDO） =====
    int SoftMotion_PDO_SetControlword(CIA402Axis* Axis_REF,uint16_t Controlword);
    int SoftMotion_PDO_SetTargetPosition(CIA402Axis* Axis_REF,int32_t TargetPosition);
    int SoftMotion_PDO_SetTargetVelocity(CIA402Axis* Axis_REF,int32_t TargetVelocity);
    int SoftMotion_PDO_SetTargetTorque(CIA402Axis* Axis_REF,int16_t TargetTorque);
    int SoftMotion_PDO_SetModesOfOperation(CIA402Axis* Axis_REF,EN_ModesOfOperation mode);
    int SoftMotion_PDO_SetDigitalOutputs(CIA402Axis* Axis_REF,uint32_t DigitalOutputs);

    // ===== 读取（TxPDO） =====
    int SoftMotion_PDO_ReadStatusWord(CIA402Axis* Axis_REF,uint16_t& StatusWord);
    int SoftMotion_PDO_ReadErrorCode(CIA402Axis* Axis_REF,uint16_t& ErrorCode);
    int SoftMotion_PDO_ReadActualPosition(CIA402Axis* Axis_REF,int32_t& ActualPosition);
    int SoftMotion_PDO_ReadActualVelocity(CIA402Axis* Axis_REF,int32_t& ActualVelocity);
    int SoftMotion_PDO_ReadActualTorque(CIA402Axis* Axis_REF,int16_t& ActualTorque);
    int SoftMotion_PDO_ReadModesOfOperation(CIA402Axis* Axis_REF,EN_ModesOfOperation& mode);
    int SoftMotion_PDO_ReadDigitalInputs(CIA402Axis* Axis_REF,uint32_t& inputs);
    int SoftMotion_PDO_ReadDigitalOutputs(CIA402Axis* Axis_REF,uint32_t& outputs);

    // ===== 读回目标值缓存或虚拟轴调试 =====
    int SoftMotion_PDO_ReadControlword(CIA402Axis* Axis_REF,uint16_t& Controlword);
    int SoftMotion_PDO_ReadTargetPosition(CIA402Axis* Axis_REF,int32_t& TargetPosition);
    int SoftMotion_PDO_ReadTargetVelocity(CIA402Axis* Axis_REF,int32_t& TargetVelocity);
    int SoftMotion_PDO_ReadTargetTorque(CIA402Axis* Axis_REF,int16_t& TargetTorque);
    int SoftMotion_PDO_ReadTargetModesOfOperation(CIA402Axis* Axis_REF,uint8_t& mode);
};


struct SoftMotionPlanParams
{
    //规划参数
    ST_PlanParams stActParam;
    ST_PlanData trackData;

    //当前运动帧参数
    ST_InterParams stInterData;
};