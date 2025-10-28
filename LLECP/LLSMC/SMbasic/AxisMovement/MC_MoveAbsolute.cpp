#include"MC_MoveAbsolute.h"
MC_MoveAbsolute::MC_MoveAbsolute(/* args */)
{
}

MC_MoveAbsolute::~MC_MoveAbsolute()
{
}

void MC_MoveAbsolute::operator()(CIA402Axis* axis)
{
    this->Execute();
}
void MC_MoveAbsolute::operator()(CIA402Axis* axis,bool bExecute,double dPosition,double dVelocity,double dAcceleration,double dDeceleration,double dJerk,EN_Direction enDirection,EN_BufferMode enBufferMode,
                    bool& bDone,bool& bBusy,bool& bCommandAborted,bool& bError,int& ErrorID)
{


    m_pCIA402Axis                                   = axis;
    m_bExecute                                      = bExecute;
    m_MotionUint.PlanningMotionParam.pos            = dPosition;
    m_MotionUint.PlanningMotionParam.vel            = dVelocity;
    m_MotionUint.PlanningMotionParam.acc            = dAcceleration;
    m_MotionUint.PlanningMotionParam.dec            = dDeceleration;
    m_MotionUint.PlanningMotionParam.jerk           = dJerk;
    m_MotionUint.PlanningMotionParam.Direction      = enDirection;
    m_MotionUint.BufferMode                         = enBufferMode;
    m_MotionUint.fbID                               = this;
    this->Execute();
    bDone = m_bDone;
    bBusy = m_bBusy;
    bCommandAborted = m_bCommandAborted;
    bError = m_bError;
    ErrorID = m_nErrorID;
}

void MC_MoveAbsolute::Execute()
{
    // 输出参数默认初始化
    m_bDone             = false;
    m_bBusy             = false;
    m_bCommandAborted   = false;
    m_bError            = false;
    m_nErrorID           = 0;
    
    //状态校验
    if(EN_AxisMotionState::motionState_errorstop != m_pCIA402Axis->AxisReadAxisState())
    {
        m_bError            = true;
        m_pCIA402Axis->AxisCheckError(m_nErrorID);
        return;
    }
    if(m_bExecute)
    {
        if((EN_AxisMotionState::motionState_standstill != m_pCIA402Axis->AxisReadAxisState()) &&
             (EN_AxisMotionState::motionState_continuous_motion!= m_pCIA402Axis->AxisReadAxisState()) &&
                (EN_AxisMotionState::motionState_discrete_motion!= m_pCIA402Axis->AxisReadAxisState()))
        {
            m_nErrorID = SMEC_AXIS_STATUS_INTERCEPTION;
            m_bError = true;
        }
    }
    else
    {
        // 输出参数默认初始化
        m_bDone             = false;
        m_bBusy             = false;
        m_bCommandAborted   = false;
        m_bError            = false;
        m_nErrorID           = 0;
        return;
    }
    //上升沿push
    if(m_Timer.R_TRIG(m_bExecute))
    {
        m_pCIA402Axis->SoftMotion_PushMotionUint2SoftMotion(m_MotionUint);
    }
    ST_MotionUint FirstMotionUint;
    int nMotionNum = m_pCIA402Axis->SoftMotion_GetMotionUintFromSoftMotion(FirstMotionUint);
    //点位小于0直接return
    if(nMotionNum < 0)
    {
        m_bCommandAborted = true;
        return;
    }
    //第一个点就是这个点
    if(this == FirstMotionUint.fbID)
    {
        //运动完成监测
        if(FirstMotionUint.bMotionDone)
        {
            m_bDone             = true;
        }
        else
        {
            m_bBusy             = true;
        }
    }
    //move被插其他块
    else
    {
        m_bCommandAborted = true;
        return;
    }
    return;

    // 输入参数保存到成员变量
}