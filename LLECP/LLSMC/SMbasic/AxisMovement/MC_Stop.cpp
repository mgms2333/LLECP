#include"MC_Stop.h"
MC_Stop::MC_Stop(/* args */)
{
}

MC_Stop::~MC_Stop()
{
}

void MC_Stop::operator()(CIA402Axis* axis)
{
    this->Execute();
}
void MC_Stop::operator()(CIA402Axis* axis,bool bExecute,double dPosition,double dVelocity,double dAcceleration,double dDeceleration,double dJerk,EN_Direction enDirection,EN_BufferMode enBufferMode,
                    bool& bDone,bool& bBusy,bool& bCommandAborted,bool& bError,int& ErrorID)
{
    m_pCIA402Axis                                   = axis;
    m_bExecute                                      = bExecute;
    m_MotionUint_New.PlanningMotionParam.pos            = dPosition;
    m_MotionUint_New.PlanningMotionParam.vel            = dVelocity;
    m_MotionUint_New.PlanningMotionParam.acc            = dAcceleration;
    m_MotionUint_New.PlanningMotionParam.dec            = dDeceleration;
    m_MotionUint_New.PlanningMotionParam.jerk           = dJerk;
    m_MotionUint_New.PlanningMotionParam.Direction      = enDirection;
    m_MotionUint_New.PlanningMotionParam.PlanningMode   = enStop;
    m_MotionUint_New.BufferMode                         = enBufferMode;
    m_MotionUint_New.MoveType                           = enAbsoluteMotion;
    m_MotionUint_New.fbID                               = this;
    this->Execute();
    bDone = m_bDone;
    bBusy = m_bBusy;
    bCommandAborted = m_bCommandAborted;
    bError = m_bError;
    ErrorID = m_nErrorID;
}

void MC_Stop::Execute()
{

}