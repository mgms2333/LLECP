#include"MC_SetOverride.h"
MC_SetOverride::MC_SetOverride(/* args */)
{
    m_dVelFactor = 1;
    m_dAccFactor = 1;
    m_dJerkFactor = 1;
}

MC_SetOverride::~MC_SetOverride()
{
}

void MC_SetOverride::operator()(CIA402Axis* axis)
{
    this->Execute();
}
void MC_SetOverride::operator()(CIA402Axis* axis,bool bExecute,double dVelFactor,double dAccFactor,double dJerkFactor,bool &bDone, bool &bError,int &nErrorID)
{
    if((bExecute && m_bExecute)&&(axis != m_pCIA402Axis))
    {
        m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_errorstop);
        m_bBusy = false;
        m_bError = true;
        m_bDone = false;
        m_nErrorID = SMEC_ABNORMAL_AXIS_CHANGE;
    }
    m_dVelFactor = dVelFactor;
    m_dAccFactor = dAccFactor;
    m_dJerkFactor = dJerkFactor;
    m_pCIA402Axis = axis;
    m_bExecute = bExecute;
    this->Execute();
    m_bBusy = m_bBusy;
    bDone = m_bDone;
    bError = m_bError;
    nErrorID = m_nErrorID;
}


void MC_SetOverride::Execute()
{
    if(nullptr == m_pCIA402Axis)
    {
        m_bError = true;
        m_nErrorID = SMEC_INVALID_AXIS;
        return;
    }
    ST_Factor stFactor;
    stFactor.dVelFactor = m_dVelFactor;
    stFactor.dJerkFactor = m_dJerkFactor;
    stFactor.dAccFactor = m_dAccFactor;
    int res = m_pCIA402Axis->Axis_SetOverride(stFactor);
    if(SMEC_SUCCESSED == res)
    {
        m_bDone = true;
    }
    else
    {
        m_bError = true;
        m_nErrorID = res;
    }
}