#include"MC_Reset.h"
MC_Reset::MC_Reset(/* args */)
{
    m_bExecute =false;
    m_bBusy = false;
    m_bDone = false;
    m_bError = false;
    m_nErrorID = false;
}

MC_Reset::~MC_Reset()
{
}

void MC_Reset::operator()(CIA402Axis* axis)
{
    this->Execute();
}
void MC_Reset::operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID)
{
    if((bExecute && m_bExecute)&&(axis != m_pCIA402Axis))
    {
        m_pCIA402Axis->AxisSetAxisState(EN_AxisMotionState::motionState_errorstop);
        m_fsReset = ReadyReset;
        m_bBusy = false;
        m_bError = true;
        m_bDone = false;
        m_nErrorID = SMEC_ABNORMAL_AXIS_CHANGE;
    }
    m_pCIA402Axis = axis;
    m_bExecute = bExecute;
    this->Execute();
    m_bBusy = m_bBusy;
    bDone = m_bDone;
    bError = m_bError;
    nErrorID = m_nErrorID;
}

void MC_Reset::Execute()
{
    return;
}
