#include"MC_InitResetAxis.h"
MC_InitResetAxis::MC_InitResetAxis(/* args */)
{
    m_Enabel =false;
    m_bBusy = false;
    m_bDone = false;
    m_bError = false;
    m_nErrorID = false;
}

MC_InitResetAxis::~MC_InitResetAxis()
{
}

void MC_InitResetAxis::operator()(CIA402Axis* axis,bool Enabel,bool &m_bBusy,bool &bDone, bool &bError,int &nErrorID)
{
    if((Enabel && m_Enabel)&&(axis != m_pCIA402Axis))
    {
        m_pCIA402Axis->AxisSetAxisState(EN_AxisMotionState::motionState_errorstop);
        m_FSInitResetAxis = ReadyInitResetAxis;
        m_bBusy = false;
        m_bError = true;
        m_bDone = false;
        m_nErrorID = SMEC_ABNORMAL_AXIS_CHANGE;
    }
    m_pCIA402Axis = axis;
    m_Enabel = Enabel;
    this->Execute();
    m_bBusy = m_bBusy;
    bDone = m_bDone;
    bError = m_bError;
    nErrorID = m_nErrorID;
}

void MC_InitResetAxis::operator()(CIA402Axis* axis)
{
    m_pCIA402Axis = axis;
    this->Execute();
}

void MC_InitResetAxis::Execute()
{
    m_bBusy = false;
    m_bError = false;
    m_bDone = false;
    m_nErrorID = SMEC_SUCCESSED;
    uint16_t nControlword;
    if(!m_Enabel)
    {
        m_Timer.R_TRIG(m_Enabel);
        m_FSInitResetAxis = ReadyInitResetAxis;
        return;
    }
    switch (m_FSInitResetAxis)
    {
    case ReadyInitResetAxis:
        if(m_Timer.R_TRIG(m_Enabel))
        {
            m_pCIA402Axis->AxisSetAxisState(EN_AxisMotionState::motionState_power_off);
            if(m_pCIA402Axis->bVirtual)
            {
                m_bDone = true;
                m_FSInitResetAxis = InitResetFinish;
                m_TimerTimeout.Ton(false, SMC_TIME_OUT);
                break;
            }
            m_FSInitResetAxis = InitResetting;
            m_bBusy = true;
            m_TimerTimeout.Ton(false, SMC_TIME_OUT);
            break;
        }
        break;
    case InitResetting:
        m_bBusy = true;
        m_pCIA402Axis->Axis_PDO_SetControlword(en_ControlWord_fr);
        if(m_TimerTimeout.Ton(true, SMC_TIME_OUT))
        {
            m_pCIA402Axis->Axis_PDO_SetControlword(en_ControlWord_Init);
            m_pCIA402Axis->AxisSetAxisState(EN_AxisMotionState::motionState_power_off);
            m_pCIA402Axis->Axis_PDO_ReadControlword(nControlword);
            if(nControlword == en_ControlWord_Init)
            {
                m_FSInitResetAxis = InitResetFinish;
            }
        }
        break;
    case InitResetFinish:
        if(m_Timer.F_TRIG(m_Enabel))
        {
            m_FSInitResetAxis = ReadyInitResetAxis;
            m_bBusy = false;
            m_bError = false;
            m_bDone = false;
            m_nErrorID = SMEC_SUCCESSED;
            m_TimerTimeout.Ton(false, SMC_TIME_OUT);
            break;
        }
        m_bDone = true;
        break;
    default:
        break;
    }
}