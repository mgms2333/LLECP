#include"MC_ClearFault.h"
MC_ClearFault::MC_ClearFault(/* args */)
{
    m_bExecute =false;
    m_bBusy = false;
    m_bDone = false;
    m_bError = false;
    m_nErrorID = false;
}

MC_ClearFault::~MC_ClearFault()
{
}

void MC_ClearFault::operator()(CIA402Axis* axis)
{
    this->Execute();
}
void MC_ClearFault::operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID)
{
    if((bExecute && m_bExecute)&&(axis != m_pCIA402Axis))
    {
        m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_errorstop);
        m_fsClearFault = ReadyReset;
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

void MC_ClearFault::Execute()
{
    if(nullptr == m_pCIA402Axis)
    {
        m_bError = true;
        m_nErrorID = SMEC_INVALID_AXIS;
        return;
    }
    uint16_t nControlWord = 0;
    uint16_t nStatusWord = 0;
    m_pCIA402Axis->Axis_PDO_ReadStatusWord(nStatusWord);
    m_pCIA402Axis->Axis_PDO_ReadControlword(nControlWord);
    m_bBusy = false;
    m_bError = false;
    m_bDone = false;
    m_nErrorID = SMEC_SUCCESSED;
    if(!m_bExecute)
    {
       m_Timer.R_TRIG(m_bExecute);
       m_fsClearFault = ReadyReset;
       return;
    }
    switch (m_fsClearFault)
    {
    case ReadyReset:
        if(m_Timer.R_TRIG(m_bExecute))
        {
            if(m_pCIA402Axis->bVirtual)
            {
                m_bDone = true;
                m_fsClearFault = ResetFinish;
                m_Timer.Ton(false, SMC_TIME_OUT);
                break;
            }
            m_fsClearFault = StartReset;
            m_bBusy = true;
            m_Timer.Ton(false, SMC_TIME_OUT);
            break;
        }
        break;
    case StartReset:
        if(!m_pCIA402Axis->Axis_CheckError())
        {
            m_pCIA402Axis->Axis_ResetError();
            m_bDone = true;
            m_fsClearFault = ResetFinish;
            m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_power_off);
            nControlWord = en_ControlWord_Init;
            m_pCIA402Axis->Axis_PDO_SetControlword(nControlWord);
            m_Timer.Ton(false, SMC_TIME_OUT);
            break;
        }
        m_bBusy = true;
        nControlWord = nControlWord | en_ControlWord_fr | en_ControlWord_qs | en_ControlWord_ev;
        m_pCIA402Axis->Axis_PDO_SetControlword(nControlWord);
        m_fsClearFault = Reseting;
        break;
    case Reseting:
        if(m_Timer.Ton(true, SMC_TIME_OUT))
        {
            m_bError = true;
            m_nErrorID = SMEC_TIMEOUT;
            m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_errorstop);
            m_fsClearFault = ResetError;
            break;
        }
        if(!m_pCIA402Axis->Axis_CheckError())
        {
             m_pCIA402Axis->Axis_ResetError();
            m_bDone = true;
            m_fsClearFault = ResetFinish;
            m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_power_off);
            nControlWord = en_ControlWord_Init;
            m_pCIA402Axis->Axis_PDO_SetControlword(nControlWord);
            m_Timer.Ton(false, SMC_TIME_OUT);
            break;
        }
        m_bBusy = true;
        break;
    case ResetFinish:
        m_bDone = true;
        if(m_Timer.F_TRIG(m_bExecute))
        {
            m_fsClearFault = ReadyReset;
            m_Timer.Ton(false, SMC_TIME_OUT);
            break;
        }
        break;
    case ResetError:
        m_bError = true;
        m_nErrorID = SMEC_TIMEOUT;
        if(m_Timer.F_TRIG(m_bExecute))
        {
            m_fsClearFault = ReadyReset;
            m_Timer.Ton(false, SMC_TIME_OUT);
            break;
        }
        break;
    default:
        m_fsClearFault = ResetError;
        m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_errorstop);
        break;
    }
}
