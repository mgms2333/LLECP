#include"MC_PowerOff.h"
MC_PowerOff::MC_PowerOff(/* args */)
{
    m_bExecute =false;
    m_bBusy = false;
    m_bDone = false;
    m_bError = false;
    m_nErrorID = false;
}

MC_PowerOff::~MC_PowerOff()
{
}

void MC_PowerOff::operator()(CIA402Axis* axis)
{
    this->Execute();
}
void MC_PowerOff::operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID)
{
    if((bExecute && m_bExecute)&&(axis != m_pCIA402Axis))
    {
        m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_errorstop);
        m_fsPowerOff = ReadyPowerOff;
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

void MC_PowerOff::Execute()
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
       m_fsPowerOff = ReadyPowerOff;
       return;
    }
    //检测使能中报错
    uint16_t unStatuBit;
    if(m_pCIA402Axis->Axis_CheckError())
    {
        m_fsPowerOff = PowerOffError;
        //反馈错误状态
        m_bError = true;
        m_Timer.Ton(false, SMC_TIME_OUT);
        m_pCIA402Axis->Axis_CheckError();
    } 
    //位置对齐
    if(ReadyPowerOff != m_fsPowerOff)
    {
        int32_t nActPos;
        m_pCIA402Axis->Axis_PDO_ReadActualPosition(nActPos);
        m_pCIA402Axis->Axis_PDO_SetTargetPosition(nActPos);
    }
    switch (m_fsPowerOff)
    {
        case ReadyPowerOff:
            if(m_Timer.R_TRIG(m_bExecute))
            {
                //检测轴是否为去使能状态（错误或运动等状态禁用）
                if(motionState_standstill != m_pCIA402Axis->Axis_ReadAxisState())
                {
                    //此状态不允许使能，也不切状态机，功能块置为error
                    m_fsPowerOff = PowerOffError;
                    //切换到错误状态
                    m_bError = true;
                    m_Timer.Ton(false, SMC_TIME_OUT);
                    break;
                }
                //切换到位置模式
                m_pCIA402Axis->Axis_PDO_SetModesOfOperation(EN_ModesOfOperation ::enModeCyclicSyncPosition);
                //虚拟轴直接使能完成
                if(m_pCIA402Axis->bVirtual)
                {
                    //反馈完成状态
                    m_bDone = true;
                    //同步状态机
                    m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_power_off);
                    m_fsPowerOff = PowerOffFinish;
                    m_Timer.Ton(false, SMC_TIME_OUT);
                    break;
                }
                //初始化控制字
                m_pCIA402Axis->Axis_PDO_SetControlword(en_ControlWord_Init);
                m_fsPowerOff = StartPowerOff;
                m_bBusy = true;
                m_Timer.Ton(false, SMC_TIME_OUT);
                break;
            }
            break;
        case StartPowerOff:
            m_bBusy = true;
            nControlWord =  nControlWord & (~en_ControlWord_eo);
            m_pCIA402Axis->Axis_PDO_SetControlword(nControlWord);
            m_fsPowerOff = PowerOff_0x07;
            break;
        case PowerOff_0x07:
            m_bBusy = true;
            nControlWord =  nControlWord & (~en_ControlWord_so);
            m_fsPowerOff = PowerOff_0x06;
            break;
        case PowerOff_0x06:
            m_bBusy = true;
            if ((nStatusWord & en_StatusWord_oe) == 0)
            {
                m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_power_off);
                nControlWord = en_ControlWord_Init;
                m_pCIA402Axis->Axis_PDO_SetControlword(nControlWord);
                m_pCIA402Axis->Axis_PDO_SetTargetPosition(0); 
                m_bBusy = true;
                m_fsPowerOff = PowerOffFinish;
                break;
            }
            //超时检测
            if(m_Timer.Ton(true, SMC_TIME_OUT))
            {
                m_fsPowerOff = PowerOffError;
                m_pCIA402Axis->Axis_SetAxisState(EN_AxisMotionState::motionState_errorstop);
                m_bError = true;
                m_nErrorID = SMEC_TIMEOUT;
                m_Timer.Ton(false, SMC_TIME_OUT);
                break;
            }
            break;
        case PowerOffFinish:
            //检测到功能块去使能
            if(m_Timer.F_TRIG(m_bExecute))
            {
                m_nErrorID = SMEC_SUCCESSED;
                m_fsPowerOff = ReadyPowerOff;
                break;
            }
            m_bDone = true;
            break;
        case PowerOffError:
            //功能块复位
            if(m_Timer.F_TRIG(m_bExecute))
            {
                m_nErrorID = SMEC_SUCCESSED;
                m_fsPowerOff = ReadyPowerOff;
                break;
            }
            m_bError = true;
            m_nErrorID = SMEC_DISENABLE_ERROR;
            break;
        default:
            m_fsPowerOff = PowerOffError;
            m_nErrorID = SMEC_DISENABLE_ERROR;
            break;
    }
}