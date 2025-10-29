#include"CIA402Axis.h"
CIA402Axis::CIA402Axis(bool bVirtual) 
{ 
   m_bVirtual = bVirtual;
   m_enAxisMotionState = motionState_power_off;
   //map init
    m_st_map.pControlword             = &m_stPDO_Virtual.Controlword;
    m_st_map.pStatusWord              = &m_stPDO_Virtual.StatusWord;
    m_st_map.pErrorCode               = &m_stPDO_Virtual.ErrorCode;
    m_st_map.pTargetPosition          = &m_stPDO_Virtual.TargetPosition;
    m_st_map.pTargetVelocity          = &m_stPDO_Virtual.TargetVelocity;
    m_st_map.pTargetTorque            = &m_stPDO_Virtual.TargetTorque;
    m_st_map.pTargetModesOfOperation  = &m_stPDO_Virtual.TargetModesOfOperation;
    m_st_map.pActualPosition          = &m_stPDO_Virtual.ActualPosition;
    m_st_map.pActualVelocity          = &m_stPDO_Virtual.ActualVelocity;
    m_st_map.pActualTorque            = &m_stPDO_Virtual.ActualTorque;
    m_st_map.pActualModesOfOperation  = &m_stPDO_Virtual.ActualModesOfOperation;
    m_st_map.pDigitalInputs           = &m_stPDO_Virtual.DigitalInputs;
    m_st_map.pDigitalOutputs          = &m_stPDO_Virtual.DigitalOutputs;
}
CIA402Axis::~CIA402Axis(){}

int CIA402Axis::Axis_InitMap(ST_SMCInitMap st_map)
{
    m_st_map = st_map;
    int32_t s = *m_st_map.pActualPosition;
    return AEC_SUCCESSED;
}
int CIA402Axis::AxisSetAxisID(uint16_t id)
{
    m_nAxisID = id;
    return AEC_SUCCESSED;
}
int CIA402Axis::AxisSetAxisState(EN_AxisMotionState enAxisMotionState)
{
    m_enAxisMotionState = enAxisMotionState;
    return AEC_SUCCESSED;
}

EN_AxisMotionState CIA402Axis::AxisReadAxisState()
{
    return m_enAxisMotionState;
}

bool CIA402Axis::AxisCheckError(int& nErrorID)
{
    nErrorID = m_nErrorCode;
    return(motionState_errorstop == m_enAxisMotionState);
}
bool CIA402Axis::AxisCheckError()
{
    return(motionState_errorstop == m_enAxisMotionState);
}

int CIA402Axis::SoftMotion_PushMotionUint2SoftMotion(const ST_MotionUint STMotionUint)
{
    //立即中断当前运动，并开始新运动
    if(EN_BufferMode::enAborting == STMotionUint.BufferMode)
    {
        m_stSoftMotionEX.vMotionUint.clear();
    }
    //缓存点位
    m_stSoftMotionEX.vMotionUint.push_back(STMotionUint);
    return AEC_SUCCESSED;
}

int CIA402Axis::SoftMotion_GetMotionUintFromSoftMotion(ST_MotionUint& stMotionUint)
{
    int res = m_stSoftMotionEX.vMotionUint.size();
    if(res > 0 )
    {
        stMotionUint = m_stSoftMotionEX.vMotionUint[0];
    }
    return res;
}



void CIA402Axis::Axis_RT()
{
    DataSynchronization();
    return;
}

void CIA402Axis::DataSynchronization()
{
    //PDO
    nCmdControlWord = *m_st_map.pControlword;
    nActStatusWord = *m_st_map.pStatusWord;
    TargetPosition_PDO = *m_st_map.pTargetPosition;
    nActualPosition_PDO = *m_st_map.pActualPosition;
    nCmdModeOpration = *m_st_map.pTargetModesOfOperation;
    nActModeOpration = *m_st_map.pActualModesOfOperation;


    //运动数据
    dSetPosition = m_stAxisMotionData_now.dTarPos = m_stAxisConfiguration.nEncodeDirection * 
                        double(TargetPosition_PDO - m_stAxisConfiguration.nEncodeHomePos) / 
                        double(m_stAxisConfiguration.nEncodeRatio * m_stAxisConfiguration.dGearRatio);
    dActPosition = m_stAxisMotionData_now.dTarPos = m_stAxisConfiguration.nEncodeDirection * 
                                        double(*m_st_map.pActualPosition - m_stAxisConfiguration.nEncodeHomePos) / 
                                        double(m_stAxisConfiguration.nEncodeRatio * m_stAxisConfiguration.dGearRatio);
    nAxisErrorID = m_nErrorCode;
    //配置
    bVirtual = m_bVirtual;
    enAxisMotionState = m_enAxisMotionState;
    dGearRatio = m_stAxisConfiguration.dGearRatio;
    nEncodeRatio = m_stAxisConfiguration.nEncodeRatio;
    nEncodeDirection = m_stAxisConfiguration.nEncodeDirection;
    nEncodeHomePos = m_stAxisConfiguration.nEncodeHomePos;
    dCurrentScales = m_stAxisConfiguration.dCurrentScales;
    dTorqueFeedforwardScale = m_stAxisConfiguration.dTorqueFeedforwardScale;
    dVelocityScale = m_stAxisConfiguration.dVelocityScale;
    nCurrentDirection = m_stAxisConfiguration.nCurrentDirection;
    return;
}