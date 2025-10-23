#include"CIA402Axis.h"
CIA402Axis::CIA402Axis(bool bVirtual) 
{ 
   m_bVirtual = bVirtual;
   m_enAxisMotionState = motionState_power_off;
   //map init
    m_st_map.pControlword             = &m_stPDO_Virtual.pControlword;
    m_st_map.pStatusWord              = &m_stPDO_Virtual.pStatusWord;
    m_st_map.pErrorCode               = &m_stPDO_Virtual.pErrorCode;
    m_st_map.pTargetPosition          = &m_stPDO_Virtual.pTargetPosition;
    m_st_map.pTargetVelocity          = &m_stPDO_Virtual.pTargetVelocity;
    m_st_map.pTargetTorque            = &m_stPDO_Virtual.pTargetTorque;
    m_st_map.pTargetModesOfOperation  = &m_stPDO_Virtual.pTargetModesOfOperation;
    m_st_map.pActualPosition          = &m_stPDO_Virtual.pActualPosition;
    m_st_map.pActualVelocity          = &m_stPDO_Virtual.pActualVelocity;
    m_st_map.pActualTorque            = &m_stPDO_Virtual.pActualTorque;
    m_st_map.pActualModesOfOperation  = &m_stPDO_Virtual.pActualModesOfOperation;
    m_st_map.pDigitalInputs           = &m_stPDO_Virtual.pDigitalInputs;
    m_st_map.pDigitalOutputs          = &m_stPDO_Virtual.pDigitalOutputs;
}
CIA402Axis::~CIA402Axis(){}

int CIA402Axis::Axis_InitMap(ST_SMCInitMap st_map)
{
    m_st_map = st_map;
    int32_t s = *m_st_map.pActualPosition;
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