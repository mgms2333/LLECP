#include"CIA402Axis.h"

// ===== 写入（RxPDO） =====
int CIA402Axis::Axis_PDO_SetControlword(uint16_t Controlword)
{
    Axis_PDO_CONTROLWORD = Controlword;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_SetTargetPosition(int32_t TargetPosition)
{
    Axis_PDO_TARGETPOSITION = TargetPosition;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_SetTargetVelocity(int32_t TargetVelocity)
{
    if(NULL == m_st_map.pTargetVelocity)
    {
        return -1;
    }
    Axis_PDO_TARGETVELOCITY = TargetVelocity;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_SetTargetTorque(int16_t TargetTorque)
{
    if(NULL == m_st_map.pTargetTorque)
    {
        return -1;
    }
    Axis_PDO_TARGETTORQUE = TargetTorque;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_SetModesOfOperation(uint8_t mode)
{
    Axis_PDO_TARGETMODESOFOPERATION = mode;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_SetDigitalOutputs(uint32_t DigitalOutputs)
{
    Axis_PDO_DIGITALOUTPUTS = DigitalOutputs;
    return AEC_SUCCESSED;
}

int CIA402Axis::Axis_PDO_ReadControlword(uint16_t& Controlword)
{
    Controlword = Axis_PDO_CONTROLWORD;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadTargetPosition(int32_t& TargetPosition)
{
    TargetPosition = Axis_PDO_TARGETPOSITION;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadTargetVelocity(int32_t& TargetVelocity)
{
    TargetVelocity = Axis_PDO_TARGETVELOCITY;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadTargetTorque(int16_t& TargetTorque)
{
    TargetTorque = Axis_PDO_TARGETTORQUE;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadTargetModesOfOperation(uint8_t& mode)
{
    mode = Axis_PDO_TARGETMODESOFOPERATION;
    return AEC_SUCCESSED;
}

// ===== 读取（TxPDO） =====
int CIA402Axis::Axis_PDO_ReadStatusWord(uint16_t& StatusWord)
{
    StatusWord = Axis_PDO_STATUSWORD;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadErrorCode(uint16_t& ErrorCode)
{
    if(NULL == m_st_map.pErrorCode)
    {
        return -1;
    }
    ErrorCode = Axis_PDO_ERRORCODE;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadActualPosition(int32_t& ActualPosition)
{
    ActualPosition = Axis_PDO_ACTUALPOSITION;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadActualVelocity(int32_t& ActualVelocity)
{
    if(NULL == m_st_map.pActualVelocity)
    {
        return -1;
    }
    ActualVelocity = Axis_PDO_ACTVELOCITY;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadActualTorque(int16_t& ActualTorque)
{
    if(NULL == m_st_map.pActualTorque)
    {
        return -1;
    }
    ActualTorque = Axis_PDO_ACTUALTORQUE;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadModesOfOperation(uint8_t& mode)
{
    mode = Axis_PDO_ACTUALMODESOFOPERATION;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadDigitalInputs(uint32_t& inputs)
{
    inputs = Axis_PDO_DIGITALINPUTS;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadDigitalOutputs(uint32_t& outputs)
{
    outputs = Axis_PDO_DIGITALOUTPUTS;
    return AEC_SUCCESSED;
}





int CIA402Axis::SoftMotion_PDO_SetControlword(uint16_t Controlword)
{
    stSoftMotion.stSoftMotionPDO.Controlword  = Controlword;
}
int CIA402Axis::SoftMotion_PDO_SetTargetPosition(int32_t TargetPosition)
{
    stSoftMotion.stSoftMotionPDO.TargetPosition  = TargetPosition;
}
int CIA402Axis::SoftMotion_PDO_SetTargetVelocity(int32_t TargetVelocity)
{
    stSoftMotion.stSoftMotionPDO.TargetVelocity = TargetVelocity;
}
int CIA402Axis::SoftMotion_PDO_SetTargetTorque(int16_t TargetTorque)
{
    stSoftMotion.stSoftMotionPDO.TargetTorque = TargetTorque;
}
int CIA402Axis::SoftMotion_PDO_SetModesOfOperation(EN_ModesOfOperation mode)
{
    stSoftMotion.stSoftMotionPDO.TargetModesOfOperation = mode;
}
int CIA402Axis::SoftMotion_PDO_SetDigitalOutputs(uint32_t DigitalOutputs)
{
    stSoftMotion.stSoftMotionPDO.DigitalInputs = DigitalOutputs;
}
int CIA402Axis::SoftMotion_PDO_ReadStatusWord(uint16_t& StatusWord)
{
    StatusWord = stSoftMotion.stSoftMotionPDO.StatusWord;
}
int CIA402Axis::SoftMotion_PDO_ReadErrorCode(uint16_t& ErrorCode)
{
    ErrorCode = stSoftMotion.stSoftMotionPDO.ErrorCode;
}
int CIA402Axis::SoftMotion_PDO_ReadActualPosition(int32_t& ActualPosition)
{
    ActualPosition = stSoftMotion.stSoftMotionPDO.ActualPosition;
}
int CIA402Axis::SoftMotion_PDO_ReadActualVelocity(int32_t& ActualVelocity)
{
    ActualVelocity = stSoftMotion.stSoftMotionPDO.ActualVelocity;
}
int CIA402Axis::SoftMotion_PDO_ReadActualTorque(int16_t& ActualTorque)
{
    ActualTorque = stSoftMotion.stSoftMotionPDO.ActualTorque;
}
int CIA402Axis::SoftMotion_PDO_ReadModesOfOperation(EN_ModesOfOperation& mode)
{
    mode = static_cast<EN_ModesOfOperation>(stSoftMotion.stSoftMotionPDO.ActualModesOfOperation);
}   
int CIA402Axis::SoftMotion_PDO_ReadDigitalInputs(uint32_t& inputs)
{
    inputs = stSoftMotion.stSoftMotionPDO.DigitalInputs;
}
int CIA402Axis::SoftMotion_PDO_ReadDigitalOutputs(uint32_t& outputs)
{
    outputs = stSoftMotion.stSoftMotionPDO.DigitalOutputs;
}
int CIA402Axis::SoftMotion_PDO_ReadControlword(uint16_t& Controlword)
{
    Controlword = stSoftMotion.stSoftMotionPDO.Controlword;
}
int CIA402Axis::SoftMotion_PDO_ReadTargetPosition(int32_t& TargetPosition)
{
    TargetPosition = stSoftMotion.stSoftMotionPDO.TargetPosition;
}
int CIA402Axis::SoftMotion_PDO_ReadTargetVelocity(int32_t& TargetVelocity)
{
    TargetVelocity = stSoftMotion.stSoftMotionPDO.ActualVelocity;
}
int CIA402Axis::SoftMotion_PDO_ReadTargetTorque(int16_t& TargetTorque)
{
    TargetTorque = stSoftMotion.stSoftMotionPDO.TargetTorque;
}
int CIA402Axis::SoftMotion_PDO_ReadTargetModesOfOperation(uint8_t& mode)
{
    mode = stSoftMotion.stSoftMotionPDO.TargetModesOfOperation;
}