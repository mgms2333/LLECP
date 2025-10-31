#include"CIA402Axis.h"


void CIA402Axis::PDOsynchronization()
{
    PDO_SetControlword(m_stMirrorPDO.Controlword);
    PDO_SetTargetPosition(m_stMirrorPDO.TargetPosition);
    PDO_SetTargetVelocity(m_stMirrorPDO.TargetVelocity);
    PDO_SetTargetTorque(m_stMirrorPDO.TargetTorque);
    PDO_SetModesOfOperation(m_stMirrorPDO.TargetModesOfOperation);

    PDO_ReadStatusWord(m_stMirrorPDO.StatusWord);
    PDO_ReadErrorCode(m_stMirrorPDO.ErrorCode);
    PDO_ReadActualPosition(m_stMirrorPDO.ActualPosition);
    PDO_ReadActualVelocity(m_stMirrorPDO.ActualVelocity);
    PDO_ReadActualTorque(m_stMirrorPDO.ActualTorque);
    PDO_ReadModesOfOperation(m_stMirrorPDO.ActualModesOfOperation);
}


// ===== 写入（RxPDO） =====
int CIA402Axis::PDO_SetControlword(uint16_t Controlword)
{
    PDO_CONTROLWORD = Controlword;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_SetTargetPosition(int32_t TargetPosition)
{
    PDO_TARGETPOSITION = TargetPosition;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_SetTargetVelocity(int32_t TargetVelocity)
{
    if(NULL == m_st_map.pTargetVelocity)
    {
        return -1;
    }
    PDO_TARGETVELOCITY = TargetVelocity;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_SetTargetTorque(int16_t TargetTorque)
{
    if(NULL == m_st_map.pTargetTorque)
    {
        return -1;
    }
    PDO_TARGETTORQUE = TargetTorque;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_SetModesOfOperation(uint8_t mode)
{
    PDO_TARGETMODESOFOPERATION = mode;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_SetDigitalOutputs(uint32_t DigitalOutputs)
{
    PDO_DIGITALOUTPUTS = DigitalOutputs;
    return AEC_SUCCESSED;
}

int CIA402Axis::PDO_ReadControlword(uint16_t& Controlword)
{
    Controlword = PDO_CONTROLWORD;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadTargetPosition(int32_t& TargetPosition)
{
    TargetPosition = PDO_TARGETPOSITION;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadTargetVelocity(int32_t& TargetVelocity)
{
    TargetVelocity = PDO_TARGETVELOCITY;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadTargetTorque(int16_t& TargetTorque)
{
    TargetTorque = PDO_TARGETTORQUE;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadTargetModesOfOperation(uint8_t& mode)
{
    mode = PDO_TARGETMODESOFOPERATION;
    return AEC_SUCCESSED;
}

// ===== 读取（TxPDO） =====
int CIA402Axis::PDO_ReadStatusWord(uint16_t& StatusWord)
{
    StatusWord = PDO_STATUSWORD;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadErrorCode(uint16_t& ErrorCode)
{
    if(NULL == m_st_map.pErrorCode)
    {
        return -1;
    }
    ErrorCode = PDO_ERRORCODE;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadActualPosition(int32_t& ActualPosition)
{
    ActualPosition = PDO_ACTUALPOSITION;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadActualVelocity(int32_t& ActualVelocity)
{
    if(NULL == m_st_map.pActualVelocity)
    {
        return -1;
    }
    ActualVelocity = PDO_ACTVELOCITY;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadActualTorque(int16_t& ActualTorque)
{
    if(NULL == m_st_map.pActualTorque)
    {
        return -1;
    }
    ActualTorque = PDO_ACTUALTORQUE;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadModesOfOperation(uint8_t& mode)
{
    mode = PDO_ACTUALMODESOFOPERATION;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadDigitalInputs(uint32_t& inputs)
{
    inputs = PDO_DIGITALINPUTS;
    return AEC_SUCCESSED;
}
int CIA402Axis::PDO_ReadDigitalOutputs(uint32_t& outputs)
{
    outputs = PDO_DIGITALOUTPUTS;
    return AEC_SUCCESSED;
}





int CIA402Axis::Axis_PDO_SetControlword(uint16_t Controlword)
{
    m_stMirrorPDO.Controlword  = Controlword;
}
int CIA402Axis::Axis_PDO_SetTargetPosition(int32_t TargetPosition)
{
    m_stMirrorPDO.TargetPosition  = TargetPosition;
}
int CIA402Axis::Axis_PDO_SetTargetVelocity(int32_t TargetVelocity)
{
    m_stMirrorPDO.TargetVelocity = TargetVelocity;
}
int CIA402Axis::Axis_PDO_SetTargetTorque(int16_t TargetTorque)
{
    m_stMirrorPDO.TargetTorque = TargetTorque;
}
int CIA402Axis::Axis_PDO_SetModesOfOperation(EN_ModesOfOperation mode)
{
    m_stMirrorPDO.TargetModesOfOperation = mode;
}
int CIA402Axis::Axis_PDO_SetDigitalOutputs(uint32_t DigitalOutputs)
{
    m_stMirrorPDO.DigitalInputs = DigitalOutputs;
}
int CIA402Axis::Axis_PDO_ReadStatusWord(uint16_t& StatusWord)
{
    StatusWord = m_stMirrorPDO.StatusWord;
}
int CIA402Axis::Axis_PDO_ReadErrorCode(uint16_t& ErrorCode)
{
    ErrorCode = m_stMirrorPDO.ErrorCode;
}
int CIA402Axis:: Axis_PDO_ReadActualPosition(int32_t& ActualPosition)
{
    ActualPosition = m_stMirrorPDO.ActualPosition;
}
int CIA402Axis::Axis_PDO_ReadActualVelocity(int32_t& ActualVelocity)
{
    ActualVelocity = m_stMirrorPDO.ActualVelocity;
}
int CIA402Axis::Axis_PDO_ReadActualTorque(int16_t& ActualTorque)
{
    ActualTorque = m_stMirrorPDO.ActualTorque;
}
int CIA402Axis::Axis_PDO_ReadModesOfOperation(EN_ModesOfOperation& mode)
{
    mode = static_cast<EN_ModesOfOperation>(m_stMirrorPDO.ActualModesOfOperation);
}   
int CIA402Axis::Axis_PDO_ReadDigitalInputs(uint32_t& inputs)
{
    inputs = m_stMirrorPDO.DigitalInputs;
}
int CIA402Axis::Axis_PDO_ReadDigitalOutputs(uint32_t& outputs)
{
    outputs = m_stMirrorPDO.DigitalOutputs;
}
int CIA402Axis::Axis_PDO_ReadControlword(uint16_t& Controlword)
{
    Controlword = m_stMirrorPDO.Controlword;
}
int CIA402Axis::Axis_PDO_ReadTargetPosition(int32_t& TargetPosition)
{
    TargetPosition = m_stMirrorPDO.TargetPosition;
}
int CIA402Axis::Axis_PDO_ReadTargetVelocity(int32_t& TargetVelocity)
{
    TargetVelocity = m_stMirrorPDO.ActualVelocity;
}
int CIA402Axis::Axis_PDO_ReadTargetTorque(int16_t& TargetTorque)
{
    TargetTorque = m_stMirrorPDO.TargetTorque;
}
int CIA402Axis::Axis_PDO_ReadTargetModesOfOperation(uint8_t& mode)
{
    mode = m_stMirrorPDO.TargetModesOfOperation;
}