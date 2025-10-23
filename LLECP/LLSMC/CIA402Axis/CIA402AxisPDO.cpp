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
    Axis_PDO_TARGETVELOCITY = TargetVelocity;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_SetTargetTorque(int16_t TargetTorque)
{
    Axis_PDO_TARGETTORQUE = TargetTorque;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_SetModesOfOperation(EN_ModesOfOperation mode)
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
    ActualVelocity = Axis_PDO_ACTVELOCITY;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadActualTorque(int16_t& ActualTorque)
{
    ActualTorque = Axis_PDO_ACTUALTORQUE;
    return AEC_SUCCESSED;
}
int CIA402Axis::Axis_PDO_ReadModesOfOperation(EN_ModesOfOperation& mode)
{
    mode = static_cast<EN_ModesOfOperation>(Axis_PDO_ACTUALMODESOFOPERATION);
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