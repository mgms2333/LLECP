#include"SoftMotion.h"

// ===== 写入（RxPDO） =====
int SoftMotion::SoftMotion_PDO_SetControlword(CIA402Axis* Axis_REF,uint16_t Controlword)
{
    Axis_REF->stSoftMotion.stSoftMotionPDO.Controlword = Controlword;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_SetTargetPosition(CIA402Axis* Axis_REF,int32_t TargetPosition)
{
    Axis_REF->stSoftMotion.stSoftMotionPDO.TargetPosition = TargetPosition;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_SetTargetVelocity(CIA402Axis* Axis_REF,int32_t TargetVelocity)
{
    Axis_REF->stSoftMotion.stSoftMotionPDO.TargetVelocity = TargetVelocity;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_SetTargetTorque(CIA402Axis* Axis_REF,int16_t TargetTorque)
{
    Axis_REF->stSoftMotion.stSoftMotionPDO.TargetTorque = TargetTorque;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_SetModesOfOperation(CIA402Axis* Axis_REF,EN_ModesOfOperation mode)
{
    Axis_REF->stSoftMotion.stSoftMotionPDO.TargetModesOfOperation = mode;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_SetDigitalOutputs(CIA402Axis* Axis_REF,uint32_t DigitalOutputs)
{
    Axis_REF->stSoftMotion.stSoftMotionPDO.DigitalOutputs = DigitalOutputs;
    return AEC_SUCCESSED;
}

int SoftMotion::SoftMotion_PDO_ReadControlword(CIA402Axis* Axis_REF,uint16_t& Controlword)
{
    Controlword = Axis_REF->stSoftMotion.stSoftMotionPDO.Controlword;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadTargetPosition(CIA402Axis* Axis_REF,int32_t& TargetPosition)
{
    TargetPosition = Axis_REF->stSoftMotion.stSoftMotionPDO.TargetPosition;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadTargetVelocity(CIA402Axis* Axis_REF,int32_t& TargetVelocity)
{
    TargetVelocity = Axis_REF->stSoftMotion.stSoftMotionPDO.TargetVelocity;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadTargetTorque(CIA402Axis* Axis_REF,int16_t& TargetTorque)
{
    TargetTorque = Axis_REF->stSoftMotion.stSoftMotionPDO.TargetTorque;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadTargetModesOfOperation(CIA402Axis* Axis_REF,uint8_t& mode)
{
    mode = Axis_REF->stSoftMotion.stSoftMotionPDO.TargetModesOfOperation;
    return AEC_SUCCESSED;
}

// ===== 读取（TxPDO） =====
int SoftMotion::SoftMotion_PDO_ReadStatusWord(CIA402Axis* Axis_REF,uint16_t& StatusWord)
{
    StatusWord = Axis_REF->stSoftMotion.stSoftMotionPDO.StatusWord;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadErrorCode(CIA402Axis* Axis_REF,uint16_t& ErrorCode)
{
    ErrorCode = Axis_REF->stSoftMotion.stSoftMotionPDO.ErrorCode;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadActualPosition(CIA402Axis* Axis_REF,int32_t& ActualPosition)
{
    ActualPosition = Axis_REF->stSoftMotion.stSoftMotionPDO.ActualPosition;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadActualVelocity(CIA402Axis* Axis_REF,int32_t& ActualVelocity)
{
    ActualVelocity = Axis_REF->stSoftMotion.stSoftMotionPDO.ActualVelocity;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadActualTorque(CIA402Axis* Axis_REF,int16_t& ActualTorque)
{
    ActualTorque = Axis_REF->stSoftMotion.stSoftMotionPDO.ActualTorque;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadModesOfOperation(CIA402Axis* Axis_REF,EN_ModesOfOperation& mode)
{
    mode = static_cast<EN_ModesOfOperation>(Axis_REF->stSoftMotion.stSoftMotionPDO.ActualModesOfOperation);
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadDigitalInputs(CIA402Axis* Axis_REF,uint32_t& inputs)
{
    inputs = Axis_REF->stSoftMotion.stSoftMotionPDO.DigitalInputs;
    return AEC_SUCCESSED;
}
int SoftMotion::SoftMotion_PDO_ReadDigitalOutputs(CIA402Axis* Axis_REF,uint32_t& outputs)
{
    outputs = Axis_REF->stSoftMotion.stSoftMotionPDO.DigitalOutputs;
    return AEC_SUCCESSED;
}