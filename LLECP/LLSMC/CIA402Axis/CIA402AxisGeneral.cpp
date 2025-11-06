#include"CIA402Axis.h"

int CIA402Axis::Axis_SetTargetPosition(double TargetPosition)
{
    //printf("CmdPosition:%f\n",TargetPosition);
    int32_t count = m_stAxisConfiguration.nEncodeDirection *  (TargetPosition * m_stAxisConfiguration.nEncodeRatio * m_stAxisConfiguration.dGearRatio) + 
                                    m_stAxisConfiguration.nEncodeHomePos;
    Axis_PDO_SetTargetPosition(count);  
    return AEC_SUCCESSED;   
}

int CIA402Axis::Axis_SetTargetTorque(double TargetTorque)
{
    int32_t count = TargetTorque * m_stAxisConfiguration.nCurrentDirection * m_stAxisConfiguration.dCurrentScales;
    Axis_PDO_SetTargetTorque(count);  
    return AEC_SUCCESSED;  
}

int CIA402Axis::Axis_SetTargetVelocity(double TargetVelocity)
{
    int32_t count = TargetVelocity * m_stAxisConfiguration.nCurrentDirection * m_stAxisConfiguration.dVelocityScale;
    Axis_PDO_SetTargetVelocity(count);  
    return AEC_SUCCESSED;  
}
