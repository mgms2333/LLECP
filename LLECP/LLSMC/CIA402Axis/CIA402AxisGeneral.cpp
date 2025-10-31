#include"CIA402Axis.h"

int CIA402Axis::Axis_SetTargetPosition(double TargetPosition)
{
    printf("Cmdpos:%f\n",TargetPosition);
    int32_t count = m_stAxisConfiguration.nEncodeDirection *  (TargetPosition * m_stAxisConfiguration.nEncodeRatio * m_stAxisConfiguration.dGearRatio) + 
                                    m_stAxisConfiguration.nEncodeHomePos;
    Axis_PDO_SetTargetPosition(count);  
    return AEC_SUCCESSED;   
}