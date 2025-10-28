#include"SoftMotion.h"
SoftMotion::SoftMotion(std::vector<CIA402Axis*> v_Axis)
{
    m_v_Axis = v_Axis;
}

SoftMotion::~SoftMotion()
{
}

void SoftMotion::SoftMotionRun()
{
    SoftMotionPlanner_RT();
    PDOsynchronization();
    AxisRealTime();
    return;
}

void SoftMotion::PDOsynchronization()
{
    for(size_t i = 0;i<m_v_Axis.size();i++)
    {
        m_v_Axis[i]->Axis_PDO_SetControlword(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.Controlword);
        m_v_Axis[i]->Axis_PDO_SetTargetPosition(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.TargetPosition);
        m_v_Axis[i]->Axis_PDO_SetTargetVelocity(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.TargetVelocity);
        m_v_Axis[i]->Axis_PDO_SetTargetTorque(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.TargetTorque);
        m_v_Axis[i]->Axis_PDO_SetModesOfOperation(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.TargetModesOfOperation);

        m_v_Axis[i]->Axis_PDO_ReadStatusWord(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.StatusWord);
        m_v_Axis[i]->Axis_PDO_ReadErrorCode(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.ErrorCode);
        m_v_Axis[i]->Axis_PDO_ReadActualPosition(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.ActualPosition);
        m_v_Axis[i]->Axis_PDO_ReadActualVelocity(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.ActualVelocity);
        m_v_Axis[i]->Axis_PDO_ReadActualTorque(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.ActualTorque);
        m_v_Axis[i]->Axis_PDO_ReadModesOfOperation(m_v_Axis[i]->m_stSoftMotion.stSoftMotionPDO.ActualModesOfOperation);
    }
    return;
}

void SoftMotion::AxisRealTime()
{
    for(size_t i = 0;i<m_v_Axis.size();i++)
    {
        m_v_Axis[i]->Axis_RT();
    }
}