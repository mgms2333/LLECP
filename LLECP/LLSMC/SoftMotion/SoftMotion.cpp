#include"SoftMotion.h"
SoftMotion::SoftMotion(std::vector<CIA402Axis*> v_Axis)
{
    m_v_Axis = v_Axis;
    for(size_t i =0;i<v_Axis.size();i++)
    {
        SoftMotionPlanParams data;
        v_Axis[i]->Axis_SetAxisID(i);
        m_vSoftMotionPlanParams.push_back(data);
    }
}

SoftMotion::~SoftMotion()
{
}

int SoftMotion::SetSoftMotionCycle(double dCycle)
{
    m_dSoftMotionCycle = dCycle;
    for(size_t i = 0;i<m_v_Axis.size();i++)
    {
        m_v_Axis[i]->Axis_SetControlCycle(dCycle);
    }
    return AEC_SUCCESSED;
}

void SoftMotion::SoftMotionRun()
{
    SoftMotionPlanner_RT();
    AxisRealTime();
    return;
}

void SoftMotion::AxisRealTime()
{
    for(size_t i = 0;i<m_v_Axis.size();i++)
    {
        m_v_Axis[i]->Axis_RT();
    }
}