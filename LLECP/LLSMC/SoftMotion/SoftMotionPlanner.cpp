#include"SoftMotion.h"
void SoftMotion::SoftMotionPlanner_RT()
{
    for(size_t i = 0;i<m_v_Axis.size();i++)
    {
        AxisMotionPlanner(m_v_Axis[i]);
    }
    return;
}

int SoftMotion::AxisMotionPlanner(CIA402Axis* pAxis)
{
    return AEC_SUCCESSED;
}