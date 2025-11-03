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
    //状态校验
    if((pAxis->AxisReadAxisState()!=EN_AxisMotionState::motionState_standstill)
            &&(pAxis->AxisReadAxisState()!=EN_AxisMotionState::motionState_discrete_motion)
            &&(pAxis->AxisReadAxisState()!=EN_AxisMotionState::motionState_continuous_motion)
            &&(pAxis->AxisReadAxisState()!=EN_AxisMotionState::motionState_synchronized_motion))
            {
                pAxis->m_stSoftMotionEX.vMotionUint.clear();
                return AEC_SUCCESSED;
            }
    if(pAxis->m_stSoftMotionEX.vMotionUint.size() == 0)
    {
        return AEC_SUCCESSED;
    }
    auto itMotionData = pAxis->m_stSoftMotionEX.vMotionUint.begin();
    while(itMotionData->bMotionDone)
    {
        itMotionData ++;
        //所有点位运动完成，清除所有点位
        if(pAxis->m_stSoftMotionEX.vMotionUint.end() == itMotionData)
        {
            pAxis->m_stSoftMotionEX.vMotionUint.clear();
            //切换状态
            pAxis->AxisSetAxisState(EN_AxisMotionState::motionState_standstill);
            return AEC_SUCCESSED;
        }
    }
    pAxis->AxisSetAxisState(EN_AxisMotionState::motionState_discrete_motion);
    //当前迭代器内是正在运动的点位
    ST_PlanningMotionParam stMotionParam = itMotionData->PlanningMotionParam;
    //正在运行当前点位
    if(pAxis->m_stSoftMotionEX.stSoftMotionMotionParam == stMotionParam)
    {
        //得到当前帧的运动参数
        FifteenSeg_Inter(m_vSoftMotionPlanParams[pAxis->nAxisID].stActParam,m_vSoftMotionPlanParams[pAxis->nAxisID].trackData,
                        pAxis->m_stSoftMotionEX.dMotionTime,m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData);
        //开始运动 
        pAxis->Axis_SetTargetPosition(m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData.P);
        //时间帧加
        pAxis->m_stSoftMotionEX.dMotionTime += m_dSoftMotionCycle;
    }
    else//新点位
    {
        pAxis->m_stSoftMotionEX.stSoftMotionMotionParam = stMotionParam;
        pAxis->m_stSoftMotionEX.dMotionTime = 0;
        ST_PlanParams stsetParam{pAxis->dActPosition,stMotionParam.pos,pAxis->dActVelocity,0,stMotionParam.vel,stMotionParam.acc,stMotionParam.dec,stMotionParam.jerk,MaxSnap,enPosition_PlanMode};
        FifteenSeg_plan(stsetParam,m_vSoftMotionPlanParams[pAxis->nAxisID].stActParam,m_vSoftMotionPlanParams[pAxis->nAxisID].trackData);
        //得到当前帧的运动参数
        FifteenSeg_Inter(m_vSoftMotionPlanParams[pAxis->nAxisID].stActParam,m_vSoftMotionPlanParams[pAxis->nAxisID].trackData,
                        pAxis->m_stSoftMotionEX.dMotionTime,m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData);
        //开始运动 
        pAxis->Axis_SetTargetPosition(m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData.P);
        //时间帧加
        pAxis->m_stSoftMotionEX.dMotionTime += m_dSoftMotionCycle;
    }
    //运动完成检测
    if(pAxis->m_stSoftMotionEX.dMotionTime > m_vSoftMotionPlanParams[pAxis->nAxisID].trackData.T)
    {
        itMotionData->bMotionDone = true;
    }
    return AEC_SUCCESSED;
}