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
    if((pAxis->Axis_ReadAxisState()!=EN_AxisMotionState::motionState_standstill)
            &&(pAxis->Axis_ReadAxisState()!=EN_AxisMotionState::motionState_discrete_motion)
            &&(pAxis->Axis_ReadAxisState()!=EN_AxisMotionState::motionState_continuous_motion)
            &&(pAxis->Axis_ReadAxisState()!=EN_AxisMotionState::motionState_synchronized_motion))
            {
                pAxis->m_stSoftMotionEX.vMotionUint.clear();
                return AEC_SUCCESSED;
            }
    double T;
    if(pAxis->m_stSoftMotionEX.vMotionUint.size() == 0)
    {
        return AEC_SUCCESSED;
    }
    auto itMotionData = pAxis->m_stSoftMotionEX.vMotionUint.begin();
    while(itMotionData->bMotionDone)
    {
        itMotionData ++;
        //所有点位运动完成
        if(pAxis->m_stSoftMotionEX.vMotionUint.end() == itMotionData)
        {
            //切换状态
            pAxis->Axis_SetAxisState(EN_AxisMotionState::motionState_standstill);
            return AEC_SUCCESSED;
        }
    }
    pAxis->Axis_SetAxisState(EN_AxisMotionState::motionState_discrete_motion);
    //当前迭代器内是正在运动的点位
    ST_PlanningMotionParam stMotionParam = itMotionData->PlanningMotionParam;
    //正在运行当前点位
    if((pAxis->m_stSoftMotionEX.stSoftMotionMotionParam == stMotionParam)&&(!pAxis->m_stSoftMotionEX.bReplan))
    {
        //通用规划
        if((EN_PlanningMode::enPositionPlanningMode == stMotionParam.PlanningMode)||(EN_PlanningMode::enVelocityPlanningMode == stMotionParam.PlanningMode))
        {
        //得到当前帧的运动参数
        FifteenSeg_Inter(m_vSoftMotionPlanParams[pAxis->nAxisID].stActParam,m_vSoftMotionPlanParams[pAxis->nAxisID].trackData,
                        pAxis->m_stSoftMotionEX.dMotionTime,m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData);
        }
        //停止规划
        else if(EN_PlanningMode::enStopPlanningMode == stMotionParam.PlanningMode)
        {
            stopmotion(pAxis->dActPosition,pAxis->dActVelocity,pAxis->dActAcceleration,stMotionParam.dec,stMotionParam.jerk,pAxis->m_stSoftMotionEX.dMotionTime,T,m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData);
        }
        //开始运动 
        printf("CmdPosition:%f\n",m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData.P);
        pAxis->Axis_SetTargetPosition(m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData.P);
        //时间帧加
        pAxis->m_stSoftMotionEX.dMotionTime += m_dSoftMotionCycle;
    }
    else//新点位
    {
        pAxis->m_stSoftMotionEX.bReplan = false;
        ST_PlanningMotionParam stMotionParam_Run = stMotionParam;
        //通用规划
        if((EN_PlanningMode::enPositionPlanningMode == stMotionParam_Run.PlanningMode)||(EN_PlanningMode::enVelocityPlanningMode == stMotionParam_Run.PlanningMode))
        {
            //绝对运动做偏移
            if(enRelativMotion == itMotionData->MoveType)
            {
                stMotionParam_Run.pos = pAxis->dActPosition + stMotionParam_Run.pos;
            }
            pAxis->m_stSoftMotionEX.stSoftMotionMotionParam = stMotionParam;
            pAxis->m_stSoftMotionEX.dMotionTime = 0;
            ST_PlanParams stsetParam;
            stsetParam.q0 = pAxis->dActPosition;
            stsetParam.q1 = stMotionParam_Run.pos;
            stsetParam.v0 = pAxis->dActVelocity;
            stsetParam.v1 = 0;
            stsetParam.a0 = 0;
            stsetParam.a1 = 0;
            stsetParam.V_max = stMotionParam_Run.vel *  pAxis->m_stSoftMotionEX.m_stFactor.dVelFactor;
            stsetParam.A_amax = stMotionParam_Run.acc *  pAxis->m_stSoftMotionEX.m_stFactor.dAccFactor;
            stsetParam.A_dmax = stMotionParam_Run.dec *  pAxis->m_stSoftMotionEX.m_stFactor.dJerkFactor;
            stsetParam.J_max = stMotionParam_Run.jerk;
            stsetParam.S_max = MaxSnap;
            stsetParam.enPlanmode = stMotionParam_Run.PlanningMode;
            FifteenSeg_plan(stsetParam,m_vSoftMotionPlanParams[pAxis->nAxisID].stActParam,m_vSoftMotionPlanParams[pAxis->nAxisID].trackData);
            //得到当前帧的运动参数
            FifteenSeg_Inter(m_vSoftMotionPlanParams[pAxis->nAxisID].stActParam,m_vSoftMotionPlanParams[pAxis->nAxisID].trackData,
                            pAxis->m_stSoftMotionEX.dMotionTime,m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData);
        }
        //停止规划
        else if(EN_PlanningMode::enStopPlanningMode == stMotionParam_Run.PlanningMode)
        {
            pAxis->m_stSoftMotionEX.stSoftMotionMotionParam = stMotionParam;
            pAxis->m_stSoftMotionEX.dMotionTime = 0;
            stopmotion(pAxis->dActPosition,pAxis->dActVelocity,pAxis->dActAcceleration,stMotionParam.dec,stMotionParam.jerk,pAxis->m_stSoftMotionEX.dMotionTime,T,m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData);
        }
        //开始运动 
        printf("CmdPosition:%f\n",m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData.P);
        pAxis->Axis_SetTargetPosition(m_vSoftMotionPlanParams[pAxis->nAxisID].stInterData.P);
        //时间帧加
        pAxis->m_stSoftMotionEX.dMotionTime += m_dSoftMotionCycle;
    }
    //运动完成检测
    if((enPositionPlanningMode == itMotionData->PlanningMotionParam.PlanningMode)||(enStopPlanningMode == itMotionData->PlanningMotionParam.PlanningMode))
    {
        if(pAxis->m_stSoftMotionEX.dMotionTime > m_vSoftMotionPlanParams[pAxis->nAxisID].trackData.T)
        {
            itMotionData->bMotionDone = true;
        }
    }
    return AEC_SUCCESSED;
}