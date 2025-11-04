#include"MC_FollowPosition.h"
MC_FollowPosition::MC_FollowPosition(/* args */)
{
}

MC_FollowPosition::~MC_FollowPosition()
{
}

void MC_FollowPosition::operator()(CIA402Axis* axis)
{
    if(nullptr == axis)
    {
        return;
    }
    m_pCIA402Axis = axis;
}
void MC_FollowPosition::operator()(CIA402Axis* axis,bool bExecute,double dPosition,bool& bBusy,bool& bError,int& ErrorID)
{
    if(nullptr == axis)
    {
        m_bError = true;
        m_nErrorID = SMEC_INVALID_AXIS;
        return;
    }
    m_pCIA402Axis = axis;
    m_bExecute = bExecute;
    m_dPosition = dPosition;
    this->Execute();
    bBusy = m_bBusy;
    bError = m_bError;
    ErrorID = m_nErrorID;
}


void MC_FollowPosition::Execute()
{
    if(nullptr == m_pCIA402Axis)
    {
        m_bError = true;
        m_nErrorID = SMEC_INVALID_AXIS;
        return;
    }
    //输出初始化
    m_bBusy             = false;
    m_bError            = false;
    m_nErrorID           = SMEC_SUCCESSED;
    int res = AEC_SUCCESSED;
    if(m_bExecute)
    {
        res = m_pCIA402Axis->Axis_SetTargetPosition(m_dPosition);
    }
    if(AEC_SUCCESSED != res)
    {
        m_bError = true;
    }

    m_bBusy = true;
    return;
}