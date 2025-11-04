#include"MC_SetEncodeRatio.h"
MC_SetEncodeRatio::MC_SetEncodeRatio(/* args */)
{
}

MC_SetEncodeRatio::~MC_SetEncodeRatio()
{
}

void MC_SetEncodeRatio::operator()(CIA402Axis* axis)
{
    if(nullptr == axis)
    {
        return;
    }
    m_pCIA402Axis = axis;
}
void MC_SetEncodeRatio::operator()(CIA402Axis* axis,bool bExecute,double dEncodeRatio,bool& bDone,bool& bError,int& ErrorID)
{
    if(nullptr == axis)
    {
        m_bError = true;
        m_nErrorID = SMEC_INVALID_AXIS;
        return;
    }
    m_pCIA402Axis = axis;
    m_bExecute = bExecute;
    m_dEncodeRatio = dEncodeRatio;
    this->Execute();
    bDone = m_bDone;
    bError = m_bError;
    ErrorID = m_nErrorID;
}


void MC_SetEncodeRatio::Execute()
{
    if(nullptr == m_pCIA402Axis)
    {
        m_bError = true;
        m_nErrorID = SMEC_INVALID_AXIS;
        return;
    }
    //输出初始化
    m_bDone             = false;
    m_bError            = false;
    m_nErrorID           = SMEC_SUCCESSED;
    int res = AEC_SUCCESSED;
    //上升沿push
    if(m_Timer.R_TRIG(m_bExecute))
    {
        res = m_pCIA402Axis->Axis_SetEncodeRatio(m_dEncodeRatio);
    }
    if(AEC_SUCCESSED != res)
    {
        m_bError = true;
    }

    m_bDone = true;
    return;
}