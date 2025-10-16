#include"FunctionBlockBase.h"


FunctionBlockBase::FunctionBlockBase(/* args */)
{
}

FunctionBlockBase::~FunctionBlockBase()
{
}

void FunctionBlockBase::Execute()
{
    return;
}

void FunctionBlockBase::operator()(CIA402Axis* axis)
{
    m_pCIA402Axis = axis;
    Execute();
}