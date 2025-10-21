#include"SMbasic.h"


SMbasic::SMbasic(/* args */)
{
}

SMbasic::~SMbasic()
{
}

void SMbasic::Execute()
{
    return;
}

void SMbasic::operator()(CIA402Axis* axis)
{
    m_pCIA402Axis = axis;
    Execute();
}