#include"MC_Power.h"
MC_Power::MC_Power(/* args */)
{
}

MC_Power::~MC_Power()
{
}

void MC_Power::Execute()
{
    return;
}
void MC_Power::operator()(CIA402Axis* axis)
{
    this->Execute();
}
void MC_Power::operator()(CIA402Axis* axis,bool Enabel,bool &bBusy,bool &bDone, bool &bEror,int &nErrorID)
{
    this->Execute();
}