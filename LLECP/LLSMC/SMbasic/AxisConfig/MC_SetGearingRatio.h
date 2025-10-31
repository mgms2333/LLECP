
#include"../SMbasic.h"
class MC_SetGearingRatio:public SMbasic
{
private:
    double m_dGearingRatio;
public:
    MC_SetGearingRatio(/* args */);
    ~MC_SetGearingRatio();

    void Execute()override;
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dGearingRatio,bool& bDone,bool& bError,int& ErrorID);
};

