
#include"../SMbasic.h"
class MC_FollowVelocity:public SMbasic
{
private:
    double m_dVelocity;
    void Execute()override;
public:
    MC_FollowVelocity(/* args */);
    ~MC_FollowVelocity();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dVelocity,bool& bBusy,bool& bError,int& ErrorID);
};

