
#include"../SMbasic.h"
class MC_FollowTorque:public SMbasic
{
private:
    double m_dTorque;
    void Execute()override;
public:
    MC_FollowTorque(/* args */);
    ~MC_FollowTorque();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dTorque,bool& bBusy,bool& bError,int& ErrorID);
};

