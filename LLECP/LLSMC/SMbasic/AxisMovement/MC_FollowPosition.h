
#include"../SMbasic.h"
class MC_FollowPosition:public SMbasic
{
private:
    double m_dPosition;
    void Execute()override;
public:
    MC_FollowPosition(/* args */);
    ~MC_FollowPosition();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dPosition,bool& bBusy,bool& bError,int& ErrorID);
};

