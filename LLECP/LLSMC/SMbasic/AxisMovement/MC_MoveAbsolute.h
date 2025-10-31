#include"../SMbasic.h"
class MC_MoveAbsolute:public SMbasic
{
private:


public:
    void Execute()override;
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dPosition,double dVelocity,double dAcceleration,double dDeceleration,double dJerk,EN_Direction enDirection,EN_BufferMode enBufferMode,
                    bool& bDone,bool& bBusy,bool& bCommandAborted,bool& bError,int& ErrorID);
    MC_MoveAbsolute(/* args */);
    ~MC_MoveAbsolute();
};

