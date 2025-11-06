#include"../SMbasic.h"
class MC_MoveRelative:public SMbasic
{
private:

    void Execute()override;

public:
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dPosition,double dVelocity,double dAcceleration,double dDeceleration,double dJerk,EN_Direction enDirection,EN_BufferMode enBufferMode,
                    bool& bDone,bool& bBusy,bool& bCommandAborted,bool& bError,int& ErrorID);
    MC_MoveRelative(/* args */);
    ~MC_MoveRelative();
};

