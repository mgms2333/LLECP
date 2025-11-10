#include"../SMbasic.h"
class MC_Stop:public SMbasic
{
private:

    void Execute()override;

public:
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dDeceleration,double dJerk,
                    bool& bDone,bool& bBusy,bool& bCommandAborted,bool& bError,int& ErrorID);
    MC_Stop(/* args */);
    ~MC_Stop();
};

