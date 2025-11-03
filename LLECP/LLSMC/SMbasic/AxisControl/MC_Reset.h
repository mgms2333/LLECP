#include"../SMbasic.h"
class MC_Reset :public SMbasic
{
private:
    FS_Reset m_fsReset;
    void Execute()override;
public:
    MC_Reset(/* args */);
    ~MC_Reset();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID);
};
