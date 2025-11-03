#include"../SMbasic.h"
class MC_InitResetAxis:public SMbasic
{
private:
    FS_InitResetAxis m_FSInitResetAxis;
    void Execute()override;

    /* data */
public:
    MC_InitResetAxis(/* args */);
    ~MC_InitResetAxis();
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID);
};
