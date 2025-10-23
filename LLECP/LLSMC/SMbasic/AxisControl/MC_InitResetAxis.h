#include"../SMbasic.h"
class MC_InitResetAxis:public SMbasic
{
private:
    FS_InitResetAxis m_FSInitResetAxis;
    /* data */
public:
    MC_InitResetAxis(/* args */);
    ~MC_InitResetAxis();
    void Execute()override;
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool Enabel,bool &bBusy,bool &bDone, bool &bError,int &nErrorID);
};
