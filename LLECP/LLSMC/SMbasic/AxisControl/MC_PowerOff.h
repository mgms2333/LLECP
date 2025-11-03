#include"../SMbasic.h"
class MC_PowerOff :public SMbasic
{
private:
    FS_PowerOff m_fsPowerOff;
    void Execute()override;

public:
    MC_PowerOff(/* args */);
    ~MC_PowerOff();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID);
};
