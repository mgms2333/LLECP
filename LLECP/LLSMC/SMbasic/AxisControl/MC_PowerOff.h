#include"../SMbasic.h"
class MC_PowerOff :public SMbasic
{
private:
    FS_PowerOff m_fsPowerOff;
public:
    MC_PowerOff(/* args */);
    ~MC_PowerOff();

    void Execute()override;
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID);
};
