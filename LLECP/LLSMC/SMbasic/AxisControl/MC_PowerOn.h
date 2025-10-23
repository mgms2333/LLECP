#include"../SMbasic.h"
class MC_PowerOn :public SMbasic
{
private:
    FS_PowerOn m_fsPowerOn;
public:
    MC_PowerOn(/* args */);
    ~MC_PowerOn();

    void Execute()override;
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool Enabel,bool &bBusy,bool &bDone, bool &bError,int &nErrorID);
};
