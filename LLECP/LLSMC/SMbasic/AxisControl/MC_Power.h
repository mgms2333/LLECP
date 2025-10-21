#include"../SMbasic.h"
class MC_Power :public SMbasic
{
private:
    FS_PowerOn fs_PowerOn;
public:
    MC_Power(/* args */);
    ~MC_Power();

    void Execute()override;
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool Enabel,bool &bBusy,bool &bDone, bool &bEror,int &nErrorID);
};
