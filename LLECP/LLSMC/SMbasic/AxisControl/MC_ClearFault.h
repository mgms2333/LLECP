#include"../SMbasic.h"
class MC_ClearFault :public SMbasic
{
private:
    FS_Reset m_fsClearFault;
    void Execute()override;

public:
    MC_ClearFault(/* args */);
    ~MC_ClearFault();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,bool &bBusy,bool &bDone, bool &bError,int &nErrorID);
};
