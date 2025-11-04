
#include"../SMbasic.h"
class MC_SetSetSWLimit:public SMbasic
{
private:
    double m_dSetSWLimit;
    void Execute()override;
public:
    MC_SetSetSWLimit(/* args */);
    ~MC_SetSetSWLimit();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dSetSWLimit,bool& bDone,bool& bError,int& ErrorID);
};

