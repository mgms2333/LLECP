
#include"../SMbasic.h"
class MC_SetEncodeRatio:public SMbasic
{
private:
    double m_dEncodeRatio;
    void Execute()override;
public:
    MC_SetEncodeRatio(/* args */);
    ~MC_SetEncodeRatio();

    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dEncodeRatio,bool& bDone,bool& bError,int& ErrorID);
};

