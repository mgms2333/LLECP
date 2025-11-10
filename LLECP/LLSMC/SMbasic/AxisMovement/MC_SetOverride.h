#include"../SMbasic.h"
class MC_SetOverride:public SMbasic
{
private:
    void Execute()override;
    /* data */
    double m_dVelFactor;
    double m_dAccFactor;
    double m_dJerkFactor;
public:
    MC_SetOverride(/* args */);
    ~MC_SetOverride();
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dVelFactor,double dAccFactor,double dJerkFactor,bool &bDone, bool &bError,int &nErrorID);
};

