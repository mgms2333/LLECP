#ifndef SMbasic_H
#define SMbasic_H
#include"../SoftMotion/SoftMotion.h"
#include"SMbasicErrorCode.h"
#include"SMbasicDef.h"
class SMbasic
{
protected:
    CIA402Axis* m_pCIA402Axis;
    SMCTimer m_Timer;
    SMCTimer m_TimerTimeout;
    bool m_Enabel;
    bool m_bBusy;
    bool m_bCommandAborted;
    bool m_bDone;
    bool m_bError;
    int m_nErrorID;
public:
    SMbasic(/* args */);
    ~SMbasic();
    void operator()(CIA402Axis* axis);
protected:
    virtual void Execute();
};
#endif // SMbasic_H