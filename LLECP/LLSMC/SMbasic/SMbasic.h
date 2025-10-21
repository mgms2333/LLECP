#ifndef SMbasic_H
#define SMbasic_H
#include"../CIA402Axis/CIA402Axis.h"
#include"SMbasicDef.h"
class SMbasic
{
private:
    CIA402Axis* m_pCIA402Axis;
    /* data */
public:
    SMbasic(/* args */);
    ~SMbasic();
    void operator()(CIA402Axis* axis);
protected:
    virtual void Execute();
};
#endif // SMbasic_H