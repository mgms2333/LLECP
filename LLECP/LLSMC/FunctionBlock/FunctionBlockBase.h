#ifndef FUNCTIONBLOCKBASE_H
#define FUNCTIONBLOCKBASE_H
#include"../CIA402Axis/CIA402Axis.h"
class FunctionBlockBase
{
private:
    CIA402Axis* m_pCIA402Axis;
    /* data */
public:
    FunctionBlockBase(/* args */);
    ~FunctionBlockBase();
    void operator()(CIA402Axis* axis);
protected:
    virtual void Execute();
};
#endif // FUNCTIONBLOCKBASE_H