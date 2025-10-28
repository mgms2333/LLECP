#include"../SMbasic.h"
class MC_MoveAbsolute:public SMbasic
{
private:
    bool m_bExecute;
    double m_dPosition;
    double m_dVelocity;
    double m_dAcceleration;
    double m_dDeceleration;
    double m_dJerk;
    EN_Direction m_enDirection;
    EN_BufferMode m_enBufferMode;
    ST_MotionUint m_MotionUint;


public:
    void Execute()override;
    void operator()(CIA402Axis* axis);
    void operator()(CIA402Axis* axis,bool bExecute,double dPosition,double dVelocity,double dAcceleration,double dDeceleration,double dJerk,EN_Direction enDirection,EN_BufferMode enBufferMode,
                    bool& bDone,bool& bBusy,bool& bCommandAborted,bool& bError,int& ErrorID);
    MC_MoveAbsolute(/* args */);
    ~MC_MoveAbsolute();
};

