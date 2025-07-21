#include<string>
#include"ScriptUint/ScriptUint_Script.h"
#include"sCmdNameDefine.h"
//#include"HRC_Rbt.h"
//通过#define HRCRbt HRC_Rbt::instance()使用HRC_Rbt函数
#define SCRIPTNUMBER_MAX 64

class RT_ScriptSystem
{
private:
    ScriptUint_Script ScripList[SCRIPTNUMBER_MAX];
    //当前push的bufferIndex
    uint32_t nPushBufferIndex;
    //当前push的LineIndex
    uint32_t nPushLineIndex;
    //runIndex
    uint32_t nRunBufferIndex;
    uint32_t nRunLineIndex;
    uint32_t nRunCmdIndex;

public:
    RT_ScriptSystem(/* args */);
    ~RT_ScriptSystem();


    //队列结构
    int InitRT_ScriptSystem();
    int PushScriptCmd(std::string sCmd);
    int PushScriptCmdFinish();
    int PushLineBreak();
    int PushBufferBreak();
    int StartRT_Script();
    int StopRT_Script();
    //实时调用
    void RT_ScriptTick();
private:
    //执行器
    EX_RES RT_ScriptActuator(ScriptUint_Cmd* pUint_Cmd);
    //执行单元
    EX_RES ActuatorUint_IF(ScriptUint_Cmd* pUint_Cmd);

    //解释器
    ScriptUint_Cmd RT_ScriptInterpreter(std::string sCmd,int& nRet);

    //计算器
    ScriptUint_Token RT_ScriptCalculator(std::vector<ScriptUint_Token>);

private:
    int ResetRT_Script();
    int PushScriptCmd(ScriptUint_Cmd Cmd);
};
