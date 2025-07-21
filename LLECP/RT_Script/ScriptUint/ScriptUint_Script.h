#pragma once
#include"ScriptUint_Line.h"
class ScriptUint_Script
{
private:
    const uint32_t nLineNum_Max = 999;
    uint32_t nLineNum_Now;
    uint32_t nLineRunIndex;
    bool bBufferRunDone=false;
    std::vector<ScriptUint_Line> v_LineList;
public:
    ScriptUint_Script();
    ~ScriptUint_Script();
    int InitScript();
    int ResetScript();
    int PushCmd(ScriptUint_Cmd scmd);
    int PushLineBreak();
    int GetLineNum();
    int GetLineRunIndex();
    ScriptUint_Cmd* GetRunLine();
    int CmdRunDone();
};