#include"ScriptUint_Script.h"

ScriptUint_Script::ScriptUint_Script()
{

}

ScriptUint_Script::~ScriptUint_Script()
{

}

int ScriptUint_Script::InitScript()
{
    for(int i = 0;i<v_LineList.size();i++)
    {
        //清空指令
        v_LineList.clear();
        //清空内存
        std::vector<ScriptUint_Line>().swap(v_LineList);
    }
    return 0;
}
int ScriptUint_Script::ResetScript()
{
    for(int i = 0;i<v_LineList.size();i++)
    {
        v_LineList[i].Reset();
    }
    nLineRunIndex = 0;
    bBufferRunDone = false;
    return 0;
}

int ScriptUint_Script::PushCmd(ScriptUint_Cmd cmd)
{
    return v_LineList[nLineNum_Now].PushCmd(cmd);
}

int ScriptUint_Script::PushLineBreak()
{
    if (nLineNum_Now == nLineNum_Max)
    return -1;
    nLineNum_Now++;
    return 0;
}
int ScriptUint_Script::GetLineNum()
{
    return nLineNum_Now;
}

ScriptUint_Cmd* ScriptUint_Script::GetRunLine()
{
    return v_LineList[nLineRunIndex].GetRunCmd();
}
int ScriptUint_Script::CmdRunDone()
{
    if(1 ==v_LineList[nLineRunIndex].CmdRunDone())
    {
        nLineRunIndex ++;
    }
    if(nLineRunIndex>nLineNum_Now)
        bBufferRunDone = true;
    return 0;
}

int ScriptUint_Script::GetLineRunIndex()
{
    return nLineRunIndex;
}