#include"RT_Script.h"
RT_ScriptSystem::RT_ScriptSystem(/* args */)
{
    v_sLogicalStatement.push_back("IF");
    v_sLogicalStatement.push_back("ELSE");
    v_sLogicalStatement.push_back("THEN");
    v_sLogicalStatement.push_back("FOR");
    v_sLogicalStatement.push_back("DO");
    v_sLogicalStatement.push_back("BY");
    v_sLogicalStatement.push_back("END_IF");
    v_sLogicalStatement.push_back("END_FOR");
    v_sLogicalStatement.push_back("WHILE");
    v_sLogicalStatement.push_back("END_WHILE");
    v_sLogicalStatement.push_back("WAIT");
    v_sLogicalStatement.push_back("SLEEP");
    v_sOperator.push_back("+");
    v_sOperator.push_back("-");
    v_sOperator.push_back("*");
    v_sOperator.push_back("/");
    v_sOperator.push_back(">");
    v_sOperator.push_back("<");
    v_sOperator.push_back("=");
    v_sOperator.push_back("<>");
    v_sOperator.push_back("<=");
    v_sOperator.push_back(">=");
}

RT_ScriptSystem::~RT_ScriptSystem()
{
    
}


int RT_ScriptSystem::InitRT_ScriptSystem()
{
    //清空数据
    nPushBufferIndex = 0;
    nPushLineIndex = 0;
    //清空buffer
    for (int i = 0;i < SCRIPTNUMBUFF_MAX;i++)
    {
        ScripList[i].InitScript();
    }
    printf("InitRT_Script()\n");
    return 0;
}
int RT_ScriptSystem::PushScriptCmd(ScriptUint_Cmd Cmd)
{
    return ScripList[nPushBufferIndex].PushCmd(Cmd);
}
int RT_ScriptSystem::PushScriptCmd(std::string sCmd)
{
    int nRet = 0;
    ScriptUint_Cmd Cmd = RT_ScriptInterpreter(sCmd,nRet);
    if (nRet!=0)
        return nRet;
    return PushScriptCmd(Cmd);
}
int RT_ScriptSystem::PushScriptCmdFinish()
{
    return 0;
}
int RT_ScriptSystem::PushLineBreak()
{
    ScripList[nPushBufferIndex].PushLineBreak();
    nPushLineIndex ++;
    return 0;
}
int RT_ScriptSystem::PushBufferBreak()
{
    nPushBufferIndex ++;
    return 0;
}

int RT_ScriptSystem::StartRT_Script()
{
    for (int i = 0;i < SCRIPTNUMBUFF_MAX;i++)
    {
        ScripList[i].ResetScript();
    }
    return 0;
}
int RT_ScriptSystem::StopRT_Script()
{
    return 0;
}
int RT_ScriptSystem::ResetRT_Script()
{
    return 0;
}

void RT_ScriptSystem::RT_ScriptTick()
{

}