#pragma once

#include"ScriptUint_Token.h"

class ScriptUint_Cmd
{
public:
    //记录IF的else结束位置
    std::vector<UN_Param>v_Param;
    //Token列表
    std::vector<ScriptUint_Token>v_Token;
    //当前指令运行状态
    std::vector<ST_State>v_State;
    bool bCmdRunDone=false;
public:

    ScriptUint_Cmd();
    ~ScriptUint_Cmd();
    int ResetCommand();
    int CmdRunDone();
    ST_CmdType GetCmdType();
    int SetCmdType(ST_CmdType setCmdType);
private:
    ST_CmdType stCmdType;
};