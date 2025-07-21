enum ST_CmdType
{
    CmdType_NULL = 0,
    //逻辑指令
    CmdType_IF,
    CmdType_ELSIF,
    CmdType_ELSE,
    CmdType_END_if,
    CmdType_LOOP,
    CmdType_END_LOOP,
    CmdType_FOR,
    CmdType_END_FOR,
    CmdType_WAIT,

    CmdType_CALCULATION,

    CmdType_ENABLE,
    CmdType_DISABLE,

    CmdType_UsrFun,

    CmdType_Max
};