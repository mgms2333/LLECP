#include"RT_Script.h"
EX_RES RT_ScriptSystem::RT_ScriptActuator(ScriptUint_Cmd* pUint_Cmd)
{
    EX_RES stRes = EX_RES::EX_RES_NULL;
    switch (pUint_Cmd->GetCmdType())
    {
    case CmdType_IF:
        stRes = ActuatorUint_IF(pUint_Cmd);
        break;
    
    default:
        break;
    }
    return stRes;
}