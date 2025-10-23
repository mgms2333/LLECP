#define AnaInSlavePos 0, 0
#define ZeroErr 0x00009434, 0x0000001b
#define frequency 2000
#define MY_STACK_SIZE 8192
#include <unistd.h>
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include<vector>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include"SMbasic_stdafx.h"
#include"RT_Script.h"
#include"EtherCATMaster.h"

#define TEST main


int RT_ScripTest() 
{

    
    RT_ScriptSystem *p = new RT_ScriptSystem();
    p->InitRT_ScriptSystem();
    p->PushScriptCmd("X:DINT;");
    p->PushScriptCmd("X13r:DINT;");
    std::string cmd = "X13r:=X-(10*(10+10));";
    //cmd = "bTest:=Real;";
    //cmd = "IF X>=1 THEN";
    p->PushScriptCmd("X:=10;");
    p->PushScriptCmd("X13r:=5;");
    p->PushScriptCmd(cmd);
    p->StartRT_Script();
    while(true)
    {
        p->RT_ScriptTick();
    }
    printf("Hello World!!!\n");
    delete p;
    p = nullptr;
    return 0;
}



MC_PowerOn fbMC_PowerOn;
MC_InitResetAxis fbMC_InitResetAxis;
std::vector<CIA402Axis*> v_Axis;
bool Enabel,bBusy,bDone, bError;int nErrorID;
std::thread RT_Thread;
timespec ti_Sleep;
void thFB_rt()
{
    while (true)
    {
        osal_monotonic_sleep(&ti_Sleep);
    }
}
int TEST()
{
    v_Axis.clear();
    EtherCATMaster* pMaster = new EtherCATMaster(0);  
    ti_Sleep.tv_sec =0;
    ti_Sleep.tv_nsec = 1000000;
    pMaster->StartMaster();
    pMaster->ConstructionCIA402AxisVec(&v_Axis);
    std::thread RT_Thread(thFB_rt);
    printf("InitReset\n");
    fbMC_InitResetAxis(v_Axis[0],false,bBusy,bDone,bError,nErrorID);
    while (true)
    {
        fbMC_InitResetAxis(v_Axis[0],true,bBusy,bDone,bError,nErrorID);
        if(bDone)
        {
             break;
        }
        osal_monotonic_sleep(&ti_Sleep);
    }
    printf("enable\n");
    fbMC_PowerOn(v_Axis[0],false,bBusy,bDone,bError,nErrorID);
    while (true)
    {
        fbMC_PowerOn(v_Axis[0],true,bBusy,bDone,bError,nErrorID);
        if(bDone)
        {
             break;
        }
        osal_monotonic_sleep(&ti_Sleep);
    }
    printf("Start\n");
    while (true)
    {
        int32_t pos = 0;
        v_Axis[0]->Axis_PDO_ReadActualPosition(pos);
        v_Axis[0]->Axis_PDO_SetTargetPosition(pos+500);
        osal_monotonic_sleep(&ti_Sleep);
    }

    return 0;
} 

