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
MC_PowerOff fbMC_PowerOff;
MC_InitResetAxis fbMC_InitResetAxis;
MC_MoveAbsolute fbMoveAbsolute;
MC_MoveRelative fbMoveRelative;
MC_MoveVelocity fbMoveVelocity;
SoftMotion* pSoftMotion;
std::vector<CIA402Axis*> v_Axis;
bool Enabel,bBusy,bDone,bInVelocity, bError,bCommandAborted;int nErrorID;
timespec ti_Sleep;
int TEST()
{
    v_Axis.clear();
    EtherCATMaster* pMaster = new EtherCATMaster(0);  
    pMaster->StartMaster();
    pMaster->ConstructionCIA402AxisVec(&v_Axis);
    pSoftMotion = new SoftMotion(v_Axis);
    pSoftMotion->SetSoftMotionCycle(0.001);
    struct timespec ts{0, 500000};   // 0 秒 + 1 000 000 纳秒 = 1 毫秒
    printf("InitReset\n");
    fbMC_InitResetAxis(v_Axis[0],false,bBusy,bDone,bError,nErrorID);
    while (true)
    {
        fbMC_InitResetAxis(v_Axis[0],true,bBusy,bDone,bError,nErrorID);
        if(bDone)
        {
             break;
        }
        pSoftMotion->SoftMotionRun();
        nanosleep(&ts, nullptr);
    }
    printf("enable\n");
    fbMC_PowerOn(v_Axis[0],false,bBusy,bDone,bError,nErrorID);
    int c = 0;
    while (true)
    {
        fbMC_PowerOn(v_Axis[0],true,bBusy,bDone,bError,nErrorID);
        if(bDone)
        {
            double pos = v_Axis[0]->dActPosition;
            double rel = 5;
            printf("AxisPos:%f\n",pos);
            // printf("SetMovePos:%f\n",pos+rel);
            // fbMoveRelative(v_Axis[0],false,rel,0.5,0.5,10,100,EN_Direction::enPositive,EN_BufferMode::enAborting,bDone,bBusy,bCommandAborted,bError,nErrorID);
            // fbMoveRelative(v_Axis[0],true,rel,0.5,0.5,10,100,EN_Direction::enPositive,EN_BufferMode::enAborting,bDone,bBusy,bCommandAborted,bError,nErrorID);
            fbMoveVelocity(v_Axis[0],false,0.5,10,10,100,EN_Direction::enPositive,bInVelocity,bBusy,bCommandAborted,bError,nErrorID);
            fbMoveVelocity(v_Axis[0],true,0.5,10,10,100,EN_Direction::enPositive,bInVelocity,bBusy,bCommandAborted,bError,nErrorID);
            break;
        }
        pSoftMotion->SoftMotionRun();
        nanosleep(&ts, nullptr);
    }
    while (true)
    {
        pSoftMotion->SoftMotionRun();
        nanosleep(&ts, nullptr);
    }
    while (true)
    {
        fbMoveRelative(v_Axis[0],true,5,0.5,0.5,10,100,EN_Direction::enPositive,EN_BufferMode::enAborting,bDone,bBusy,bCommandAborted,bError,nErrorID);
        if(bDone)
        {
            c = c+1;
            if(c>5000)
            {
                printf("EndPos:%f\n",v_Axis[0]->dActPosition);
                printf("disable\n");
                fbMC_PowerOff(v_Axis[0],false,bBusy,bDone,bError,nErrorID);
                break;
            }
        }
        pSoftMotion->SoftMotionRun();
        nanosleep(&ts, nullptr);
    }
    while (true)
    {
        if(!bDone)
        {
            fbMC_PowerOff(v_Axis[0],true,bBusy,bDone,bError,nErrorID);
            if(bDone)
            {
                printf("finish\n");
            }
        }
        pSoftMotion->SoftMotionRun();
        nanosleep(&ts, nullptr);
    }
    return 0;
} 

