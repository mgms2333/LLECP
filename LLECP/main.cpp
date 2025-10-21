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



MC_Power fbMC_Power;
std::vector<CIA402Axis*> v_Axis;
bool Enabel,bBusy,bDone, bEror;int nErrorID;
std::thread RT_Thread;
timespec ti_Sleep;
int CreateRT_Thread(std::thread* th,void (*func)())
{
        
  th = new std::thread(func);
  // 获取 pthread 原生句柄
  pthread_t native = th->native_handle();
  // 设置调度策略与优先级
  sched_param sch_params;
  sch_params.sched_priority = 1;   // 优先级范围通常 1~99（越高越实时）
  int policy = SCHED_FIFO;          // 实时调度策略 FIFO
  if (pthread_setschedparam(native, policy, &sch_params)) 
  {
      printf("无法设置实时优先级需要root权限\n");
  }
  th->detach();  
  return 0;
}
void thFB_rt()
{
    while (true)
    {
        fbMC_Power(v_Axis[0],true,bBusy,bDone,bEror,nErrorID);
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
    CreateRT_Thread(&RT_Thread,&thFB_rt);
    int c =0;
    while (true)
    {
        *(v_Axis[0]->m_st_map.pTargetPosition) = *(v_Axis[0]->m_st_map.pActualPosition);
        *(v_Axis[0]->m_st_map.pControlword) = 0x86;
        osal_monotonic_sleep(&ti_Sleep);
        c++;
        if(c == 1000)
        {
             c = 0;
             break;
        }
    }
    

    while (true)
    {
        *(v_Axis[0]->m_st_map.pControlword) = 6;
        *(v_Axis[0]->m_st_map.pTargetPosition) = *(v_Axis[0]->m_st_map.pActualPosition);
        osal_monotonic_sleep(&ti_Sleep);
        c++;
        if(c == 2000)
        {
             c = 0;
             break;
        }
    }

    while (true)
    {
        *(v_Axis[0]->m_st_map.pControlword) = 7;
        *(v_Axis[0]->m_st_map.pTargetPosition) = *(v_Axis[0]->m_st_map.pActualPosition);
        osal_monotonic_sleep(&ti_Sleep);
        c++;
        if(c == 2000)
        {
             c = 0;
             break;
        }
    }

    
    while (true)
    {
        *(v_Axis[0]->m_st_map.pControlword) = 15;
        *(v_Axis[0]->m_st_map.pTargetPosition) = *(v_Axis[0]->m_st_map.pActualPosition);
        osal_monotonic_sleep(&ti_Sleep);
        c++;
        if(c == 2000)
        {
             c = 0;
             break;
        }
    }
    printf("Start\n");
    while (true)
    {
        *(v_Axis[0]->m_st_map.pTargetPosition) = *(v_Axis[0]->m_st_map.pActualPosition) + 500;
        osal_monotonic_sleep(&ti_Sleep);
    }

    return 0;
} 

