#include"EtherCATMaster.h"

EtherCATMaster::EtherCATMaster(uint8_t nMasterIndex)
{
    m_nMasterIndex = nMasterIndex;
    m_bIsConfigDc = false;
    m_nShitTime = 0;
    m_fCycle = 1.0;
    m_bIsSimulation;
    m_sNetWork = "enp2s0";
    ti_Sleep.tv_sec = 0;
    ti_Sleep.tv_nsec = m_fCycle*1000000;

    m_bEtherCAT_RT = false;
    m_bEtherCAT_RT_Check = false;
}

EtherCATMaster::~EtherCATMaster()
{
}
int EtherCATMaster::CreateRT_Thread(std::thread* th,void (EtherCATMaster::*func)())
{
        
        th = new std::thread(func, this);
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
}
int EtherCATMaster::StartMaster()
{
    m_bEtherCAT_RT = true;
    m_bEtherCAT_RT_Check = true;
      /* create process data thread */
    CreateRT_Thread(&thEtherCAT_RT,&EtherCATMaster::EtherCAT_RT);
      /* create thread to handle slave error handling in OP */
    CreateRT_Thread(&thEtherCAT_RT_Check,&EtherCATMaster::EtherCAT_RT_Check);
      /* bringup network */
      //ecatbringup(m_sNetWork.c_str());
    return 0;
}
int EtherCATMaster::CloseMaster()
{
  return 0;
}

 void EtherCATMaster::EtherCAT_RT()
 {
    while (true)
    {
      if(m_bEtherCAT_RT)
      {
        printf("RunEtherCAT_RT\n");
      }
      osal_monotonic_sleep(&ti_Sleep);
    }
    
    return;
 }
 void EtherCATMaster::EtherCAT_RT_Check()
 {
    while (true)
    {
      if(m_bEtherCAT_RT_Check)
      {
        printf("RunEtherCAT_RT\n");
      }
      osal_monotonic_sleep(&ti_Sleep);
    }
    return; 
 }