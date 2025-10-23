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
  sch_params.sched_priority = 99;   // 优先级范围通常 1~99（越高越实时）
  int policy = SCHED_FIFO;          // 实时调度策略 FIFO
  if (pthread_setschedparam(native, policy, &sch_params)) 
  {
      printf("无法设置实时优先级需要root权限\n");
  }
  th->detach();  
  return 0;
}
int EtherCATMaster::StartMaster()
{
    InitRT_Thread();
      /* bringup network */
      //ecatbringup(m_sNetWork.c_str());
    ScanSlave();
    InitSlave();
    return 0;
}
int EtherCATMaster::CloseMaster()
{
   /* stop SOEM, close socket */
   ecx_close(&m_ctx);
   return 0;
}

int EtherCATMaster::InitRT_Thread()
{
    m_bEtherCAT_RT = true;
    m_bEtherCAT_RT_Check = true;
      /* create process data thread */
    CreateRT_Thread(&thEtherCAT_RT,&EtherCATMaster::EtherCAT_RT);
      /* create thread to handle slave error handling in OP */
    //CreateRT_Thread(&thEtherCAT_RT_Check,&EtherCATMaster::EtherCAT_RT_Check);
    return 0;
}

 void EtherCATMaster::EtherCAT_RT()
 {
    printf("RunEtherCAT_RT start\n");
    while (true)
    {
      if(m_bEtherCAT_RT && m_ctx.slavecount!=0)
      {
        m_bInOP = true;
        if (!m_bIsSimulation)
        {
              // 发送总线数据
            #ifdef PDO_OVERLAB
                ec_send_overlap_processdata();  //重叠模式发送数据
            #else
                ecx_send_processdata(&m_ctx);
            #endif 
            // 接收总线数据
            m_wkc = ecx_receive_processdata(&m_ctx,EC_TIMEOUTRET);
        }
      }
      for(size_t i = 0;i<m_v_slave.size();i++)
      {
         m_v_slave[i]->Slave_RT();
      }
      osal_monotonic_sleep(&ti_Sleep);
    }
    return;
 }
 void EtherCATMaster::EtherCAT_RT_Check()
 {
  printf("EtherCAT_RT_Check start\n");
    while (true)
    {
      if(m_bEtherCAT_RT_Check)
      {

      }
      osal_monotonic_sleep(&ti_Sleep);
    }
    return; 
 }

 int EtherCATMaster::ScanSlave()
 {
   int cnt, i, j, nSM;
   uint16 ssigen;

   printf("Starting slaveinfo\n");

   /* initialise SOEM, bind socket to ifname */
   if (ecx_init(&m_ctx, m_sNetWork.c_str()))
   {
      printf("ecx_init on %s succeeded.\n", m_sNetWork.c_str());

      /* find and auto-config slaves */
      if (ecx_config_init(&m_ctx) > 0)
      {
         m_p_ECTgroup = &m_ctx.grouplist[0];
          if (ecx_statecheck(&m_ctx,0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE*4) != EC_STATE_PRE_OP)
                {
                    printf("[HRC.StartMaster]could not set EC_STATE_PRE_OP\n");
                    return false;
                }
         ecx_config_map_group(&m_ctx, m_PDOmap, 0);

         ecx_configdc(&m_ctx);
         while (m_ctx.ecaterror)
            printf("%s", ecx_elist2string(&m_ctx));
         printf("%d slaves found and configured.\n", m_ctx.slavecount);
         m_expectedWKC = (m_p_ECTgroup->outputsWKC * 2) + m_p_ECTgroup->inputsWKC;
         printf("Calculated workcounter %d\n", m_expectedWKC);
         /* wait for all slaves to reach SAFE_OP state */
         ecx_statecheck(&m_ctx, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);

         if (m_ctx.slavelist[0].state != EC_STATE_SAFE_OP)
         {
            printf("Not all slaves reached safe operational state.\n");
            ecx_readstate(&m_ctx);
            for (i = 1; i <= m_ctx.slavecount; i++)
            {
               if (m_ctx.slavelist[i].state != EC_STATE_SAFE_OP)
               {
                  printf("Slave %d State=%2x StatusCode=%4x : %s\n",
                         i, m_ctx.slavelist[i].state, m_ctx.slavelist[i].ALstatuscode, ec_ALstatuscode2string(m_ctx.slavelist[i].ALstatuscode));
               }
            }
         }
        //控制进入OP
        m_ctx.slavelist[0].state = EC_STATE_OPERATIONAL;
        ecx_writestate(&m_ctx,0);
        ecx_send_processdata(&m_ctx);
        m_nWkc = ecx_receive_processdata(&m_ctx,EC_TIMEOUTRET);
        ecx_statecheck(&m_ctx,0, EC_STATE_OPERATIONAL, 50000);
        // uint32 Isize, Osize;
        // ecx_readPDOmap(&m_ctx,1, &Osize, &Isize);
         writeSDO_INT(1, 0x1c12, 0x00, 0x00);
         writeSDO_INT(1, 0x1c12, 0x01, 0x1600);
         writeSDO_INT(1, 0x1c12, 0x00, 0x01);

         writeSDO_INT(1, 0x1c13, 0x00, 0x00);
         writeSDO_INT(1, 0x1c13, 0x01, 0x1a00);
         writeSDO_INT(1, 0x1c13, 0x00, 0x01);
         ecx_readstate(&m_ctx);
         for (cnt = 1; cnt <= m_ctx.slavecount; cnt++)
         {
            printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                   cnt, m_ctx.slavelist[cnt].name, m_ctx.slavelist[cnt].Obits, m_ctx.slavelist[cnt].Ibits,
                   m_ctx.slavelist[cnt].state, m_ctx.slavelist[cnt].pdelay, m_ctx.slavelist[cnt].hasdc);
            if (m_ctx.slavelist[cnt].hasdc) printf(" DCParentport:%d\n", m_ctx.slavelist[cnt].parentport);
            printf(" Activeports:%d.%d.%d.%d\n", (m_ctx.slavelist[cnt].activeports & 0x01) > 0,
                   (m_ctx.slavelist[cnt].activeports & 0x02) > 0,
                   (m_ctx.slavelist[cnt].activeports & 0x04) > 0,
                   (m_ctx.slavelist[cnt].activeports & 0x08) > 0);
            printf(" Configured address: %4.4x\n", m_ctx.slavelist[cnt].configadr);
            printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n", (int)m_ctx.slavelist[cnt].eep_man, (int)m_ctx.slavelist[cnt].eep_id, (int)m_ctx.slavelist[cnt].eep_rev);
            for (nSM = 0; nSM < EC_MAXSM; nSM++)
            {
               if (m_ctx.slavelist[cnt].SM[nSM].StartAddr > 0)
                  printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n", nSM, etohs(m_ctx.slavelist[cnt].SM[nSM].StartAddr), etohs(m_ctx.slavelist[cnt].SM[nSM].SMlength),
                         etohl(m_ctx.slavelist[cnt].SM[nSM].SMflags), m_ctx.slavelist[cnt].SMtype[nSM]);
            }
            for (j = 0; j < m_ctx.slavelist[cnt].FMMUunused; j++)
            {
               printf(" FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x Act:%2.2x\n", j,
                      etohl(m_ctx.slavelist[cnt].FMMU[j].LogStart), etohs(m_ctx.slavelist[cnt].FMMU[j].LogLength), m_ctx.slavelist[cnt].FMMU[j].LogStartbit,
                      m_ctx.slavelist[cnt].FMMU[j].LogEndbit, etohs(m_ctx.slavelist[cnt].FMMU[j].PhysStart), m_ctx.slavelist[cnt].FMMU[j].PhysStartBit,
                      m_ctx.slavelist[cnt].FMMU[j].FMMUtype, m_ctx.slavelist[cnt].FMMU[j].FMMUactive);
            }
            printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n",
                   m_ctx.slavelist[cnt].FMMU0func, m_ctx.slavelist[cnt].FMMU1func, m_ctx.slavelist[cnt].FMMU2func, m_ctx.slavelist[cnt].FMMU3func);
            printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n", m_ctx.slavelist[cnt].mbx_l, m_ctx.slavelist[cnt].mbx_rl, m_ctx.slavelist[cnt].mbx_proto);
            ssigen = ecx_siifind(&m_ctx, cnt, ECT_SII_GENERAL);
            m_ctx.slavelist[cnt].state = EC_STATE_OPERATIONAL;
            ecx_writestate(&m_ctx,cnt);
            ecx_send_processdata(&m_ctx);
            m_nWkc = ecx_receive_processdata(&m_ctx,EC_TIMEOUTRET);
            ecx_statecheck(&m_ctx,0, EC_STATE_OPERATIONAL, 50000);
            /* SII general section */
            if (ssigen)
            {
               m_ctx.slavelist[cnt].CoEdetails = ecx_siigetbyte(&m_ctx, cnt, ssigen + 0x07);
               m_ctx.slavelist[cnt].FoEdetails = ecx_siigetbyte(&m_ctx, cnt, ssigen + 0x08);
               m_ctx.slavelist[cnt].EoEdetails = ecx_siigetbyte(&m_ctx, cnt, ssigen + 0x09);
               m_ctx.slavelist[cnt].SoEdetails = ecx_siigetbyte(&m_ctx, cnt, ssigen + 0x0a);
               if ((ecx_siigetbyte(&m_ctx, cnt, ssigen + 0x0d) & 0x02) > 0)
               {
                  m_ctx.slavelist[cnt].blockLRW = 1;
                  m_ctx.slavelist[0].blockLRW++;
               }
               m_ctx.slavelist[cnt].Ebuscurrent = ecx_siigetbyte(&m_ctx, cnt, ssigen + 0x0e);
               m_ctx.slavelist[cnt].Ebuscurrent += ecx_siigetbyte(&m_ctx, cnt, ssigen + 0x0f) << 8;
               m_ctx.slavelist[0].Ebuscurrent += m_ctx.slavelist[cnt].Ebuscurrent;
            }
            printf(" CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE details: %2.2x\n",
                   m_ctx.slavelist[cnt].CoEdetails, m_ctx.slavelist[cnt].FoEdetails, m_ctx.slavelist[cnt].EoEdetails, m_ctx.slavelist[cnt].SoEdetails);
            printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
                   m_ctx.slavelist[cnt].Ebuscurrent, m_ctx.slavelist[cnt].blockLRW);
         }
      }
      else
      {
         printf("No slaves found!\n");
      }
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", m_sNetWork);
   }
 }

int EtherCATMaster::InitSlave()
{
  SlaveBase* pSlave;
  for (size_t i = 1; i <= m_ctx.slavecount; i++)
  {
    switch ((int)m_ctx.slavelist[i].eep_id)
    {
      case 27:
          m_v_slave.clear();
          pSlave = new NETX_50_RE_ECS();
        //  writeSDO_INT(i, 0x1c12, 0x00, 0x00);
        //  writeSDO_INT(i, 0x1c12, 0x01, 0x1600);
        //  writeSDO_INT(i, 0x1c12, 0x00, 0x01);

        //  writeSDO_INT(i, 0x1c13, 0x00, 0x00);
        //  writeSDO_INT(i, 0x1c13, 0x01, 0x1a00);
        //  writeSDO_INT(i, 0x1c13, 0x00, 0x01);
          pSlave->InitPDOmap(&m_ctx.slavelist[i]);
          m_v_slave.push_back(pSlave);
          break;
      
      default:
          break;
    }
  }
  return 0;
}

int EtherCATMaster::ConstructionCIA402AxisVec(std::vector<CIA402Axis*>* v_Axis)
{
  uint16 res =0;
  for(size_t i =0;i<m_v_slave.size();i++)
  {
    CIA402Axis *p_Axis;
    if(m_v_slave[i]->ConstructionCIA402Axis(p_Axis)!=-1)
    {
      v_Axis->push_back(p_Axis);
      res++;
    }
  }
  return res;
}