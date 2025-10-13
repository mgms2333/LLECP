#ifndef ETHERCATMASTER_H
#define ETHERCATMASTER_H
#include<vector>
#include<string>
#include"SOEM/soem/soem.h"
#include <stdint.h>
#include<stdio.h>
#include"EtherCATDef.h"
#include <thread>

class EtherCATMaster
{
protected:
    static ecx_contextt ctx;
    
    uint8_t m_nMasterIndex;
    //ethercat master param
    bool m_bIsConfigDc;
    int m_nShitTime = 0;
    float m_fCycle = 1.0;
    bool m_bIsSimulation;
    std::string m_sNetWork = "enp2s0";

    //EtherCAT thread
    std::thread thEtherCAT_RT, thEtherCAT_RT_Check;
    /* data */
    uint8_t m_nSlaveNum;

    std::vector<std::string> m_vSlaveInfo;
    ec_slavet m_MasterInfo;




    int expectedWKC;
    int m_nWkc;

    //PDO

    int m_nRxPDOSize;//slave recv
    int m_nTxPDOSize;//slave send
    uint8_t  m_PDOmap[MAX_PDO_SIZE];

public:
    EtherCATMaster(uint8_t nMasterIndex);
    ~EtherCATMaster();
    int StartMaster();
    int CloseMaster();
    void Test();

protected:
    //Thread
    int CreateRT_Thread(std::thread* th,void (EtherCATMaster::*func)());
    void EtherCAT_RT();
    void EtherCAT_RT_Check();
    bool m_bEtherCAT_RT;
    bool m_bEtherCAT_RT_Check;
    timespec ti_Sleep;
};
#endif // ETHERCATMASTER_H