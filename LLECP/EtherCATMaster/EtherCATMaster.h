
#include"Slave.h"

class EtherCATMaster
{
private:
    //IGH
    uint8_t m_nMasterIndex;
    ec_master_t *m_pMaster = NULL;
    ec_domain_t *m_pDomain = NULL;
    /* data */
    uint8_t m_nSlaveNum;

    std::vector<std::string> m_vSlaveInfo;
public:
    EtherCATMaster(uint8_t nMasterIndex);
    ~EtherCATMaster();
    //cmd
    int StarttMaster_cmd();
    int RestartMaster_cmd();
    int PrintSlaveStruct_cmd(uint8_t SlaveID);
    int PrintSlaveState_cmd();
    int CheckSlaveState_OP_cmd();

    //InitMaster
    int InitMaster();
    int ScanSlave();
};
