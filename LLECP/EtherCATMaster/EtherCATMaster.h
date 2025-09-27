#ifndef ETHERCATMASTER_H
#define ETHERCATMASTER_H
#include<vector>
#include<string>
#include"SOEM/include/soem/soem.h"
#include <stdint.h>
class EtherCATMaster
{
private:
    static ecx_contextt ctx;
    uint8_t m_nMasterIndex;
    /* data */
    uint8_t m_nSlaveNum;

    std::vector<std::string> m_vSlaveInfo;
public:
    EtherCATMaster(uint8_t nMasterIndex);
    ~EtherCATMaster();
    void Test();
};
#endif // ETHERCATMASTER_H