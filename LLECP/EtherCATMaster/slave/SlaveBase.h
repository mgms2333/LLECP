#ifndef SLAVEBASE_H
#define SLAVEBASE_H
#include"SOEM/soem/soem.h"
#include<string>
#include"SlaveDef.h"
#include <stdio.h>
#include <stdint.h>

class SlaveBase
{
protected:
    //io类型
    EN_SlaveType m_SlaveType;

    uint32_t m_eep_man;
    /** ID from EEprom */
    uint32_t m_eep_id;
    /** revision from EEprom */
    uint32_t m_eep_rev;
    /* data */
public:
    SlaveBase(/* args */);
    ~SlaveBase();
    virtual int InitPDOmap(ec_slavet* ec_clave);
};

#endif
