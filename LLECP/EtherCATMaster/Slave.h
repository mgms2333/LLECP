#include<sys/mman.h>
#include<vector>
#include<iostream>
#include <regex>
#include <cstdio>     // for popen, fgets
#include <iostream>   // for std::cout
#include <memory>     // for std::unique_ptr
#include <array>      // for std::array
#include"ecrt.h"
#include<limits.h>
#include<pthread.h>
#include<sched.h>
#include<stdio.h>
#include<stdlib.h>
class Slave
{
private:
    uint8_t m_nSlaveID;
    uint32_t m_nVendorID;
    uint32_t m_nProductCode;
    uint32_t m_nRevisionNumber;
    uint32_t m_PDO_NUM;
    /* data */
public:
    Slave(/* args */);
    ~Slave();
};
