#include"../EtherCATMaster/EtherCATMaster.h"
int main()
{
    EtherCATMaster* pMaster = new EtherCATMaster(0);  
    pMaster->ScanSlave();
    return 0;
}