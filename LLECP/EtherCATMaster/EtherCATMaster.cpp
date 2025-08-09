#include"EtherCATMaster.h"

EtherCATMaster::EtherCATMaster(uint8_t nMasterIndex)
{
    m_nMasterIndex = nMasterIndex;
}

EtherCATMaster::~EtherCATMaster()
{
}
int EtherCATMaster::RestartMaster_cmd()
{
    system("/etc/init.d/ethercat restart"); 
    return 0;
}
int EtherCATMaster::StarttMaster_cmd()
{
    system("/etc/init.d/ethercat start"); 
    return 0;
}
int EtherCATMaster::PrintSlaveStruct_cmd(uint8_t SlaveID)
{
    std::string cmd = "ethercat cstruct -p ";
    cmd += std::to_string(SlaveID);

    std::array<char, 128> buffer;
    std::string result;

    // 打开管道
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        std::cerr << "popen() failed!" << std::endl;
        return -1;
    }

    // 读取命令行输出
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data(); // 追加到结果字符串中
    }

    // 打印到程序中
    std::cout << result;

    return 0;
}
int EtherCATMaster::PrintSlaveState_cmd()
{
    system("ethercat slaves"); 
    return 0;
}
int EtherCATMaster::CheckSlaveState_OP_cmd()
{
    system("ethercat state OP"); 
    return 0;
}

int EtherCATMaster::InitMaster()
{
    m_pMaster = ecrt_request_master(m_nMasterIndex);
    if(!m_pMaster){
        return -1;
    }
    m_pDomain = ecrt_master_create_domain(m_pMaster);
    if(!m_pDomain){
        return -1;
    }
}

int EtherCATMaster::ScanSlave()
{
    std::string cmd = "ethercat cstruct";
    std::array<char, 128> buffer;
    std::string sSlaveInfo;

    // 打开管道
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
        std::cerr << "popen() failed!" << std::endl;
        return -1;
    }

    // 读取命令行输出
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        sSlaveInfo += buffer.data(); // 追加到结果字符串中
    }
    std::vector<uint32_t> v_pos;
    size_t pos = 0;
    m_nSlaveNum = 0;
    std::string Findstring = "/* Master " + std::to_string(m_nMasterIndex) + ", Slave ";
    while ((pos = sSlaveInfo.find(Findstring, pos)) != std::string::npos) 
    {

        v_pos.push_back(pos);
        ++m_nSlaveNum;
        pos += Findstring.length(); // 继续向后找
    }
    v_pos.push_back(sSlaveInfo.length());
    m_vSlaveInfo.clear();
    for(size_t i = 0;i<m_nSlaveNum;i++)
    {
        std::string f  = "/* Master " + std::to_string(m_nMasterIndex) + ", Slave "+ std::to_string(i+1);
        int start = sSlaveInfo.find("/* Master " + std::to_string(m_nMasterIndex) + ", Slave "+ std::to_string(i));
        int end =sSlaveInfo.find("/* Master " + std::to_string(m_nMasterIndex) + ", Slave "+ std::to_string(i+1));
        std::string t = sSlaveInfo.substr(start,end);
        m_vSlaveInfo.push_back(t);
        std::cout<<t<<std::endl;
    }
    
    return m_nSlaveNum;


}