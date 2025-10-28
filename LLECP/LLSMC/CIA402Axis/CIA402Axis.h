#ifndef CIA402AXISDEF_H   // 如果还没定义这个宏
#define CIA402AXISDEF_H   // 定义宏，防止重复包含
#include"CIA402AxisDef.h"
#include"AxisErrorCode.h"
#define Axis_PDO_CONTROLWORD  *(m_st_map.pControlword)
#define Axis_PDO_STATUSWORD  *(m_st_map.pStatusWord)
#define Axis_PDO_TARGETPOSITION  *(m_st_map.pTargetPosition)
#define Axis_PDO_ACTUALPOSITION  *(m_st_map.pActualPosition)
#define Axis_PDO_TARGETTORQUE  *(m_st_map.pTargetTorque)
#define Axis_PDO_ACTUALTORQUE  *(m_st_map.pActualTorque)
#define Axis_PDO_TARGETVELOCITY *(m_st_map.pTargetVelocity)
#define Axis_PDO_ACTVELOCITY  *(m_st_map.pActualVelocity)
#define Axis_PDO_TARGETMODESOFOPERATION  *(m_st_map.pTargetModesOfOperation)
#define Axis_PDO_ACTUALMODESOFOPERATION  *(m_st_map.pActualModesOfOperation)
#define Axis_PDO_ERRORCODE  *(m_st_map.pErrorCode)
#define Axis_PDO_DIGITALOUTPUTS *(m_st_map.pDigitalOutputs) 
#define Axis_PDO_DIGITALINPUTS *(m_st_map.pDigitalInputs)
class SoftMotion;

class CIA402Axis 
{
    friend SoftMotion;
protected:
    //connfig
    bool m_bVirtual;
    uint32_t m_nAxisID;
    ST_SMC_PDO_Virtual m_stPDO_Virtual;
    ST_SMCInitMap m_st_map;
    ST_SMCAxisConfiguration m_stAxisConfiguration;
    double m_dOverride;
    double m_dControlCycle;//ms

    //Data
    ST_SMCAxisMotionData m_stAxisMotionData_now;
    ST_SMCAxisMotionData m_stAxisMotionData_old[5];

    //state
    EN_AxisMotionState m_enAxisMotionState;//sm
    ST_SoftMotionData m_stSoftMotion;

public:
    //external data
    //axis data
    int nCmdControlWord;      // 控制字（主站发往从站）
    int nActStatusWord;       // 状态字（从站反馈主站）

    // --- 目标位置信息 ---
    double dSetPosition;      // 目标位置（工程单位）
    int    TargetPosition_PDO;// 目标位置（PDO映射整数值）
    // --- 目标运动参数 ---
    double dSetVelocity;      // 目标速度
    // --- 补偿或控制器内部计算值 ---
    double dSetVelocity_c;    // 修正后速度（补偿）
    double dSetAcceleration_c;// 修正后加速度
    double dSetJerk_c;        // 修正后跃度
    double dSetCurrent;       // 目标电流/力矩（A 或 %）
    int    pActualPosition_PDO; // 实际位置（PDO原始值）
    double dActPosition;        // 实际位置（工程单位）
    // --- 实际运动参数 ---
    double dActVelocity;        // 实际速度
    double dActAcceleration;    // 实际加速度
    double dActJerk;            // 实际跃度
    // --- 实际补偿参数 ---
    double dActVelocity_c;      // 实际补偿后速度
    double dActAcceleration_c;  // 实际补偿后加速度
    double dActJerk_c;          // 实际补偿后跃度
    double dActCurrent;         // 实际电流/力矩反馈

    // 模式与错误信息（Mode / Error）
    int nCmdModeOpration;    // 当前指令的操作模式（例如 Profile Position、Cyclic Synchronous Position 等）
    int nActModeOpration;    // 从站反馈的操作模式
    int nAxisErrorID;        // 轴错误代码（Fault ID）
    
    //state
    bool bVirtual;    
    uint32_t nAxisID;
    EN_AxisMotionState enAxisMotionState;
    int m_nErrorCode;
    
    //config
    double dGearRatio;
    uint32_t nEncodeRatio;
    int nEncodeDirection;
    int nEncodeHomePos;
    double dCurrentScales;
    double dTorqueFeedforwardScale;
    double dVelocityScale;
    int nCurrentDirection;
    double dPositive;
    double dNegative;
    double dOverride;
public:
    CIA402Axis(bool bVirtual);
    ~CIA402Axis();
    int Axis_InitMap(ST_SMCInitMap st_map);
    int AxisSetAxisState(EN_AxisMotionState enAxisMotionState);
    EN_AxisMotionState AxisReadAxisState();
    bool AxisCheckError(int& nErrorID);
    bool AxisCheckError();
    //softmotion直接控制pdo
    // ===== 写入（RxPDO） =====
    int Axis_PDO_SetControlword(uint16_t Controlword);
    int Axis_PDO_SetTargetPosition(int32_t TargetPosition);
    int Axis_PDO_SetTargetVelocity(int32_t TargetVelocity);
    int Axis_PDO_SetTargetTorque(int16_t TargetTorque);
    int Axis_PDO_SetModesOfOperation(uint8_t mode);
    int Axis_PDO_SetDigitalOutputs(uint32_t DigitalOutputs);

    // ===== 读取（TxPDO） =====
    int Axis_PDO_ReadStatusWord(uint16_t& StatusWord);
    int Axis_PDO_ReadErrorCode(uint16_t& ErrorCode);
    int Axis_PDO_ReadActualPosition(int32_t& ActualPosition);
    int Axis_PDO_ReadActualVelocity(int32_t& ActualVelocity);
    int Axis_PDO_ReadActualTorque(int16_t& ActualTorque);
    int Axis_PDO_ReadModesOfOperation(uint8_t& mode);
    int Axis_PDO_ReadDigitalInputs(uint32_t& inputs);
    int Axis_PDO_ReadDigitalOutputs(uint32_t& outputs);

    // ===== 读回目标值缓存或虚拟轴调试 =====
    int Axis_PDO_ReadControlword(uint16_t& Controlword);
    int Axis_PDO_ReadTargetPosition(int32_t& TargetPosition);
    int Axis_PDO_ReadTargetVelocity(int32_t& TargetVelocity);
    int Axis_PDO_ReadTargetTorque(int16_t& TargetTorque);
    int Axis_PDO_ReadTargetModesOfOperation(uint8_t& mode);



    //MC功能块控制
    int SoftMotion_PDO_SetControlword(uint16_t Controlword);
    int SoftMotion_PDO_SetTargetPosition(int32_t TargetPosition);
    int SoftMotion_PDO_SetTargetVelocity(int32_t TargetVelocity);
    int SoftMotion_PDO_SetTargetTorque(int16_t TargetTorque);
    int SoftMotion_PDO_SetModesOfOperation(EN_ModesOfOperation mode);
    int SoftMotion_PDO_SetDigitalOutputs(uint32_t DigitalOutputs);
    int SoftMotion_PDO_ReadStatusWord(uint16_t& StatusWord);
    int SoftMotion_PDO_ReadErrorCode(uint16_t& ErrorCode);
    int SoftMotion_PDO_ReadActualPosition(int32_t& ActualPosition);
    int SoftMotion_PDO_ReadActualVelocity(int32_t& ActualVelocity);
    int SoftMotion_PDO_ReadActualTorque(int16_t& ActualTorque);
    int SoftMotion_PDO_ReadModesOfOperation(EN_ModesOfOperation& mode);
    int SoftMotion_PDO_ReadDigitalInputs(uint32_t& inputs);
    int SoftMotion_PDO_ReadDigitalOutputs(uint32_t& outputs);
    int SoftMotion_PDO_ReadControlword(uint16_t& Controlword);
    int SoftMotion_PDO_ReadTargetPosition(int32_t& TargetPosition);
    int SoftMotion_PDO_ReadTargetVelocity(int32_t& TargetVelocity);
    int SoftMotion_PDO_ReadTargetTorque(int16_t& TargetTorque);
    int SoftMotion_PDO_ReadTargetModesOfOperation(uint8_t& mode);



    int SoftMotion_PushMotionUint2SoftMotion(const ST_MotionUint STMotionUint);
    int SoftMotion_GetMotionUintFromSoftMotion(ST_MotionUint& stMotionUint);


    //Config
    //设置减速比，调用即时生效*******减速比定义：在设置的编码器分辨率下一个编码器分辨率周期下的运动距离*****对于HR机器人应该是360/101
    int Axis_SetGearRatio(double dGearRatio);
    //设置编码器分辨率，调用即时生效
    int Axis_SetEncodeRatio(uint32_t nEncodeRatio);
    //设置编码器方向实时生效
    int Axis_SetEncodeDirection(int nEncodeDirection);
    //设置编码器原点，实时生效
    int Axis_SetEncodeHomePos(int32_t nEncodeHomePos);
    //设置电流常数，实时生效
    int Axis_SetCurrentScales(double dCurrentScales);
    //设置速度常数
    int Axis_SetVelocityScale(double dVelocityScale);
    //设置电流方向，实时生效
    int Axis_SetCurrentDirection(int nCurrentDirection);
    //设置硬件边界
    int Axis_SetSWLimit(double dPositive,double dNegative);
    //设置控制周期(ms)
    int Axis_SetControlCycle(double dControlCycle);
    //驱动赋错误码
    int Axis_SetDriveErrorCode(int ErrorCode);

protected:
    //轴的实时函数
    void Axis_RT();
    
};


#endif