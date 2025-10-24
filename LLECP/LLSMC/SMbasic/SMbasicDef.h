#pragma once
#define SMC_TIME_OUT 500 * 1000
//Functional Status of MC_InitResetAxis 

enum FS_InitResetAxis
{
    ReadyInitResetAxis,
    InitResetting,
    InitResetFinish,
    InitResetError,
};
//Functional Status of MC_ClearFault
enum FS_ClearFault
{
    ReadyClearFault,
    StartClearFault,
    ClearFaulting,
    ClearFaultFinish,
    ClearFaultError,
};
//Functional Status of MC_Reset
enum FS_Reset
{
    ReadyReset,
    StartReset,
    Reseting,
    ResetFinish,
    ResetError,
};
//Functional Status of MC_PowerOn
enum FS_PowerOn
{
    ReadyPowerOn,
    StartPowerOn,
    PowerOning_0x06,
    PowerOning_0x07,
    PowerOning_0x0F,
    PowerOnFinish,
    PowerOnError,
};
//Functional Status of MC_PowerOff
enum FS_PowerOff
{
    ReadyPowerOff,
    StartPowerOff,
    PowerOff_0x07,
    PowerOff_0x06,
    PowerOffFinish,
    PowerOffError,
};

enum FS_Motion
{
    ReadyMotion,
    InMotion,
    MotionFinish,
    MotionError,
};
//Functional Status of MC_Home
enum FS_Home
{
    ReadyHome,
    StartHome,
    Home_0x1F,
    Home_0x06,
    HomeFinish,
    HomeError,
};


typedef enum
{
    en_ControlWord_Init                     = 0, 
    en_ControlWord_so                       = 1 << 0,  
    en_ControlWord_ev                       = 1 << 1,
    en_ControlWord_qs                       = 1 << 2,
    en_ControlWord_eo                       = 1 << 3,
    en_ControlWord_oms0                     = 1 << 4,
    en_ControlWord_oms1                     = 1 << 5,
    en_ControlWord_oms2                     = 1 << 6,
    en_ControlWord_fr                       = 1 << 7,
    en_ControlWord_h                        = 1 << 8,
    en_ControlWord_oms3                     = 1 << 9,
    en_ControlWord_r                        = 1 << 10,
    en_ControlWord_ms1                      = 1 << 11,
    en_ControlWord_ms2                      = 1 << 12,
    en_ControlWord_ms3                      = 1 << 13,
    en_ControlWord_ms4                      = 1 << 14,
    en_ControlWord_ms5                      = 1 << 15,

} EN_SMC_ControlWord;

typedef enum// : EC_T_USHORT
{
	en_StatusWord_rtso                      = 1 << 0,
	en_StatusWord_so                        = 1 << 1,
	en_StatusWord_oe                        = 1 << 2,
	en_StatusWord_f                         = 1 << 3,
	en_StatusWord_ve                        = 1 << 4,
	en_StatusWord_qs                        = 1 << 5,
	en_StatusWord_sod                       = 1 << 6,
	en_StatusWord_w                         = 1 << 7,
	en_StatusWord_ms                        = 1 << 8,
	en_StatusWord_rm                        = 1 << 9,
	en_StatusWord_tr                        = 1 << 10,
	en_StatusWord_ila                       = 1 << 11,
	en_StatusWord_oms1                      = 1 << 12,
	en_StatusWord_oms2                      = 1 << 13,
	en_StatusWord_ms1                       = 1 << 14,
	en_StatusWord_ms2                       = 1 << 15,
} EN_SMC_StatusWord;


