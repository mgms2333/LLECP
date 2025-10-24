#pragma once
enum SMEC_SMbasicErrorCode
{
    SMEC_SUCCESSED,
    SMEC_TIMEOUT,
    SMEC_AXIS_STATUS_INTERCEPTION,//轴状态拦截此操作
    SMEC_ABNORMAL_AXIS_CHANGE,//异常轴改变
};