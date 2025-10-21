#ifndef CIA402AXISDEF_H   // 如果还没定义这个宏
#define CIA402AXISDEF_H   // 定义宏，防止重复包含
#include"CIA402AxisDef.h"

class CIA402Axis
{
public:
    /* data */
    ST_SMCInitMap m_st_map;
public:
    CIA402Axis();
    ~CIA402Axis();
    int Axis_InitMap(ST_SMCInitMap st_map);

};


#endif