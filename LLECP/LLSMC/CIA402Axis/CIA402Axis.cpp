#include"CIA402Axis.h"
CIA402Axis::CIA402Axis() { /* 初始化 */ }
CIA402Axis::~CIA402Axis(){}

int CIA402Axis::Axis_InitMap(ST_SMCInitMap st_map)noexcept
{
    m_st_map = st_map;
    int32_t s = *m_st_map.pActualPosition;
    return 0;
}