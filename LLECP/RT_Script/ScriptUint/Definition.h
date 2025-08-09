#include<vector>
#include<string>
#include <stdint.h> 
union  UN_Param
{
    bool bParm;
    uint32_t nParm;
};

struct  ST_State
{
    bool bState;
    int nPState;
    double dState;
    long lParm;
};

enum EX_RES
{
    EX_RES_NULL,
    EX_RES_Wait,
    EX_RES_Done,
    EX_RES_Error
};

enum class ST_TokenType {

    TokenType_NULL,//空
    TokenType_Number,//数字
    TokenType_Operator,//操作运算符
    TokenType_Parenthesis_L,//括号
    TokenType_Parenthesis_R,
    TokenType_Assignment,//赋值
    TokenType_Keyword,//变量
    TokenType_Minus//负号
};

enum class ST_OperatorType {
    OperatorType_NULL,
    OperatorType_PLUS,//+
    OperatorType_Subtract,//-
    OperatorType_Ride,//*
    OperatorType_Division,///
    OperatorType_Equal,//=
    OperatorType_Greater,//>
    OperatorType_Less,//<
    OperatorType_Unequal,//<>
    OperatorType_GreaterEqual,//>=
    OperatorType_LessEqual,//<=
    OperatorType_ERROR
};

union UN_TokenValue
{
    ST_OperatorType OperatorType;
    double NumberValue;
    uint32_t KeywordAddr;
};