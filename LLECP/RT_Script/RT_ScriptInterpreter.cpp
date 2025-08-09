#include"RT_Script.h"

ScriptUint_Cmd RT_ScriptSystem::RT_ScriptInterpreter(std::string sCmd,int& nRet)
{
    std::vector<ScriptUint_Token>v_Token;
    std::string type = "";
    for (size_t i = 0; i < sCmd.length(); i++)
    {
        ScriptUint_Token Token;
        //校验是否运算符
        if(IsOperator(sCmd[i]))
        {
            //尝试读取下一个字符
            if(IsOperator(sCmd[i+1]))
            {
                std::string sToken = std::string(1, sCmd[i]) + sCmd[i+1];
                Token = AnalysisToken_Operator(sToken);
                i++;
                continue;
            }
        Token = AnalysisToken_Operator(std::string(1, sCmd[i]));
        v_Token.push_back(Token);
        }

        //校验是否为变量，首为字符为变量
        if(IsCharacter(sCmd[i]))
        {
            std::string sVariable="";
            while(IsVariable(sCmd[i]))
            {
                sVariable += sCmd[i];
                i++;
            }
            Token = AnalysisToken_Variable(sVariable);
        }
    }
    
    ScriptUint_Cmd cmd;
    cmd.v_Token = v_Token;
    return cmd;
}

bool RT_ScriptSystem::IsOperator(const char c)
{
    for(uint16_t i = 0;i<v_sOperator.size();i++)
    {
        if(c == v_sOperator[i][0])
            return true;
    }
    return false;
}
bool RT_ScriptSystem::IsVariable(const char c)
{
    return (c >= 'A' && c <= 'Z') ||   // 大写字母
           (c >= 'a' && c <= 'z') ||   // 小写字母
           (c >= '0' && c <= '9') ||   // 数字
           c == '_' ||                 // 下划线
           c == '[' ||                 // 左方括号
           c == ']';                   // 右方括号
}

bool RT_ScriptSystem::IsCharacter(const char c)
{
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

ScriptUint_Token RT_ScriptSystem::AnalysisToken_Operator(std::string sToken)
{
    ScriptUint_Token Token;
    Token.TokenType = ST_TokenType::TokenType_Operator;
    if (sToken == "+") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_PLUS;
    } 
    else if (sToken == "-") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_Subtract;
    } 
    else if (sToken == "*") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_Ride;
    } 
    else if (sToken == "/") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_Division;
    } 
    else if (sToken == "=") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_Equal;
    } 
    else if (sToken == "<>") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_Unequal;
    }
    else if (sToken == ">") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_Greater;
    }
    else if (sToken == "<") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_Less;
    }
    else if (sToken == ">=") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_GreaterEqual;
    }
    else if (sToken == "<=") {
        Token.TokenValue.OperatorType = ST_OperatorType::OperatorType_LessEqual;
    }
    return Token;
}

ScriptUint_Token RT_ScriptSystem::AnalysisToken_Variable(std::string sToken)
{
    //先从全局变量寻找，再从局部变量寻找
    ScriptUint_Token Token;
    Token.nRemarks = -2;
    Token.TokenType = ST_TokenType::TokenType_Keyword;
    for (size_t i = 0; i < v_ScriptVariableGlobal.size(); i++)
    {
        if(sToken == v_ScriptVariableGlobal[i].m_VariableName)
        {
            Token.nRemarks = -1;
            Token.TokenValue.KeywordAddr = i;
        }
        for (size_t i = 0; i < v_ScriptVariableBuffer[nPushBufferIndex].size(); i++)
        {
            if(sToken == v_ScriptVariableGlobal[i].m_VariableName)
            {
                Token.nRemarks = nPushBufferIndex;
                Token.TokenValue.KeywordAddr = i;
            }
        }
    }
    if(-2 == Token.nRemarks )
    {
        printf("变量未找到，编译错误\n");
    }
    return Token;
}