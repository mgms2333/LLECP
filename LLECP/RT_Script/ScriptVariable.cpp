#include"ScriptVariable.h"
ScriptVariable::ScriptVariable(VariableType Type)
{
    m_VariableType = Type;
    switch (m_VariableType)
    {
    case en_BOOL:
        m_pData = new bool;
        break;
    case en_UINT32:
        m_pData = new unsigned int;
        break;
    case en_INT32:
        m_pData = new int;
        break;
    case en_DOUBLE_64:
        m_pData = new double;
        break;
    case en_STRING:
        m_pData = new std::string;
        break;
    default:
        break;
    }
}

ScriptVariable::~ScriptVariable()
{
    switch (m_VariableType)
    {
    case en_BOOL:
        delete static_cast<bool*>(m_pData);
        break;
    case en_UINT32:
        delete static_cast<unsigned int*>(m_pData);
        break;
    case en_INT32:
        delete static_cast<int*>(m_pData);
        break;
    case en_DOUBLE_64:
        delete static_cast<double*>(m_pData);
        break;
    case en_STRING:
        delete static_cast<std::string*>(m_pData);
        break;
    default:
        break;
    }
    m_pData = nullptr;
}
