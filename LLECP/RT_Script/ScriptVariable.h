#include<string>
enum VariableType
{
    en_BOOL,
    en_UINT32,
    en_INT32,
    en_DOUBLE_64,
    en_STRING,
};

class ScriptVariable
{
private:
    void* m_pData;
public:
    VariableType m_VariableType;
    std::string m_VariableName;
    ScriptVariable(VariableType Type);
    ~ScriptVariable();
};
