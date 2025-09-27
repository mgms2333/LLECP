#define AnaInSlavePos 0, 0
#define ZeroErr 0x00009434, 0x0000001b
#define frequency 2000
#define MY_STACK_SIZE 8192
 
#include <time.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>


#include"RT_Script.h"
int main() 
{

    
    RT_ScriptSystem *p = new RT_ScriptSystem();
    p->InitRT_ScriptSystem();
    p->PushScriptCmd("X:DINT;");
    p->PushScriptCmd("X13r:DINT;");
    std::string cmd = "X13r:=X-(10*(10+10));";
    //cmd = "bTest:=Real;";
    //cmd = "IF X>=1 THEN";
    p->PushScriptCmd("X:=10;");
    p->PushScriptCmd("X13r:=5;");
    p->PushScriptCmd(cmd);
    p->StartRT_Script();
    while(true)
    {
        p->RT_ScriptTick();
    }
    printf("Hello World!!!\n");
    delete p;
    p = nullptr;
    return 0;
}
