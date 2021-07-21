#include "include.h"
extern void Hadrware_Init(void);

int main()
{
    MCU_Init();
    Hadrware_Init();
    
    while(true)
    {
        KernelPolling();
    }
}
