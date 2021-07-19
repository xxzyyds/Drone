#include "include.h"
extern void Hadrware_Init(void);

int main()
{
    //MCU��ʼ��
    MCU_Init();
    
    //���Ӳ����ʼ��
    Hadrware_Init();
    
    while(true)
    {
        //������ѵ 
        KernelPolling();
    }
}
