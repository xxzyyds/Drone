#include "include.h"
#include "FollowLine_eg3.h"
#include "FollowLine.h"
extern void Hadrware_Init(void);
//extern bool FollowLine = false;
//extern bool FollowApriTag = false;
//extern FollowManager_t FollowManager;
//extern SonarManager_t SonarManager;
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
