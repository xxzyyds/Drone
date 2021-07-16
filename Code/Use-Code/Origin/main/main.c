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
    //MCU初始化
    MCU_Init();
    
    //相关硬件初始化
    Hadrware_Init();
    
	  while(true)
    {
      //核心轮训 
      KernelPolling();
    }
}
