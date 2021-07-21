/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：kernel.c
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
循环核心驱动，使用该函数轮训就绪函数


*/
//外部文件引用
#include "include.h"
#include "communication.h"
#include "gcs.h"
#include "speed_estimator.h"
#include "led.h"
#include "battery.h"
#include "stdio.h"
#include "HARDWARE_uart.h"
#include "SPL06.h"
#include "timer_drv.h"
#include "Ano_OF.h"
#include "pos_ctrl.h"
#include "program_ctrl.h"
#include "ANO_GCS_DT.h"
#include "FollowLine.h"
//宏定义区



//Extern引用



//私有函数区
void Update(void);
void UpdateUSBQueue(void);


//私有变量区
bool KernelRunning = false;
bool InitComplete = false;
int16_t Angle_Int16[3];
uint8_t Buff[20];
int sum = 0;

/******************************************************************************
  * 函数名称：KernelPolling
  * 函数描述：核心轮询程序
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：1ms运行一次    
  *    
  *
******************************************************************************/
uint32_t test_time[5];
uint32_t test_runtime[3];
uint8_t data[5] = {0xAA,0xBB,0xCC,0xDD,0xEE};
//uint32_t test_systick[255];
//uint8_t test_cnt;
void KernelPolling()
{
    static uint32_t Cnt = 0;

    if(!InitComplete)
        return ;
    
    if(!KernelRunning)
        return;
        //
        test_time[0] = GetSysTime_us();
        
    KernelRunning = false;
        
        //查询是否有数据需要解析
    ANO_DT_Data_Receive_Anl_Task();
        
    Cnt++;
    if (Cnt % 3 == 0)                    
    {
                //
        test_runtime[0] = GetSysTime_us();
        GetMPU6050Data();
        WZ_Est_Calcu(0.003f);
        test_runtime[1] = GetSysTime_us();
        test_runtime[2] = test_runtime[1] - test_runtime[0];
        //
        ATT_Update(&g_MPUManager,&g_Attitude, 0.003f); 
                //            
        FlightPidControl(0.003f);      
        
        MotorControl();
    }
    
    if (Cnt % 8 == 0)
    {
        GetAngle(&g_Attitude);
    }

    if (Cnt % 11 == 0)
    {    
        AnoOF_State_Task(11);
    }
    
    //巡线任务
    if (Cnt % 10 == 0)
    {
        PollingGCS();
        UpdateCentControl(0.02);
    }
    
    if (Cnt % 20 == 0)
    {
        //
        UpdateSPL06Info();
        //
        WZ_Obs_Calcu(0.02f);
        //
        WZ_Fix_Calcu(0.02f);
        //
        ALT_Ctrl(0.02f);        
        //
        POS_Ctrl(0.02f);
    }
        
    if (Cnt % 50 == 0)
    {            
        //
        One_Key_Take_off_Land_Ctrl_Task(50);
        //
        Program_Ctrl_Task(50); 
    }

    PollingLED();
    PollingUSART();
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
