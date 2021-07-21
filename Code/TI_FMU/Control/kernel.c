/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�kernel.c
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
ѭ������������ʹ�øú�����ѵ��������


*/
//�ⲿ�ļ�����
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
//�궨����



//Extern����



//˽�к�����
void Update(void);
void UpdateUSBQueue(void);


//˽�б�����
bool KernelRunning = false;
bool InitComplete = false;
int16_t Angle_Int16[3];
uint8_t Buff[20];
int sum = 0;

/******************************************************************************
  * �������ƣ�KernelPolling
  * ����������������ѯ����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��1ms����һ��    
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
        
        //��ѯ�Ƿ���������Ҫ����
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
    
    //Ѳ������
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
