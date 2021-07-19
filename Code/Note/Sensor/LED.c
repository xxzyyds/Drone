/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�LED.c
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
LED������ʹ�÷�ʽ���£�
g_LedManagerΪLED�ƿ��ƽṹ�壬Ҫ����LED����˸��ֻ��Ҫ���Ĵ˽ṹ���е�
ö��ֵ���ɡ���g_LedManager.emLed_Status
���磺
1.��Ҫ״̬��������Ҫ�������
g_LedManager.emLed_Status = StatusOn;

����ö����Դ����.h�ļ��е�emLED_Status_t

*/
//�ⲿ�ļ�����
#include "LED.h"
#include "control.h"
#include "Hardware.h"
#include "battery.h"
#include "timer_drv.h"

//�궨����
#define LED_FLASH_FREQ      100


//Extern����
extern FMUflg_t g_FMUflg;    


//˽�к�����
void LEDEventHandle(void);


//˽�б�����
LedManager_t g_LedManager;


/******************************************************************************
  * �������ƣ�LEDInit
  * ������������ʼ��LED��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void LEDInit(void)        
{
    P1DIR |= GPIO_PIN0;
    
    //����GPIO����
    P2DIR |= GPIO_PIN0;
    P2DIR |= GPIO_PIN1;
    P2DIR |= GPIO_PIN2;
    
    //����GPIO���
    P1OUT &= ~GPIO_PIN0;
    P2OUT &= ~GPIO_PIN0;
    P2OUT |= GPIO_PIN2;
}

/******************************************************************************
  * �������ƣ�PollingLED
  * ������������ѯ��ǰ�Ƿ���LED�����������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void PollingLED()
{
    g_LedManager.u16FlashTime++;
    
    //����LED�ƹ�����״̬����LED��    
    LEDEventHandle();
}

#include "FollowLine.h"
extern FollowManager_t FollowManager;

//LED�ƴ�������
void LEDEventHandle()
{   
    //������̿ص���ʱʱ����ʼ��˸
    if(FollowManager.ActionList == ActionCountdown)
    {
        static int cnt = 0;
        cnt++;
        
        if(cnt % 50 == 0)
        {
            static bool tmp = false;
            tmp = !tmp;
            if(tmp)
            {
                //LED������
                P2OUT |= GPIO_PIN1;
            }else
            {
                //LED������
                P2OUT &= ~GPIO_PIN1;
            }
        }
    }else
    {
        P2OUT &= ~GPIO_PIN1;
    }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
