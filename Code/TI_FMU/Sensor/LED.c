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
    LED_STATUS_OFF;
    LED_POWER_OFF;
    g_LedManager.emLEDStatus = StatusOff;
    g_LedManager.emLEDPower = PowerOff;    
    P1DIR |= GPIO_PIN0;
    
    P2DIR |= GPIO_PIN0;
    P2DIR |= GPIO_PIN1;
    P2DIR |= GPIO_PIN2;
    
    P1OUT &= ~GPIO_PIN0;
    P2OUT &= ~GPIO_PIN0;
//    P2DIR &= ~GPIO_PIN1;
    P2DIR &= ~GPIO_PIN2;
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

    switch(g_LedManager.emLEDStatus)
    {
        case StatusFlash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                LED_STATUS_TOGGLE;
            }
            break;
        case StatusToggle:
            LED_STATUS_TOGGLE;
            break;
        case StatusOn:
            LED_STATUS_ON;
            break;
        case StatusOff:
            LED_STATUS_OFF;
            break;
        default:
            break;
    }
    
    switch(g_LedManager.emLEDPower)
    {
        case PowerOn:
            LED_POWER_ON;
            break;
        case PowerFlash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                LED_POWER_TOGGLE;
            }
            break;
        case PowerOff:
            LED_POWER_OFF;
            break;
        case PowerToggle:
            LED_POWER_TOGGLE;
            break;
        default:
            break;
    }
    
    switch(g_LedManager.emLEDMotor)
    {
        case MotorFlash:
            if(g_LedManager.u16FlashTime % LED_FLASH_FREQ == 0)
            {
                M1_Toggle;
                M2_Toggle;
                M3_Toggle;
                M4_Toggle;
            }
            break;
        case MotorClockwiseFlash:
            
            break;
        case MotorOn:
            M1_ON;
            M2_ON;
            M3_ON;
            M4_ON;
            break;
        case MotorOff:
            M1_OFF;
            M2_OFF;
            M3_OFF;
            M4_OFF;
            break;
        default:
            break;
    }
    
    LEDEventHandle();
}

#include "FollowLine.h"
extern FollowManager_t FollowManager;
void LEDEventHandle()
{
    if(!g_FMUflg.unlock)
    {
        g_LedManager.emLEDPower = PowerOff;
        g_LedManager.emLEDStatus = StatusOn;
    }else
    {
        g_LedManager.emLEDStatus = StatusOff;
        g_LedManager.emLEDPower = PowerOn; 
    }
    
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
                P2OUT |= GPIO_PIN1;
            }else
            {
                P2OUT &= ~GPIO_PIN1;
            }
        }
    }else
    {
        P2OUT &= ~GPIO_PIN1;
    }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
