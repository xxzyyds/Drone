/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：LED.c
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
LED灯驱动使用方式如下：
g_LedManager为LED灯控制结构体，要控制LED的闪烁，只需要更改此结构体中的
枚举值即可——g_LedManager.emLed_Status
例如：
1.想要状态灯亮起，需要以下语句
g_LedManager.emLed_Status = StatusOn;

其他枚举量源自于.h文件中的emLED_Status_t

*/
//外部文件引用
#include "LED.h"
#include "control.h"
#include "Hardware.h"
#include "battery.h"

//宏定义区
#define LED_FLASH_FREQ      100


//Extern引用
extern FMUflg_t g_FMUflg;    


//私有函数区
void LEDEventHandle(void);


//私有变量区
LedManager_t g_LedManager;


/******************************************************************************
  * 函数名称：LEDInit
  * 函数描述：初始化LED灯
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
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
  * 函数名称：PollingLED
  * 函数描述：轮询当前是否有LED亮起任务就绪
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null
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

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
