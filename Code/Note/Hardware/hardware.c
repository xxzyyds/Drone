#include "hardware.h"
#include "include.h"
#include "msp.h"
#include "HARDWARE_i2c.h"
#include "HARDWARE_uart.h"
#include "timer_drv.h"
#include "power.h"
#include "beep.h"

void power_managre(void);

/*MCU��ʼ��*/
void MCU_Init(void)
{
    //ֹͣ�ල���Ź���ʱ��
    MAP_WDT_A_holdTimer();
    
    //����ϵͳʱ�ӵ�48M
    SystemCoreClockUpdate();
    
    //�δ��ʱ����ʼ��
    SysTick_Configuration();

    power_init();
    beep_init();
    
    //���PWM��ʼ��
    Motor_Init();
    
    //100ms��ʱ�����ڵȴ�������ʼ������Ҫɾ������䣡
    Delay_ms(100);
    
    //IIC��ʼ��
    I2C_Init();

    //���ڳ�ʼ��
    USART_Init();
    
    //MCU�����ж�
    MAP_Interrupt_enableMaster();  
}

void power_managre()
{
    //���PC6Ϊ��
    //��ôPC7Ϊ��
    
    //�������PC6Ϊ��
    //��ôPC7Ϊ��
}

/*LED��*/
void Hardware_LED_Red_ON(uint8_t luminance)
{
    P2OUT |= GPIO_PIN0;
}


void Hardware_LED_Red_OFF(void)
{
    P2OUT &= ~GPIO_PIN0;
}


void Hardware_LED_Red_TOGGLE(void)
{
    
}

void Hardware_LED_Green_ON(uint8_t luminance)
{
    P1OUT |= GPIO_PIN0;
}

void Hardware_LED_Green_OFF(void)
{
    P1OUT &= ~GPIO_PIN0;
}


void Hardware_LED_Green_TOGGLE(void)
{

}


void Hardware_LED_Blue_ON(uint8_t luminance)
{

}


void Hardware_LED_Blue_OFF(void)
{

}


void Hardware_LED_Blue_TOGGLE(void)
{

}


void Hardware_LED_MOTOR1_ON(void)
{

}

void Hardware_LED_MOTOR1_OFF(void)
{

}

void Hardware_LED_MOTOR1_TOGGLE(void)
{

}

void Hardware_LED_MOTOR2_ON(void)
{

}

void Hardware_LED_MOTOR2_OFF(void)
{

}

void Hardware_LED_MOTOR2_TOGGLE(void)
{

}

void Hardware_LED_MOTOR3_ON(void)
{

}

void Hardware_LED_MOTOR3_OFF(void)
{

}

void Hardware_LED_MOTOR3_TOGGLE(void)
{

}

void Hardware_LED_MOTOR4_ON(void)
{

}

void Hardware_LED_MOTOR4_OFF(void)
{

}

void Hardware_LED_MOTOR4_TOGGLE(void)
{

}

//void Delay_ms(uint16_t ms)
//{
//    uint32_t curTicks;
//    curTicks = msTicks;
//    while ((msTicks - curTicks) < ms);
//}

