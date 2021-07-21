#include "hardware.h"
#include "include.h"
#include "msp.h"
#include "HARDWARE_i2c.h"
#include "HARDWARE_uart.h"
#include "timer_drv.h"


void USARTEUSCIA2_Init(void);
void USARTEUSCIA0_Init(void);
/**/
void MCU_Init(void)
{
    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();
    SystemCoreClockUpdate();
    SysTick_Configuration();

    Motor_Init();
    Delay_ms(100);
    I2C_Init(); 
    USART_Init();
    MAP_Interrupt_enableMaster();  
}


//SPIÇø
uint8_t Hardware_SPI1_RW(uint8_t data)
{
    return 0;
}

uint8_t Hardware_SPI1_MEM_Write(uint8_t Reg, uint8_t data)
{
    return 0;
}

uint8_t Hardware_SPI1_MEM_Writes(uint8_t Reg, uint8_t *Buff, uint8_t Length)
{
    return 0;
}

uint8_t Hardware_SPI1_MEM_Read(uint8_t Reg)
{
    return 0;
}

uint8_t Hardware_SPI1_MEM_Reads(uint8_t Reg, uint8_t *Buff, uint8_t Length)
{
    return 0;
}

/*LEDÇø*/
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

