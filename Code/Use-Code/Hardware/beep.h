#ifndef _BEEP_H
#define _BEEP_H

#include "stdint.h"

#define BEEP_ON P3OUT |= GPIO_PIN0
#define BEEP_OFF P3OUT &= ~GPIO_PIN0
#define BEEP_TOGGLE GPIO_toggleOutputOnPin(GPIO_PORT_P3,GPIO_PIN0)

typedef enum
{
    beep_on = 0,                        //���������
    beep_off,                       //�����Ϩ�� 
    beep_flash,                     //�������˸
    beep_clockwiseFlash,            //null
}embeep_t;

typedef struct
{
    uint16_t u16FlashTime;          //��˸��ʱ,1ms�ۼ�һ��
    uint16_t beep_flash_freq;
    embeep_t   beep;
}BeepManager_t;

extern BeepManager_t BeepManager;

void beep_init(void);
void beep_update(void);


#endif
