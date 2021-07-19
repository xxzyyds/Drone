/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�UART.c
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
�������ڵ��������

*/
//�ⲿ�ļ�����
#include "HARDWARE_uart.h"
#include "msp.h"
#include "pid.h"
#include "include.h"
#include "Remote.h"
#include "ZKHD_Link.h"
#include "myMath.h"
#include "program_ctrl.h"
#include "FollowLine.h"

#define USART_RX_TIMEOUT_MAX 5

bool lbUsartRx = false;
int TimeoutUSART = 0;
uint8_t SBusRxBuff[30] = {0};

Usart_t UsartGroup[Num_USART];

void UART_A0_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_A1_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_A2_ReceiveHandle(uint8_t *ptr, uint8_t length);
void UART_A3_ReceiveHandle(uint8_t *ptr, uint8_t length);

/******************************************************************************
  * �������ƣ�USART_Init
  * �������������ڳ�ʼ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��TI��˾ͨ��������ַ�ṩ���û�������������Ϣ
  * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
  * A2����S.BUS���ߣ�������Ϊ100K    
  * A0����ANO������������Ϊ115200
  * A3����OpenMV��������115200
  * A1����������������115200
******************************************************************************/

void *A1ReceiveHandle;
void USART_Init()
{
    eUSCI_UART_Config uartConfigA2 =
        {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,               // SMCLK Clock Source
            30,                                           // BRDIV = 78
            0,                                            // UCxBRF = 2
            0,                                            // UCxBRS = 0
            EUSCI_A_UART_EVEN_PARITY,                     // No Parity-
            EUSCI_A_UART_LSB_FIRST,                       // LSB First
            EUSCI_A_UART_TWO_STOP_BITS,                   // One stop bit
            EUSCI_A_UART_MODE,                            // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
        };

    eUSCI_UART_Config uartConfigA0 =
        {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,               // SMCLK Clock Source
            26,                                           // BRDIV = 78
            0,                                            // UCxBRF = 2
            111,                                          // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                       // No Parity-
            EUSCI_A_UART_LSB_FIRST,                       // LSB First
            EUSCI_A_UART_ONE_STOP_BIT,                    // One stop bit
            EUSCI_A_UART_MODE,                            // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
        };

    eUSCI_UART_Config uartConfigA1 =
        {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,               // SMCLK Clock Source
            26,                                           // BRDIV = 78
            0,                                            // UCxBRF = 2
            111,                                          // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                       // No Parity-
            EUSCI_A_UART_LSB_FIRST,                       // LSB First
            EUSCI_A_UART_ONE_STOP_BIT,                    // One stop bit
            EUSCI_A_UART_MODE,                            // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
        };

    //������������λ��  baud:115200
    eUSCI_UART_Config uartConfigA3 =
        {
            EUSCI_A_UART_CLOCKSOURCE_SMCLK,               // SMCLK Clock Source
            26,                                           // BRDIV = 78
            0,                                            // UCxBRF = 2
            111,                                          // UCxBRS = 0
            EUSCI_A_UART_NO_PARITY,                       // No Parity-
            EUSCI_A_UART_LSB_FIRST,                       // LSB First
            EUSCI_A_UART_ONE_STOP_BIT,                    // One stop bit
            EUSCI_A_UART_MODE,                            // UART mode
            EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION // Oversampling
        };

    //
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,
                                                   GPIO_PIN3 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    MAP_UART_initModule(EUSCI_A2_BASE, &uartConfigA2);
    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A2_BASE);
    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A2_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

    //
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                                   GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A0_BASE, &uartConfigA0);
    /* Enable UART module */
    MAP_UART_enableModule(EUSCI_A0_BASE);
    /* Enabling interrupts */
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

    //��ʼ��A3�������š���ʼ��ģ�顢ʹ��ģ�顢ʹ���ж�
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P9,
                                                   GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A3_BASE, &uartConfigA3);
    MAP_UART_enableModule(EUSCI_A3_BASE);
    MAP_UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A3_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

    //��ʼ��A1�������š���ʼ��ģ�顢ʹ��ģ�顢ʹ���ж�
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
                                                   GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_UART_initModule(EUSCI_A1_BASE, &uartConfigA1);
    MAP_UART_enableModule(EUSCI_A1_BASE);
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT);

    //��ʼ�����ں���ʹ���ж�
    MAP_Interrupt_enableInterrupt(INT_EUSCIA0);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA2);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA3);

    UsartGroup[UART_A0].RxHandle = UART_A0_ReceiveHandle;
    UsartGroup[UART_A1].RxHandle = UART_A1_ReceiveHandle;
    UsartGroup[UART_A2].RxHandle = UART_A2_ReceiveHandle;
    UsartGroup[UART_A3].RxHandle = UART_A3_ReceiveHandle;
    A1ReceiveHandle = UART_A1_ReceiveHandle;

    UsartGroup[UART_A0].TxCommpleate = true;
    UsartGroup[UART_A1].TxCommpleate = true;
    UsartGroup[UART_A2].TxCommpleate = true;
    UsartGroup[UART_A3].TxCommpleate = true;

    UsartGroup[UART_A0].moduleInstance = EUSCI_A0_BASE;
    UsartGroup[UART_A1].moduleInstance = EUSCI_A1_BASE;
    UsartGroup[UART_A2].moduleInstance = EUSCI_A2_BASE;
    UsartGroup[UART_A3].moduleInstance = EUSCI_A3_BASE;
}

/******************************************************************************
  * �������ƣ�PollingUSART
  * ������������ѯ���ڽ�����Ϣ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע������ϵͳ����λ��ͨ�ţ��˴�Ϊ���
  *
  *
******************************************************************************/
void PollingUSART()
{
    for (int i = 0; i < Num_USART; i++)
    {
        if (UsartGroup[i].RxStart)
        {
            UsartGroup[i].RxTimeout++;

            if (UsartGroup[i].RxTimeout > USART_RX_TIMEOUT_MAX)
            {
                //UsartGroup[i].RxCnt = UsartGroup[i].RxCnt > MAX_RECEIVE_CNT ? MAX_RECEIVE_CNT : UsartGroup[i].RxCnt;
                if (UsartGroup[i].RxCnt > MAX_RECEIVE_CNT)
                {
                    UsartGroup[i].RxCnt = MAX_RECEIVE_CNT;
                }

                UsartGroup[i].RxHandle(UsartGroup[i].RxBuff, UsartGroup[i].RxCnt);
                UsartGroup[i].RxTimeout = 0;
                UsartGroup[i].RxStart = false;
                UsartGroup[i].RxCnt = 0;
            }
        }

        if (UsartGroup[i].TxCommpleate)
        {
            if (deQueue(&UsartGroup[i].qTx, UsartGroup[i].TxBuff, &UsartGroup[i].TxLength))
            {
                UsartGroup[i].TxCommpleate = false;

                UART_transmitData(UsartGroup[i].moduleInstance, UsartGroup[i].TxBuff[0]);
            }
        }
    }
}

/******************************************************************************
  * �������ƣ�USART_TX
  * �������������жϵķ�ʽ��������
  * ��    �룺
  * Usart_t usart:ѡ��Ҫ�������ݵ�ģ��
  * uint8_t *ptx:Ҫ���͵����ݵ�ַ
  * uint8_t len:Ҫ���͵����ݳ���
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void USART_TX(Usart_t *usart, uint8_t *pTx, uint8_t len)
{
    for (int i = 0; i < Num_USART; i++)
    {
        if (usart == &UsartGroup[i])
        {
            enQueue(&UsartGroup[i].qTx, pTx, len);
            break;
        }
    }
}

/******************************************************************************
  * �������ƣ�EUSCIA2_IRQHandler
  * ����������EUSCIA2�Ϸ�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��SBUS��������
  *
  *
******************************************************************************/
void EUSCIA2_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A2_BASE);

    UART_clearInterruptFlag(EUSCI_A2_BASE, status);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        UsartGroup[UART_A2].RxBuff[UsartGroup[UART_A2].RxCnt] = MAP_UART_receiveData(EUSCI_A2_BASE);
        UsartGroup[UART_A2].RxCnt++;
        UsartGroup[UART_A2].RxStart = true;
        UsartGroup[UART_A2].RxTimeout = 0;

#ifdef SBUS
        if (UsartGroup[UART_A2].RxBuff[0] != 0x0F)
        {
            UsartGroup[UART_A2].RxCnt = 0;
        }

        if (UsartGroup[UART_A2].RxCnt == 25)
        {
            UART_A2_ReceiveHandle(UsartGroup[UART_A2].RxBuff, UsartGroup[UART_A2].RxCnt);
            UsartGroup[UART_A2].RxCnt = 0;
        }
        else if (UsartGroup[UART_A2].RxCnt > 25)
        {
            UsartGroup[UART_A2].RxCnt = 0;
        }

#endif
    }
}

typedef enum
{
    PosP_In1 = 0,
    PosP_In01,
    PosP_De1,
    PosP_De01,

    SpdP_In1,
    SpdP_In01,
    SpdP_De1,
    SpdP_De01,
    SpdD_In1,
    SpdD_In01,
    SpdD_De1,
    SpdD_De01,
} Noll;

#include "Ano_OF.h"
#include "ANO_GCS_DT.h"
//uint8_t test_buf[256];
//uint8_t test_buf_p;
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    UART_clearInterruptFlag(EUSCI_A0_BASE, status);
    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        //      test_buf[test_buf_p++] =
        uint8_t tmp_data = MAP_UART_receiveData(EUSCI_A0_BASE);
        UsartGroup[UART_A0].RxBuff[UsartGroup[UART_A0].RxCnt] = MAP_UART_receiveData(EUSCI_A0_BASE);
        UsartGroup[UART_A0].RxCnt++;
        UsartGroup[UART_A0].RxStart = true;
        UsartGroup[UART_A0].RxTimeout = 0;
        AnoOF_GetOneByte(tmp_data);
    }

    if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        if (!UsartGroup[UART_A0].TxCommpleate)
        {
            UsartGroup[UART_A0].TxCnt++;

            if (UsartGroup[UART_A0].TxCnt >= UsartGroup[UART_A0].TxLength)
            {
                UsartGroup[UART_A0].TxCommpleate = true;
                UsartGroup[UART_A0].TxCnt = 0;
            }
            else
            {
                UART_transmitData(UsartGroup[UART_A0].moduleInstance,
                                  UsartGroup[UART_A0].TxBuff[UsartGroup[UART_A0].TxCnt]);
            }
        }
    }
}

void EUSCIA1_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);
    UART_clearInterruptFlag(EUSCI_A1_BASE, status);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t temp = MAP_UART_receiveData(EUSCI_A1_BASE);

        UsartGroup[UART_A1].RxBuff[UsartGroup[UART_A1].RxCnt] = MAP_UART_receiveData(EUSCI_A1_BASE);
        UsartGroup[UART_A1].RxCnt++;
        UsartGroup[UART_A1].RxStart = true;
        UsartGroup[UART_A1].RxTimeout = 0;
    }

    if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        if (!UsartGroup[UART_A1].TxCommpleate)
        {
            UsartGroup[UART_A1].TxCnt++;

            if (UsartGroup[UART_A1].TxCnt >= UsartGroup[UART_A1].TxLength)
            {
                UsartGroup[UART_A1].TxCommpleate = true;
                UsartGroup[UART_A1].TxCnt = 0;
            }
            else
            {
                UART_transmitData(UsartGroup[UART_A1].moduleInstance,
                                  UsartGroup[UART_A1].TxBuff[UsartGroup[UART_A1].TxCnt]);
            }
        }
    }
}

void EUSCIA3_IRQHandler(void)
{
    uint32_t status = MAP_UART_getEnabledInterruptStatus(EUSCI_A3_BASE);
    UART_clearInterruptFlag(EUSCI_A3_BASE, status);

    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        uint8_t temp = MAP_UART_receiveData(EUSCI_A3_BASE);
        ANO_DT_Data_Receive_Prepare(temp);

        UsartGroup[UART_A3].RxBuff[UsartGroup[UART_A3].RxCnt] = MAP_UART_receiveData(EUSCI_A3_BASE);
        UsartGroup[UART_A3].RxCnt++;
        UsartGroup[UART_A3].RxStart = true;
        UsartGroup[UART_A3].RxTimeout = 0;
    }

    if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
    {
        if (!UsartGroup[UART_A3].TxCommpleate)
        {
            UsartGroup[UART_A3].TxCnt++;

            if (UsartGroup[UART_A3].TxCnt >= UsartGroup[UART_A3].TxLength)
            {
                UsartGroup[UART_A3].TxCommpleate = true;
                UsartGroup[UART_A3].TxCnt = 0;
            }
            else
            {
                UART_transmitData(UsartGroup[UART_A3].moduleInstance,
                                  UsartGroup[UART_A3].TxBuff[UsartGroup[UART_A3].TxCnt]);
            }
        }
    }
}

//* A0����ANO������������Ϊ115200
#include "ANO_DT.h"
void UART_A0_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
    ANO_DT_Data_Receive_Anl(ptr, length);
}

//  * A2����S.BUS���ߣ�������Ϊ100K   
void UART_A2_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
    memcpy(SBusRxBuff, ptr, length);
    AnalyRC();
}

extern u16 val, spd;
extern _program_ctrl_st program_ctrl;
extern FollowManager_t FollowManager;
extern SonarManager_t SonarManager;
void UART_A3_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
    for(int i=0;i<length;i++)
	{
		if(*ptr==0xAA && *(ptr+1)==0xAA)
			FollowManager.ptrFrame = (OpenMVFrame_t *)(ptr+i);
	}
}

//  * A1���ӳ�ǰOpenMV��������115200
void UART_A1_ReceiveHandle(uint8_t *ptr, uint8_t length)
{
    //ano gcs
}
/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
