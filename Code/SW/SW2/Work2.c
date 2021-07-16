
#include "FollowLine.h"
//#include "HARDWARE_uart.h"
//#include "stdbool.h"
//#include "pid.h"
//#include "timer_drv.h"
//#include "myMath.h"
//#include "gcs.h"
////#include "program_ctrl.h"
//#include "sdk.h"

//extern Usart_t UsartGroup[Num_USART];
//extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
//extern UAV_info_t g_UAVinfo;
#include"FollowLine_eg3.h"

bool FollowLine = false;
bool FollowApriTag = false;

//void proControl(int16_t Distance, int16_t Speed);
//void TimeoutCheck(void);
//void UpdateStatus(void);
//void UpdateAction(float dt);
//void UpdateButton(void);
//void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction);
//bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction);
//void UpdateDebugInfo(void);
//void HoldCurrentPostion(float dt);

FollowManager_t FollowManager;
SonarManager_t SonarManager;



/*
        |+X ???????
        |
        |
+Y------------- -Y
        |
        |
        |-X
        
*/

void Follow_Init()
{
    FollowManager.ptrPIDInfoV = &PIDGroup[emPID_FolloLinePosVertically];
    FollowManager.ptrPIDInfoH = &PIDGroup[emPID_FolloLinePosHorizontally];

    FollowManager.ptrPIDInfoV->kp = 1.5f;
    FollowManager.ptrPIDInfoH->kp = 1.5f;

    FollowManager.ptrPIDInfoH->DeathArea = 3;
    FollowManager.ptrPIDInfoV->DeathArea = 3;

    PIDGroup[emPID_FolloLineSpdVertically].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdVertically].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdVertically].kd = 0.014f;

    PIDGroup[emPID_FolloLineSpdHorizontally].kp = 0.45f;
    PIDGroup[emPID_FolloLineSpdHorizontally].ki = 0.13f;
    PIDGroup[emPID_FolloLineSpdHorizontally].kd = 0.014f;

    PIDGroup[emPID_FolloLinePosVertically].desired = 120 / 2;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = 160 / 2;

    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;

    FollowManager.ptrFrame = (OpenMVFrame_t *)UsartGroup[UART_A3].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;

    P1DIR &= ~(1 << BIT1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    P1DIR &= ~(1 << BIT4);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
}

//??100hz???????? 10ms
void UpdateCentControl(float dt)
{
    //?§Ø?OpenMV?????????????????§Ö????OpenMV??????§¹????
    if (FollowManager.ptrFrame->CentPoint.x1 > 200 || FollowManager.ptrFrame->CentPoint.y1 > 200)
        return;

    //???¡ã?????????
    UpdateButton();    

    //??????????
    UpdateStatus();

    //???????????
    UpdateAction(dt);
}

static int Num = 0;
static int Status = 0;
static int NumStop = 0;

//???????????§Ø????????
void UpdateStatus()
{
    
    //????ActionList????????????????
    switch (FollowManager.ActionList)
    {
        //?§Ø?
        case ActionWaitting:
            //Do nothing;
            break;

        //???????
        case ActionCountdown:
        {
            //????????????????¦Ë??Follow_Init??
            FollowManager.CountDownNumMs--;

            //????????????????????ActionTakeOff
            if (FollowManager.CountDownNumMs <= 0)
            {
                FollowManager.ActionList = ActionTakeOff;
            }
        }
        break;

        //????????
        case ActionTakeOff:
        {
            //?????????????????5s??500 * 10ms = 5000ms = 5s???????????ActionHoverStartPoint??????
            ActionHoldPoint(MAX_HOVER_ERR, 500, ActionHoverStartPoint);
        }
        break;

        //?????????5S
        case ActionHoverStartPoint:
            ActionHoldPoint(MAX_HOVER_ERR, 500, ActionGoForward);
            break;
        
        //???
        case ActionGoForward:
            ActionHoldPoint(MAX_HOVER_ERR, 500, ActionLand);
            break;
        
        //????????????????????????????????
        case ActionLand:
        {
            //????????
            static int Cnt = MAX_TIMEOUT1;

            if (Cnt-- < 0)
            {
                FollowManager.ActionList = ActionLock;
            }
        }
        break;

        //????????
        case ActionLock:
            FollowManager.ActionList = ActionWaitting;
            break;
        default:
            break;
    }
}

//???§Ø???
void UpdateAction(float dt)
{
    switch (FollowManager.ActionList)
    {

    //?????????
    case ActionWaitting:
        //Do nothing
        break;

    //??????????
    case ActionTakeOff:
        sdk_takeoff(50);    //???????1.2m????
        break;

    //???????
    case ActionHoverStartPoint:
        sdk_velocity_reset();
        break;

    //???
    case ActionGoForward:
        {
            if(FollowManager.ptrFrame->CentPoint.y1 != 100 && FollowManager.ptrFrame->CentPoint.x1 != 100 && FollowManager.ptrFrame->FormType != ApriTag)
            {
                sdk_velociyt_y_set(-20);
            }
            //????????????100?? ???
            if(FollowManager.ptrFrame->CentPoint.y1 == 100)
            {
                sdk_land();
            }
            else
            {
                //?????????100?? ?
                if (FollowManager.ptrFrame->CentPoint.x1 == 100 || NumStop == 1)
                {
                    if (Num <= 100)
                    {
                        sdk_velocity_reset();
                        Num += 1;
                                            Status = 1;
                    }
                    if (Num > 100 && Num <= 200)
                    {
                        sdk_velociyt_y_set(-20);
                        Num += 1;
                    }
                    if (Num > 200 && Num <= 300)
                    {
                        sdk_yaw_set(180);
                        Num += 1;
                    }
                    if(Num > 300 && Num <= 400)
                    {
                        sdk_velociyt_x_set(-20);
                        Num += 1;
                    }
                    if (Num > 400 && Num <= 500)
                    {
                        sdk_velociyt_y_set(-20);
                        Num += 1;
                    }
                }
                else
                {
                    //??????????
                    if (FollowManager.ptrFrame->FormType == ApriTag)
                    {
                        if( NumStop <= 100)
                        {
                            sdk_velocity_reset();
                            NumStop +=1;
                        }
                    }
                }
            }
        }
        break;
    //???????
    case ActionLand:
        //????????
        sdk_land();
        break;

    //????????
    case ActionLock:
        g_UAVinfo.FMUflg->unlock = 0;
        break;
    default:
        break;
    }
}

void HoldCurrentPostion(float dt)
{
    static float OldPos[2];
    //????????LPF
    
    //?????????
    PIDGroup[emPID_FolloLinePosVertically].measured = FollowManager.ptrFrame->CentPoint.y1;
    PIDGroup[emPID_FolloLinePosHorizontally].measured = FollowManager.ptrFrame->CentPoint.x1;

    PIDGroup[emPID_FolloLineSpdVertically].measured = (FollowManager.ptrFrame->CentPoint.y1 - OldPos[0]);
    PIDGroup[emPID_FolloLineSpdHorizontally].measured = (FollowManager.ptrFrame->CentPoint.x1 - OldPos[1]);
    
    OldPos[0] = FollowManager.ptrFrame->CentPoint.y1;
    OldPos[1] = FollowManager.ptrFrame->CentPoint.x1;
    
    UpdatePID(FollowManager.ptrPIDInfoH, dt);  //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt);  //PID
    
    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;
    
    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt);  //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);  //PID

    //???????????????
    sdk_velocity_set(PIDGroup[emPID_FolloLineSpdVertically].out, PIDGroup[emPID_FolloLineSpdHorizontally].out);
}

bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction)
{
    static int cnt = 0;
    bool ChangeFinished = false;

    if (FollowManager.ptrFrame->FormType == TargetFormType)
    {
        cnt++;

        if (cnt > HoldTime)
        {
            cnt = 0;
            FollowManager.ActionList = NextAction;
            ChangeFinished = true;
        }
    }
    else
    {
        cnt = 0;
    }

    return ChangeFinished;
}

void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction)
{
    static bool Enter = true;
    static uint16_t CountDown = 0;

    if (Enter)
    {
        CountDown = HoldTime;
        Enter = false;
    }
    else
    {
        CountDown--;
    }

    if (CountDown == 0)
    {
        Enter = true;
        FollowManager.ActionList = NextAction;
    }
}

void UpdateButton()
{
    //?§Ø??????????????§¹????????§Ø?????????????
    volatile static uint8_t input = 0;
    volatile static uint8_t input2 = 0;
    input = P1IN & BIT1;
    input2 = P1IN & BIT4;

    //?§Ø???????????
    if (input)
    {
    }
    else
    {
        FollowLine = true;
    }

    //?§Ø????ApriTag????????
    if (input2)
    {
    }
    else
    {
        FollowApriTag = true;
    }

    //?§Ø???????
    if (FollowApriTag == false && FollowLine == false)
    {
        return;
    }
    else
    {
        static bool CloseGate = true;

        //??????????????
        if (CloseGate)
        {
            CloseGate = false;
            FollowManager.ActionList = ActionCountdown;
        }
    }
}
