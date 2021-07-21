#include "FollowLine.h"
#include "HARDWARE_uart.h"
#include "stdbool.h"
#include "pid.h"
#include "timer_drv.h"
#include "myMath.h"
#include "gcs.h"
#include "program_ctrl.h"

extern Usart_t UsartGroup[Num_USART];
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
extern UAV_info_t g_UAVinfo;
extern u16 val, spd;
extern _program_ctrl_st program_ctrl;


void proControl(int16_t Distance, int16_t Speed);
void TimeoutCheck(void);
void UpdateStatus(void);
void UpdateAction(float dt);
void UpdateButton(void);
void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction);
bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction);
void UpdateDebugInfo(void);
void HoldCurrentPostion(float dt);

FollowManager_t FollowManager;
SonarManager_t SonarManager;

/*
        |-X
        |
        |
+Y------------- -Y
        |
        |
        |+X
*/


void Follow_Init()
{
    FollowManager.ptrPIDInfoV = &PIDGroup[emPID_FolloLineVertically];
    FollowManager.ptrPIDInfoH = &PIDGroup[emPID_FolloLineHorizontally];
    

    FollowManager.ptrPIDInfoV->kp = 1;
//    FollowManager.ptrPIDInfoV->ki = 0.001;
    

    FollowManager.ptrPIDInfoH->kp = 1;
//    FollowManager.ptrPIDInfoH->ki = 0.001;
    
    PIDGroup[emPID_FolloLineVertically].desired = 60/2;
    PIDGroup[emPID_FolloLineHorizontally].desired = 80/2;
    
    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;
    
    FollowManager.ptrFrame = (OpenMVFrame_t*)UsartGroup[UART_A3].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;
    P1DIR &= ~(1 << BIT1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    
    for(int i = 0;i < 3; i++)
    {
        StandardControl.StandardControlDirction[i].Speed = 0; 
    }
}

void UpdateSpeedDirction()
{
    for(int i = 0;i < 3;i++)
    {
        if(StandardControl.StandardControlDirction[i].Distance > 0)
        {
            StandardControl.StandardControlDirction[i].Speed = ABS(StandardControl.StandardControlDirction[i].Speed);
        }else
        {
            StandardControl.StandardControlDirction[i].Speed = -ABS(StandardControl.StandardControlDirction[i].Speed);
        }
    }
}

//以100hz的速度轮询
void UpdateCentControl(float dt)
{
    UpdateButton();
    
    if(FollowManager.ptrFrame->CentPoint.x1 > 100 || FollowManager.ptrFrame->CentPoint.y1 > 100)
        return;

    UpdateStatus();
    UpdateAction(dt);
}

//此函数只做状态判断和状态更新
void UpdateStatus()
{
    switch(FollowManager.ActionList)
    {
        case ActionWaitting:
            //Do nothing;
            break;
        case ActionCountdown:
            FollowManager.CountDownNumMs--;
        
            if(FollowManager.CountDownNumMs <= 0)
            {
                FollowManager.ActionList = ActionTakeOff;
            }
            break;
        case ActionTakeOff:
            {
                ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionHoldCross);
            }
            break;
        case ActionHoverStartPoint:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoRight);
            break;
        case ActionGoRight:
            {
                static ActionTimes_t ActionTimes = ActionTimes1;
                
                switch(ActionTimes)
                {
                    case ActionTimes1:
                        if(ActionFormChange(MAX_FORMCHANGE_TIME, MirrorFlipLtype, ActionHoldMirrorFlipTurnLtype))
                        {
                            ActionTimes++;
                        }
                        break;
                    case ActionTimes2:
                        if(ActionFormChange(MAX_FORMCHANGE_TIME, MirrorFlipLtype, ActionHoldTurnTtype))
                        {
                            ActionTimes++;
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
        case ActionHoldMirrorFlipTurnLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoForward);
            break;
        case ActionGoForward:
            {
                static ActionTimes_t ActionTimes = ActionTimes1;
                
                switch(ActionTimes)
                {
                    case ActionTimes1:
                        if(ActionFormChange(MAX_FORMCHANGE_TIME, MirrorFlipTurnLtype, ActionGoLeft))
                        {
                            ActionTimes++;
                        }
                        break;
                    case ActionTimes2:
                        if(ActionFormChange(MAX_FORMCHANGE_TIME, MirrorFlipLtype, ActionHoldCross))
                        {
                            ActionTimes++;
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
        case ActionHoldMirrorFlipLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionLand);
            break;
        case ActionGoLeft:
            ActionFormChange(MAX_FORMCHANGE_TIME, TurnLtype, ActionHoldTurnLtype);
            break;
        case ActionHoldTurnLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoBack);
            break;
        case ActionGoBack:
            ActionFormChange(MAX_FORMCHANGE_TIME, Ltype, ActionHoldLtype);
            break;
        case ActionHoldLtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoRight);
            break;
        case ActionHoldTurnTtype:
            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME, ActionGoForward);
            break;
        case ActionHoldCross:
//            ActionHoldPoint(MAX_HOVER_ERR, MAX_HOVER_TIME * 10, ActionLand);
            break;            
        case ActionHoverStopPoint:
            {
                
            }
            break;
        case ActionFollowTarget:
            {
                
            }
            break;
        case ActionLand:
            {
                static int Cnt = MAX_TIMEOUT1;

                if(Cnt-- < 0)
                {
                    FollowManager.ActionList = ActionLock;
                }
            }
            break;
        case ActionLock:
            FollowManager.ActionList = ActionWaitting;
            break;
        case ActionTest:
            {
                static int cnt = 0;
                cnt++;
            }
            break;
        case ActionSonar:
            {

            }
            
            break;
        default:
            break;
    }
}

//只执行动作
void UpdateAction(float dt)
{
    switch(FollowManager.ActionList)
    {
        case ActionWaitting:
            //Do nothing
            break;
        case ActionTakeOff:
            UpdateCMD(0, 0, CmdTakeOff);
            break;
        case ActionHoverStartPoint:
            //起飞
            {
                HoldCurrentPostion(dt);
            }
            break;
        case ActionGoForward:
        case ActionGoBack:
            //前后飞行
            {
                HoldCurrentPostion(dt);
                if(FollowManager.ActionList == ActionGoForward)
                {
                    program_ctrl.vel_cmps_h[X] = -20;
                }
                else if(FollowManager.ActionList == ActionGoBack)
                {
                    program_ctrl.vel_cmps_h[X] = 20;
                }
            }
            break;
        case ActionGoLeft:
        case ActionGoRight:
            {
                HoldCurrentPostion(dt);
                if(FollowManager.ActionList == ActionGoLeft)
                {    
                    program_ctrl.vel_cmps_h[Y] = 20;
                }
                else if(FollowManager.ActionList == ActionGoRight)
                {
                    program_ctrl.vel_cmps_h[Y] = -20;
                }
            }
            break;
            
        //悬停动作
        case ActionHoldLtype:
        case ActionHoldMirrorFlipLtype:
        case ActionHoldTurnLtype:
        case ActionHoldMirrorFlipTurnLtype:
        case ActionHoldCross:
        case ActionHoldTtype:
        case ActionHoldTurnTtype:
        case ActionHoldFeaturePoint:
            if(FollowManager.ptrFrame->FormType == Cross)
            {
                HoldCurrentPostion(dt);
            }else
            {
                program_ctrl.vel_cmps_h[Y] = 0;
                program_ctrl.vel_cmps_h[X] = 0;
            }
            break;
        case ActionHoverStopPoint:
            //控制
            break;
        case ActionLand:
            //降落
            UpdateCMD(0, 0, CmdLand);
            break;
        case ActionLock:
            g_UAVinfo.FMUflg->unlock = 0;
            break;
        case ActionTest:

            break;
        case ActionSonar:
            {
                
            }
            break;
        default:
            break;
    }

    {//此处为Debug信息
        static int cnt = 0;
        cnt++;
        
        if(cnt % 4 == 0)
        {
            UpdateDebugInfo();
        }
    }
}

void HoldCurrentPostion(float dt)
{
    //更新测量点
    PIDGroup[emPID_FolloLineVertically].measured = FollowManager.ptrFrame->CentPoint.y1;
    PIDGroup[emPID_FolloLineHorizontally].measured = FollowManager.ptrFrame->CentPoint.x1;

    UpdatePID(FollowManager.ptrPIDInfoH, dt);  //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt);  //PID
    
    program_ctrl.vel_cmps_h[Y] = FollowManager.ptrPIDInfoH->out;
    program_ctrl.vel_cmps_h[X] = FollowManager.ptrPIDInfoV->out;
}

#include "Ano_OF.h"
extern _ano_of_st ANO_OF;
extern HeightInfo_t HeightInfo;
void UpdateDebugInfo()
{
    UpdateToGCSLine2(PIDGroup[emPID_FolloLineVertically].measured,PIDGroup[emPID_FolloLineHorizontally].measured,PIDGroup[emPID_FolloLineHorizontally].desired,PIDGroup[emPID_FolloLineVertically].desired, 0,0,0,0);
}

bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction)
{
    static int cnt = 0;
    bool ChangeFinished = false;
            
    if(FollowManager.ptrFrame->FormType == TargetFormType)
    {
        cnt++;
        
        if(cnt > HoldTime)
        {
            cnt = 0;
            FollowManager.ActionList = NextAction;
            ChangeFinished = true;
        }
    }else
    {
        cnt = 0;
    }
    
    return ChangeFinished;
}

void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction)
{
    static uint16_t CountDown = MAX_HOVER_TIME;
    
    CountDown--;

    if(CountDown == 0)
    {
        CountDown = MAX_HOVER_TIME;
        FollowManager.ActionList = NextAction;
    }
}

//规定：参数Distance,Speed中，左为正方向，右为负方向
void proControl(int16_t Distance, int16_t Speed)
{   
    Distance = LIMIT(Distance, -8, 8);
    Speed = LIMIT(Speed, -10, 10);

    if(Distance > 0 && Speed > 0)
    {
        UpdateCMD(Distance, Speed, CmdLeft);
    }else if(Distance < 0 && Speed < 0)
    {
        UpdateCMD(Distance, Speed, CmdRight);
    }
}



//超时限制
void TimeoutCheck()
{
    FSMList_t tmpAction;
    static int Cnt = 0;
    
    if(tmpAction == FollowManager.ActionList)
    {
        Cnt++;
        
        if(Cnt > MAX_TIMEOUT2)
        {
            tmpAction = ActionEmergencyLand;
        }else if(Cnt > MAX_TIMEOUT1)
        {
            tmpAction = ActionLand;
        }
    }else
    {
        Cnt = 0;
        tmpAction = FollowManager.ActionList;
    }
}

void UpdateButton()
{
    volatile static uint8_t input = 0;
    input = P1IN & BIT1;
    
    if(input)
    {
        
    }else
    {
        FollowManager.ActionList = ActionCountdown;
        Delay_ms(300);
    }
}

