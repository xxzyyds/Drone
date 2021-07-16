/*
���������2020�����������Ҫ����

����û���Ϊ�Ѿ����ո��ļ�ʹ�÷�������ɾ�����ļ���Ȼ�����FollowLine.c�ļ�

1.�õ�����ɫ�����ݣ�
2.
*/

#include "FollowLine.h"
#include "HARDWARE_uart.h"
#include "stdbool.h"
#include "pid.h"
#include "timer_drv.h"
#include "myMath.h"
#include "gcs.h"
#include "sdk.h"
#include "stdbool.h"
//#include "program_ctrl.h"

extern Usart_t UsartGroup[Num_USART];
extern PIDInfo_t PIDGroup[emNum_Of_PID_List];
extern UAV_info_t g_UAVinfo;
bool FollowLine = false;
bool FollowApriTag = false;

bool is_set_round(float first_angle, float current_angle);
void proControl(int16_t Distance, int16_t Speed);
void TimeoutCheck(void);
void UpdateStatus(void);
void UpdateAction(float dt);
void UpdateButton(void);
void ActionHoldPoint(int8_t Err, int16_t HoldTime, FSMList_t NextAction);
bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction);
void UpdateDebugInfo(void);
void HoldCurrentPostion(float dt);
void HoldYawAngle(float dt);
float update_yaw_info_in_360(void);
FollowManager_t FollowManager;
SonarManager_t SonarManager;

/*
        |+X
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
    FollowManager.ptrPIDInfoY = &PIDGroup[emPID_FollowSpdYaw];

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

    PIDGroup[emPID_FolloLinePosVertically].desired = 60 / 2;
    PIDGroup[emPID_FolloLinePosHorizontally].desired = 80 / 2;

    FollowManager.ptrPIDInfoH->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoH->OutLimitLow = -20;
    FollowManager.ptrPIDInfoV->OutLimitHigh = 20;
    FollowManager.ptrPIDInfoV->OutLimitLow = -20;

    FollowManager.CountDownNumMs = MAX_COUNTDOWN;
    FollowManager.TargetAltitudeCM = TARGETALTITUDECM;

    FollowManager.FrontOpenmvFramePtr = (OpenMVFrame_t *)UsartGroup[UART_A1].RxBuff;
    FollowManager.GroundOpenmvFramePtr = (OpenMVFrame_t *)UsartGroup[UART_A3].RxBuff;
    FollowManager.ptrUAVInfo = &g_UAVinfo;

    P1DIR &= ~(1 << BIT1);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    P1DIR &= ~(1 << BIT4);
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);

    FollowManager.ptrPIDInfoY->kp = 0.02f;
    FollowManager.ptrPIDInfoY->ki = 0.001f;

    FollowManager.ptrPIDInfoY->OutLimitLow = -0.5;
    FollowManager.ptrPIDInfoY->OutLimitHigh = 0.5;
    FollowManager.ptrPIDInfoY->desired = 160 / 2;
}

//��100hz���ٶ���ѯ 10ms
void UpdateCentControl(float dt)
{
    //�ж�OpenMV���ص������Ƿ���ã��е�ʱ��OpenMV�᷵����Ч����
    if (FollowManager.GroundOpenmvFramePtr->CentPoint.x1 > 200 || FollowManager.GroundOpenmvFramePtr->CentPoint.y1 > 200)
        return;

    //���¾���
    FollowManager.distance = FollowManager.FrontOpenmvFramePtr->cnt1 | FollowManager.FrontOpenmvFramePtr->Target >> 8;

    //���°�ť����ʵ��
    UpdateButton();

    //���³̿�״̬��
    UpdateStatus();

    //���³̿ض�����
    UpdateAction(dt);
}

float angle_temp_dt = 0;
//�˺���ֻ��״̬�жϺ�״̬����
void UpdateStatus()
{
    uint16_t distance = 0;
    static float pre_yaw_temp = 0;
    static int cnt = 0;

    //����ActionList�����ݣ����벻ͬ��״̬
    switch (FollowManager.ActionList)
    {
    //�ж�
    case ActionWaitting:
        //Do nothing;
        break;

    //����ʱ״̬
    case ActionCountdown:
    {
        //����ʱ�����ݳ�ʼ���λ��Follow_Init��
        FollowManager.CountDownNumMs--;

        //������ʱ����ʱ��״̬���ΪActionTakeOff
        if (FollowManager.CountDownNumMs <= 0)
        {
            FollowManager.ActionList = ActionTakeOff;
        }
    }
    break;

    //�Զ����״̬
    case ActionTakeOff:
    {
        //�Զ���ɶ�������ʱ��Ϊ5s��500 * 10ms = 5000ms = 5s����Ȼ������ActionHoverStartPoint������
        ActionHoldPoint(MAX_HOVER_ERR, 500, ActionHoverStartPoint);
    }
    break;

    //��ͣ������5S��Ȼ�������Զ�����״̬
    case ActionHoverStartPoint:
        ActionHoldPoint(MAX_HOVER_ERR, 300, ActionFindPole);
        //        ActionHoldPoint(MAX_HOVER_ERR, 200, ActionFindLandSpace);
        break;
    case ActionFindPole:
        // ��ʼ������˳ʱ��
        // ���Ҽ�¼��ת�ĽǶ�
        // �Ƿ�����ɫ�ˣ�
        if (FollowManager.FrontOpenmvFramePtr->FormType == Pole)
        {
            //      ��
            //      ֹͣ����

            //      ������һ������
            FollowManager.ActionList = ActionCloseToPole;
            //            FollowManager.ActionList = ActionResetAngle;
            //            FollowManager.ActionList = ActionTest;
        }
        break;
    case ActionTest:
        ActionHoldPoint(MAX_HOVER_ERR, 300, ActionPreLand);
        break;
    case ActionCloseToPole:
        // Ŀ�꣺������ɫ���ڻ�������
        // ��ǰ�ߣ����ҿ���PID��������
        // �Ƿ���뵽����50cm?
        //      ��
        //      ������һ������

        if (FollowManager.distance < 55)
        {
            cnt++;

            FollowManager.ActionList = ActionTurnRound;

            pre_yaw_temp = g_Attitude.yaw;
        }
        break;
    case ActionTurnRound:
        // Ŀ�꣺������ɫ���ڻ�������
        // ��������yaw�ǵ�360��
        //  yaw�Ƿ�������
        //      ��
        //      �л���Ѱ�ҽ����
        if (is_set_round(pre_yaw_temp, g_Attitude.yaw))
        {
            FollowManager.ActionList = ActionResetAngle;
        }
//        angle_temp_dt = absFloat(update_yaw_info_in_360() - pre_yaw_temp);
//        if(angle_temp_dt > 355)
//        {
//            FollowManager.ActionList = ActionLand;
//        }

        break;
    case ActionResetAngle:
        // �ص�����Ƕ�
        ActionHoldPoint(MAX_HOVER_ERR, 200, ActionFindLandSpace);

        break;
    case ActionFindLandSpace:
        // �����Ͻǵ�Ŀ����
        // �Ƿ��ֽ���㣬����ʶ�����ӵ�OpenMV�������Ƿ��ҵ�
        // ��
        // �л�������ģʽ
        if (FollowManager.GroundOpenmvFramePtr->FormType == Cirle)
        {
            FollowManager.ActionList = ActionPreLand;
        }
        break;
    case ActionPreLand:
        ActionHoldPoint(MAX_HOVER_ERR, 200, ActionLand);
        break;

    //�Զ�����״̬����ʱ�����Ժ󣬽�����������
    case ActionLand:
    {
        //����ʱ�߼�
        static int Cnt = MAX_TIMEOUT1;

        if (Cnt-- < 0)
        {
            FollowManager.ActionList = ActionLock;
        }
    }
    break;

    //��������
    case ActionLock:
        FollowManager.ActionList = ActionWaitting;
        break;
    default:
        break;
    }
}

extern float PIDGroup_desired_yaw_pos_tmp;
//ִֻ�ж���
void UpdateAction(float dt)
{
    static float last_yaw = 0;
    switch (FollowManager.ActionList)
    {
    //����ʱ����
    case ActionWaitting:
        //Do nothing7
        break;

    //�Զ��������
    case ActionTakeOff:
        sdk_takeoff(80);
        break;

    //��ͣ����
    case ActionHoverStartPoint:
        //���
        {
        }
        break;
    case ActionFindPole:
        sdk_yaw_set(180);
        last_yaw = sdk_manager.yaw_mark;
        break;
    case ActionCloseToPole:
        sdk_yaw_stop();
        sdk_velociyt_x_set(15);

        if (FollowManager.FrontOpenmvFramePtr->FormType == Pole)
        {
            HoldYawAngle(dt);
        }
        else
        {
        }
        break;
    case ActionTurnRound:
        sdk_velocity_reset();
        if (FollowManager.FrontOpenmvFramePtr->FormType == Pole)
        {
            HoldYawAngle(dt);
        }

        if (FollowManager.distance != 0xFF && FollowManager.distance != 20 && FollowManager.distance != 0)
        {
            if (FollowManager.distance > 65)
            {
                sdk_velociyt_x_set(15);
            }
            else if (FollowManager.distance < 35)
            {
                sdk_velociyt_x_set(-15);
            }
            else
            {
                sdk_velociyt_x_set(0);
            }
        }

        sdk_velociyt_y_set(10);
        break;
    case ActionResetAngle:
        sdk_velocity_reset();
        PIDGroup_desired_yaw_pos_tmp = 0;
        // �ص�����Ƕ�
        break;
    case ActionFindLandSpace:
        sdk_velocity_reset();
        sdk_velocity_set(20, 10);
        break;

    case ActionPreLand:
        //        if()
        HoldCurrentPostion(dt);
        break;
    case ActionTest:
        sdk_velociyt_x_set(15);
        //        sdk_yaw_stop();
        //        sdk_yaw_set(-180);
        break;
    case ActionGoLeft:

        break;
    case ActionGoBack:
        break;
    //�Զ�����
    case ActionLand:
        //��������
        sdk_land();
        HoldCurrentPostion(dt);
        break;

    //��������
    case ActionLock:
        //        g_UAVinfo.FMUflg->unlock = 0;
        break;
    default:
        break;
    }
}

void HoldCurrentPostion(float dt)
{
    static float OldPos[2];
    //��������LPF

    //���²�����
    PIDGroup[emPID_FolloLinePosVertically].measured = FollowManager.GroundOpenmvFramePtr->CentPoint.y1;
    PIDGroup[emPID_FolloLinePosHorizontally].measured = FollowManager.GroundOpenmvFramePtr->CentPoint.x1;

    PIDGroup[emPID_FolloLineSpdVertically].measured = (FollowManager.GroundOpenmvFramePtr->CentPoint.y1 - OldPos[0]);
    PIDGroup[emPID_FolloLineSpdHorizontally].measured = (FollowManager.GroundOpenmvFramePtr->CentPoint.x1 - OldPos[1]);

    OldPos[0] = FollowManager.GroundOpenmvFramePtr->CentPoint.y1;
    OldPos[1] = FollowManager.GroundOpenmvFramePtr->CentPoint.x1;

    UpdatePID(FollowManager.ptrPIDInfoH, dt); //PID
    UpdatePID(FollowManager.ptrPIDInfoV, dt); //PID

    PIDGroup[emPID_FolloLineSpdVertically].desired = FollowManager.ptrPIDInfoV->out;
    PIDGroup[emPID_FolloLineSpdHorizontally].desired = FollowManager.ptrPIDInfoH->out;

    UpdatePID(&PIDGroup[emPID_FolloLineSpdHorizontally], dt); //PID
    UpdatePID(&PIDGroup[emPID_FolloLineSpdVertically], dt);   //PID

    sdk_velocity_set(PIDGroup[emPID_FolloLineSpdVertically].out, PIDGroup[emPID_FolloLineSpdHorizontally].out);
}

bool ActionFormChange(int8_t HoldTime, FormType_t TargetFormType, FSMList_t NextAction)
{
    static int cnt = 0;
    bool ChangeFinished = false;

    if (FollowManager.GroundOpenmvFramePtr->FormType == TargetFormType)
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
    //�ж����������Ƿ���Ч����ʵ���ж�������������
    volatile static uint8_t input = 0;
    volatile static uint8_t input2 = 0;
    input = P1IN & BIT1;
    input2 = P1IN & BIT4;

    //�ж�Ѳ�߰�ť�Ƿ���
    if (input)
    {
    }
    else
    {
        FollowLine = true;
    }

    //�ж�Ѱ��ApriTag��ť�Ƿ���
    if (input2)
    {
    }
    else
    {
        FollowApriTag = true;
    }

    //�жϵ�ǰ�Ƿ񱻶ఴ
    if (FollowApriTag == false && FollowLine == false)
    {
        return;
    }
    else
    {
        static bool CloseGate = true;

        //�����߽��뵹��ʱ״̬
        if (CloseGate)
        {
            CloseGate = false;
            FollowManager.ActionList = ActionCountdown;
        }
    }
}

extern float PIDGroup_desired_yaw_pos_tmp;
void HoldYawAngle(float dt)
{
    //���²�����
    PIDGroup[emPID_FollowSpdYaw].measured = FollowManager.FrontOpenmvFramePtr->CentPoint.x1;
    UpdatePID(FollowManager.ptrPIDInfoY, dt);

    //    PIDGroup_desired_yaw_pos_tmp = -FollowManager.ptrPIDInfoY->out;
    sdk_yaw_little(FollowManager.ptrPIDInfoY->out);
    //    sdk_yaw_little(FollowManager.ptrPIDInfoY->out);
}

float update_yaw_info_in_360()
{
    float angle = 0;

    angle = g_Attitude.yaw;

    if (angle < 0)
    {
        angle += 360;
    }

    return angle;
}

bool is_set_round(float first_angle, float current_angle)
{
    static uint8_t step = 0;
    bool result = false;

    switch (step)
    {
    case 0:
        //waiting
        step = 1;
        break;
    case 1:
        //mark
        step = 2;
        break;
    case 2:
        if (absFloat(current_angle - first_angle) / 2 > 90)
        {
            step = 3;
        }
        break;
    case 3:
        if (absFloat(current_angle - first_angle) / 2 < 5)
        {
            step = 0;
            result = true;
        }
        break;
    default:
        break;
    }

    return result;
}
