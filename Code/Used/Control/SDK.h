#ifndef _SDK_H
#define _SDK_H

#include <stdint.h>
#include "FollowLine.h"

//T432 API�ӿ�

//T432�ٶȿ���
/*
       (+X)
        |
        |
(-Y)---------(+Y)
        |
        |
       (-X)


Speed:
���Ʒɻ����ݷ����ٶ�

//�������
T432_API_Speed(CmdTakeOff,0,0);

//��������
T432_API_Speed(CmdLand,0,0);

�ɻ�����߷��У����ǽ����ٶȲ�����30
T432_API_Speed(CmdSpeeedControl,0,20);
*/

typedef struct
{
    uint8_t sdk_alt_step;
    uint8_t sdk_yaw_step;
    float sdk_yaw_d_angle;
    uint16_t sdk_yaw_angle_count;
    float sdk_alt_out;
    float sdk_velocity_x;
    float sdk_velocity_y;
    bool sdk_auto_takeoff;

    //�����ٶȻ��֣��ó�����λ��
    float location_x;
    float location_y;

    float yaw_mark;
    float *yaw_pos_ptr;
} sdk_manager_t;

extern sdk_manager_t sdk_manager;

#define MAX_SPEED 30

/******************************************************************************
  * �������ƣ�sdk_init
  * ����������sdk�ĳ�ʼ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
void sdk_init(void);

/******************************************************************************
  * �������ƣ�sdk_yaw_set
* ����������ʹ��sdk�����˻�����ת�Ƿ���
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��void
******************************************************************************/
void sdk_yaw_set(float angle);
void sdk_target_set(float x_pos, float y_pos, float velocity_x, float velocity);


/******************************************************************************
  * �������ƣ�sdk_velocity_set
  * ����������ʹ��sdk�޸�����ˮƽ�ٶ�����ֵ
  * ��    �룺float x float y��ʾˮƽ�ٶȷ���ʹ�С
  * ��    ����void
  * ��    �أ�void 
    * ��    ע���ٶȲο�������԰��մ�ͼ�����У�һ����˵���ٶȾ���ֵ�趨��20������ɴ󲿷ֳ̿�����
  *
  *
******************************************************************************/
void sdk_velocity_set(float x, float y);

/******************************************************************************
  * �������ƣ�sdk_round_set
* ����������ʹ��sdk��Χ��ĳһ�뾶��Բ���˶����൱���Ƹ˷��У�����ʱ�����˻���ͷ��׼��
  * ��    �룺float distance, float angle, uint8_t R_
distance����ʾԲ���˶��İ뾶
float angle����ʾ���еĽǶ�
uint8_t R_����ʾ��ת������ת�����R����0�����������˻��Ƹ���ʱ����ת�����RС��0�����������˻��Ƹ�˳ʱ����ת
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��void
******************************************************************************/
void sdk_round_set(float distance, float angle, uint8_t R_);

/******************************************************************************
  * �������ƣ�sdk_alititude_set
  * ����������ʹ��sdk�޸������߶�ֵ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
    * ��    ע���ݲ�����
  *    
  *
******************************************************************************/
void sdk_alititude_set(float distance);

/******************************************************************************
  * �������ƣ�sdk_takeoff
  * ����������ʹ��sdk�������˻�����ɹ����������Զ�����������
  * ��    �룺alitude  �Զ���ɺ�ķ��и߶�
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��ע�ⰲȫ������ʹ��    
  *    
  *
******************************************************************************/
void sdk_takeoff(float alititude);

/******************************************************************************
  * �������ƣ�sdk_land
  * ����������ʹ��sdk�������˻��Ľ��乤���������Զ�����������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��ע�ⰲȫ������ʹ��    
  *    
  *
******************************************************************************/
void sdk_land(void);

/******************************************************************************
  * �������ƣ�sdk_lock
* ����������ʹ��sdk�����˻���������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��void
******************************************************************************/
void sdk_lock(void);

/******************************************************************************
  * �������ƣ�sdk_unlock
* ����������ʹ��sdk�����˻����н���
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��void
******************************************************************************/
void sdk_unlock(void);

/******************************************************************************
  * �������ƣ�sdk_velociyt_y_set
* �����������趨Y����ٶ�ֵ
* ��    �룺float y ��ʾY����ٶ��趨ֵ
  * ��    ����void
  * ��    �أ�void 
* ��    ע�������趨y����ٶ�ֵ
******************************************************************************/
void sdk_velociyt_y_set(float y);

/******************************************************************************
  * �������ƣ�sdk_velociyt_x_set
  * �����������趨X����ٶ�ֵ
  * ��    �룺float X ��ʾY����ٶ��趨ֵ
  * ��    ����void
  * ��    �أ�void
  * ��    ע�������趨x����ٶ�ֵ
******************************************************************************/
void sdk_velociyt_x_set(float x);

/******************************************************************************
  * �������ƣ�sdk_update
  * ����������ʹ��sdk�������ݸ��²���
  * ��    �룺float dt ��λ����ʱ��
  * ��    ����void
  * ��    �أ�void 
* ��    ע���������Ѿ�����kernel�����б���ִ�У�����Ҫ���и��ģ�������Ѿ�֪����
    *���ĵ����ݣ�����Խ��г���
  *    
  *
******************************************************************************/
void sdk_update(float dt);

/******************************************************************************
  * �������ƣ�is_yaw_set_compleate
* ����������ȷ���Ƿ�YAW���ת���Ѿ�����
* ��    �룺void
  * ��    ����void
  * ��    �أ�void 
* ��    ע��yaw��ת����Ҫʱ�䣬�˺�������ȷ��yaw��ת���Ƿ����
******************************************************************************/
bool is_yaw_set_compleate(void);

/******************************************************************************
  * �������ƣ�sdk_yaw_little
* �����������÷ɻ���ת΢С��yaw�ǣ�������0.5��֮��
* ��    �룺void
  * ��    ����void
  * ��    �أ�void 
* ��    ע��void
******************************************************************************/
void sdk_yaw_little(float yaw);

/******************************************************************************
  * �������ƣ�sdk_yaw_stop
* ����������ֹͣyaw��ת��
* ��    �룺void
  * ��    ����void
  * ��    �أ�void 
* ��    ע���൱���÷ɻ����¹�λ
******************************************************************************/
void sdk_yaw_stop(void);
void sdk_reset_Location(void);

/******************************************************************************
  * �������ƣ�sdk_velocity_reset
* ��������������ˮƽ�ٶ�
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
* ��    ע���൱���÷ɻ�������ͣ��ֹ
******************************************************************************/
void sdk_velocity_reset(void);

/******************************************************************************
  * �������ƣ�sdk_yaw_reset
* ��������������yaw��
* ��    �룺void
  * ��    ����void
  * ��    �أ�void 
* ��    ע���൱���÷ɻ����¹�λ
******************************************************************************/
void sdk_yaw_reset(void);
#endif