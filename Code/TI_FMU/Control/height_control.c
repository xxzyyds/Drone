/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：height_control.c
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
高度控制驱动


*/
//外部文件引用
#include "height_control.h"
#include "control.h"
#include "SPL06.h"
#include "imu.h"
#include "mpu6050.h"
#include "fmuConfig.h"
#include "myMath.h"
#include "math.h"
#include "Remote.h"
#include "pid.h"

//宏定义区
#define IN_RANGE(value, min, max)        ( (value)>(min) && (value)<(max) )
#define THROTTLE_BASE 500

//Extern引用
extern SPL06Manager_t g_SPL06Manager;


//私有函数区
float constrain_float(float amt, float low, float high);
float ctrl_get_boosted_throttle(float throttle_in);
void UpdateAlt(void);

//私有变量区
HeightInfo_t HeightInfo;
float dt2 = 0;
bool Acc_Enable_Flag = false;

/******************************************************************************
  * 函数名称：HeightInit
  * 函数描述：高度函数初始化
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void HeightInit()
{
    HeightInfo.Alt = 0;
    HeightInfo.Z_Speed = 0;
}

/******************************************************************************
  * 函数名称：UpdateAltInfo
  * 函数描述：更新高度信息
  * 输    入：float dt:单位运行时间
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void UpdateAltInfo(float dt)
{  
    //更新高度值
    UpdateAlt();
}

/******************************************************************************
  * 函数名称：UpdateAlt
  * 函数描述：更新气压计检测的高度值
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void UpdateAlt()
{
    UpdateSPL06Info();
    HeightInfo.Alt = g_SPL06Manager.fRelative_Alt * 100;   //cm
}

/******************************************************************************
  * 函数名称：ControlAlt
  * 函数描述：高度控制函数
  * 输    入：
  * float dt:单位运行时间
  *
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *    
******************************************************************************/
void ControlAlt(float dt)
{
    /*更新测量值*/
    PIDGroup[emPID_Height_Pos].measured = HeightInfo.Alt;
    PIDGroup[emPID_Height_Spd].measured = HeightInfo.Z_Speed;

    ClacCascadePID(&PIDGroup[emPID_Height_Spd],&PIDGroup[emPID_Height_Pos], dt);      //X轴
    
    //输出要给一个基准油门
    HeightInfo.Thr = PIDGroup[emPID_Height_Spd].out + THROTTLE_BASE;
}

/******************************************************************************
  * 函数名称：ctrl_get_boosted_throttle
  * 函数描述：油门倾角补偿值
  * 输    入：
  * float throttle_in：油门输入值
  * 
  * 输    出：void
  * 返    回：油门输出值
  * 备    注：null
  *    
  *
******************************************************************************/
float ctrl_get_boosted_throttle(float throttle_in)
{
    float cos_tilt = cosine(g_Attitude.roll)*sine(g_Attitude.pitch);
    float inverted_factor = constrain_float(2.0f * cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f / constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    
    return throttle_out;
}

/******************************************************************************
  * 函数名称：constrain_float
  * 函数描述：数据限幅函数
  * 输    入：
  * float amt：需要限幅的数据
  * float low：限幅最小值
  * float high：限幅最大值
  * 输    出：void
  * 返    回：限幅后的值
  * 备    注：null
  *    
  *
******************************************************************************/
float constrain_float(float amt, float low, float high)
{
    if (isnan(amt)) 
   {
        return (low + high) * 0.5f;
    }
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

//======================ANOTC==========================================
#include "gcs.h"
#include "program_ctrl.h"
//程序移植映射表
//=====mapping=====

#define PROGRAM_CTRL_VELOCITY_Z             (program_ctrl.vel_cmps_h[Z])
#define PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL  (program_ctrl.auto_takeoff_land_velocity)

//=====mapping=====

#define MAX_EXP_WZ_VEL_UP 200
#define MAX_EXP_WZ_VEL_DW 150
#define MAX_EXP_WZ_ACC    500


//wz_ctrl struct
static float exp_vel_transition[4];
static float exp_vel_d;

static float exp_acc;
static float fb_acc;
static float exp_vel;
static float fb_vel;
static float exp_hei;
static float fb_hei;

//pid struct
static float hei_err,vel_err,acc_err,acc_err_i,acc_out,wz_out;

#define H_KP 1.5f
#define V_KP 5.0f
#define V_KD 0.05f
#define A_KP 0.4f
#define A_KI 0.6f

//extern uint8_t fc_state
extern int16_t ExpAlt;
extern int16_t FbAlt;
uint8_t fc_state_take_off = 0;
void ALT_Ctrl(float dT_s)
{
	//==input calculate
	//fb
	fb_vel = HeightInfo.Z_Speed;
	fb_hei = HeightInfo.Z_Postion;
	fb_acc = HeightInfo.Z_Acc;
    
	//exp
	exp_vel_transition[0] = (Remote.thr - 1500) * 0.001f;
    
	//deadzone
	if(exp_vel_transition[0]<0.1f && exp_vel_transition[0]>-0.1f)
	{
		exp_vel_transition[0] = 0;
	}
	//
	if(exp_vel_transition[0]>0)
	{
		//摇杆设置的Z速度 + 程控Z速度 + 起飞降落Z速度
		exp_vel_transition[1] = exp_vel_transition[0] *MAX_EXP_WZ_VEL_UP + PROGRAM_CTRL_VELOCITY_Z + PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL;
	}
	else
	{
		exp_vel_transition[1] = exp_vel_transition[0] *MAX_EXP_WZ_VEL_DW + PROGRAM_CTRL_VELOCITY_Z + PROGRAM_CTRL_AUTO_TAKEOFF_LAND_VEL;
	}
	//
	float tmp = MAX_EXP_WZ_ACC*dT_s;
	exp_vel_d = (exp_vel_transition[1] - exp_vel_transition[2]);
	if(exp_vel_d > tmp)
	{
		exp_vel_d = tmp;
	}
	else if(exp_vel_d < -tmp)
	{
		exp_vel_d = -tmp;
	}
	//
	exp_vel_transition[2] += exp_vel_d;
	//
	exp_vel_transition[3] += 0.2f *(exp_vel_transition[2] - exp_vel_transition[3]);
	
	//==exp_val state
	//
	if(g_UAVinfo.UAV_Mode >= Altitude_Hold && fc_state_take_off != 0 )
  {
		exp_vel = exp_vel_transition[3];
		//
		exp_hei += exp_vel *dT_s;
		//
		if(exp_hei > fb_hei+150)
		{
			exp_hei = fb_hei+150;
		}
		else if(exp_hei < fb_hei-150)
		{
			exp_hei = fb_hei-150;
		}
	}
	else
	{
		exp_vel = 0;
		exp_hei = fb_hei;
	}
	//==ctrl
    
    ExpAlt = exp_hei;
    FbAlt = fb_hei;
    //高度限幅，升限为1.5M
//    exp_hei = LIMIT(exp_hei,-20,150);
    
	hei_err = (exp_hei - fb_hei);
	vel_err = ((H_KP *hei_err  + exp_vel) - (fb_vel + V_KD *fb_acc));
	exp_acc = (V_KP *vel_err);
	acc_err = exp_acc - fb_acc;
	acc_err_i += A_KI *acc_err *dT_s;
	acc_err_i = (acc_err_i > 600)?600:((acc_err_i<0)?0:acc_err_i);
	//output
	acc_out = A_KP *exp_acc;
	wz_out = acc_out + acc_err_i;
	wz_out = (wz_out > 1000)?1000:((wz_out < 0)?0:wz_out);
	HeightInfo.Thr = wz_out;
	
	//unlock state
	
	if(g_FMUflg.unlock == 0 )
	{
		acc_err_i = 0;
		exp_hei = fb_hei;
		fc_state_take_off = 0;
	}
	else
	{
		if(g_UAVinfo.UAV_Mode >= Altitude_Hold)
		{
			//有向上的目标速度，状态切换为起飞
			if((int16_t)(exp_vel_transition[2])>0)
			{
				fc_state_take_off = 1;
			}
		}
		else//g_UAVinfo.UAV_Mode < Altitude_Hold
		{
			fc_state_take_off = 1;
		}
	}
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
