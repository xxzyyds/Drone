/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：motor.h
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
#ifndef __MOTOR_H
#define __MOTOR_H
//外部文件引用
#include "include.h"
#include "math.h"

//宏定义区

//数据结构声明

//Extern引用

//函数声明
void Motor_Init(void);
void UpdateMotor(int16_t M1, int16_t M2, int16_t M3, int16_t M4);

#endif

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
