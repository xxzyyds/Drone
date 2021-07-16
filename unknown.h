// 结构体：
//     emPID_Pitch_Spd = 0 ,    --->[外环] 倾斜
//     emPID_Roll_Spd,          --->[外环] 翻滚
//     emPID_Yaw_Spd,           --->[外环] 偏航

//     emPID_Pitch_Pos,         --->[内环] 倾斜
//     emPID_Roll_Pos,          --->[内环] 翻滚
//     emPID_Yaw_Pos,           --->[内环] 偏航

//     emPID_Height_Spd,        --->[外环] 高度
//     emPID_Height_Pos,        --->[内环] 高度

//     emPID_FolloLinePosVertically,
//     emPID_FolloLinePosHorizontally,
//     emPID_FolloLineSpdVertically,
//     emPID_FolloLineSpdHorizontally,
//     emPID_FollowSpdYaw 

//     emNum_Of_PID_List

// 私有方法：

//     float kp;           //< proportional gain
//     float ki;           //< integral gain
//     float kd;           //< derivative gain
//     float out;           //真正给到电机的输出
//     float Err;
//     float desired;       //< 期望值
//     float measured;      //

//     float DeathArea;

//     float Err_LimitHigh;
//     float Err_LimitLow;

//     float offset;      //
//     float prevError;    //< previous error
//     float integ;        //< integral

//     float IntegLimitHigh;       //< integral limit
//     float IntegLimitLow;

//     float OutLimitHigh;
//     float OutLimitLow;

//PID定义：13個結構體，PIDGroup[0]~[12]
//extern PIDInfo_t PIDGroup[13];

//PID修改
// ClacCascadePID(&PIDGroup[emPID_Roll_Spd] , &PIDGroup[emPID_Roll_Pos] , dt);          //X轴 [1]、[4]
// ClacCascadePID(&PIDGroup[emPID_Pitch_Spd] , &PIDGroup[emPID_Pitch_Pos] , dt);        //Y轴 [0]、[3]
// ClacCascadePID(&PIDGroup[emPID_Yaw_Spd] , &PIDGroup[emPID_Yaw_Pos] , dt);            //Z轴 [2]、[5] 【角度】

    // UpdatePID(pidAngE, dt);              -->外环[Pos]
    // pidRate->desired = pidAngE->out;     -->[外环输出作为内环的期望-->给出最后输出]
    // UpdatePID(pidRate, dt);              -->内环[Spd]

    // UpdatePID(~~, dt);    
        // pid->Err = pid->desired - pid->measured + pid->offset;        //当前角度与实际角度的误差，期望 - 测量【当前】 + 偏移
        // pid->integ += pid->Err * dt;     // integ?
        // deriv = -(pid->measured - pid->prevError)/dt;                            //deriv?
        // pid->out = pid->kp * pid->Err + pid->ki * pid->integ + pid->kd * deriv;  //PID输出

// 电机修改---根据上面的 out【原理？】
// MOTOR1 += +PIDGroup[emPID_Roll_Spd].out - PIDGroup[emPID_Pitch_Spd].out + PIDGroup[emPID_Yaw_Spd].out;
// MOTOR2 += +PIDGroup[emPID_Roll_Spd].out + PIDGroup[emPID_Pitch_Spd].out - PIDGroup[emPID_Yaw_Spd].out;
// MOTOR3 += -PIDGroup[emPID_Roll_Spd].out + PIDGroup[emPID_Pitch_Spd].out + PIDGroup[emPID_Yaw_Spd].out;
// MOTOR4 += -PIDGroup[emPID_Roll_Spd].out - PIDGroup[emPID_Pitch_Spd].out - PIDGroup[emPID_Yaw_Spd].out;

// ResetPID(void)---->停止在80cm


