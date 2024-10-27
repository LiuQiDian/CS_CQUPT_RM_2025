/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include <stdbool.h>


//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1

#define MOTOR_RPM_TO_ANGULAR_SPEED  0.0014494083741176f //rpm转换角速度rad/s (减速比51:1)
#define MOTOR_RPM_TO_LINEAR_SPEED   0.0031416017f          //rpm转换线速度m/s
#define MOTOR_ECD_TO_ANGLE          0.00001503902725490196f //编码器值转换角度

//拨弹速度
#define TRIGGER_SPEED               8.0f //(rad/s)
//摩擦轮速度
#define FRIC_SPEED                  15.0f //(m/s)


//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  500
#define REVERSE_TIME                250
#define REVERSE_SPEED_LIMIT         13.0f


//拨弹轮电机PID速度单环
#define TRIGGER_SPEED_PID_KP        1000.0f
#define TRIGGER_SPEED_PID_KI        0.15f
#define TRIGGER_SPEED_PID_KD        0.0f

#define TRIGGER_PID_MAX_OUT   16000.0f
#define TRIGGER_PID_MAX_IOUT  2000.0f

//拨弹轮电机PID角度单环
#define TRIGGER_ANGLE_PID_KP        1300.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_PID_ANGLE_MAX_OUT   10000.0f
#define TRIGGER_PID_ANGLE_IOUT  1000.0f
//摩擦轮电机PID
#define FRIC_SPEED_PID_KP     15000.0f
#define FIRC_SPEED_PID_KI     3.0f
#define FRIC_SPEED_PID_KD     0.0f

#define FRIC_PID_MAX_OUT   16000.0f
#define FRIC_PID_MAX_IOUT  7000.0f

#define SHOOT_HEAT_REMAIN_VALUE     80

#define FULL_COUNT 25
typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    ANGLE_SHOOT,
    SPEED_SHOOT,
} shoot_mode_e;

typedef struct//摩擦轮电机数据结构体
{
  const motor_measure_t *fric_motor_measure;
  fp32 accel;
  fp32 speed;
  fp32 speed_set;
  int16_t give_current;
} fric_motor_t;

typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure; //拨弹盘电机数据
    fric_motor_t fric_motor[4];          //摩擦轮电机数据
    pid_type_def fric_speed_pid[4];     //摩擦轮电机速度pid
    pid_type_def trigger_speed_pid;
    pid_type_def trigger_angle_pid;
    fp32 trigger_speed_set;
    fp32 trigger_speed;
    fp32 fric_speed_set;
    int16_t trigger_give_current;
    uint16_t heat_limit;
    uint16_t heat;
    const bool *fire;
    uint16_t block_time;
    uint16_t firetime;
    bool change_fric;
    uint16_t trigger_angle;
    uint8_t ecd_count;
    fp32 angle;
    fp32 set_angle;
    bool_t move_flag;
    bool_t last_state;
    uint16_t reverse_time;
} shoot_control_t;



typedef struct
{
  int16_t trigger_give_current;
	int16_t fric_give_current[4];
} shoot_current_t;


//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(shoot_current_t *shoot_init_current);
extern void shoot_control_loop(shoot_current_t *shoot_current);
#endif
