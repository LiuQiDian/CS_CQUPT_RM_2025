//发射，无需多言
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能.
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

#include "shoot.h"
#include "main.h"

#include "cmsis_os.h"

#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "pid.h"



#define shoot_laser_on()    laser_on()      //激光开启宏定义
#define shoot_laser_off()   laser_off()     //激光关闭宏定义
//微动开关IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)




/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          堵转倒转处理
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          初始化"gimbal_CV"变量
  * @param[out]     init: 新增了init_CV
  * @retval         none
  */
//static void CV_init(shoot_control_t *init);
static void shoot_bullet_control(void);

shoot_control_t shoot_control;          //射击数据


/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(shoot_current_t *shoot_init_current)
{
    uint8_t i;
    const static fp32 Trigger_speed_pid[3] = {TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD};
    const static fp32 Trigger_angle_pid[3] = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    const static fp32 fric_speed_pid[3] = {FRIC_SPEED_PID_KP, FIRC_SPEED_PID_KI, FRIC_SPEED_PID_KD};
    shoot_control.shoot_mode = SHOOT_STOP;
    //遥控器指针
    shoot_control.shoot_rc = get_remote_control_point();
    //获取拨弹盘电机指针
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    //初始化拨弹盘PID
    PID_init(&shoot_control.trigger_speed_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_PID_MAX_OUT, TRIGGER_PID_MAX_IOUT);
    PID_init(&shoot_control.trigger_angle_pid, PID_POSITION, Trigger_angle_pid, TRIGGER_PID_ANGLE_MAX_OUT, TRIGGER_PID_ANGLE_IOUT);
    // 获取摩擦轮电机数据指针，初始化PID
    for (i = 0; i < 4; i++)
    {
        shoot_control.fric_motor[i].fric_motor_measure = get_fric_motor_measure_point(i);
        PID_init(&shoot_control.fric_speed_pid[i], PID_POSITION, fric_speed_pid, FRIC_PID_MAX_OUT, FRIC_PID_MAX_IOUT);
    }
    //更新拨弹盘数据
    shoot_feedback_update();
    shoot_init_current->trigger_give_current = 0;
    for (i = 0; i < 4; i++)
    {
        shoot_init_current->fric_give_current[i] = 0;
    }
    shoot_control.change_fric = 0;
    shoot_control.move_flag = 0;
    shoot_control.last_state = 0;
}



/**
  * @brief          射击循环
  * @param[in]      void
  * @retval         返回can控制值
  */
void shoot_control_loop(shoot_current_t *shoot_current)
{
	uint8_t i = 0;
    //状态机函数用于判断并更新状态，shoot_control_loop用于根据状态进行控制操作
    shoot_set_mode();        //设置状态机
    shoot_feedback_update(); //更新数据

    switch (shoot_control.shoot_mode)
    {
    case ANGLE_SHOOT:
        shoot_control.fric_speed_set = FRIC_SPEED;
        break;
    case SHOOT_READY_FRIC:
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.fric_speed_set = FRIC_SPEED;
        break;
    case SPEED_SHOOT:
        if(fabs(shoot_control.fric_motor[0].speed) > 10.0f  &&  fabs(shoot_control.fric_motor[1].speed) > 10.0f)
        {
            shoot_control.trigger_speed_set = TRIGGER_SPEED;
        }
        else
        {
            shoot_control.trigger_speed_set = 0.0f;
        }
        shoot_control.fric_speed_set = FRIC_SPEED;
        break;
    case SHOOT_STOP:
    default:
        shoot_control.trigger_speed_set = 0.0f;
        shoot_control.fric_speed_set = 0.0f;
        break;
    }

    switch (shoot_control.shoot_mode)
    {
    case ANGLE_SHOOT:
        shoot_bullet_control();
        break;
    case SPEED_SHOOT:
        trigger_motor_turn_back();
        break;
    }

    PID_calc(&shoot_control.trigger_speed_pid, shoot_control.trigger_speed, shoot_control.trigger_speed_set);
    shoot_current->trigger_give_current = (int16_t)(shoot_control.trigger_speed_pid.out);

    shoot_control.fric_motor[0].speed_set = -shoot_control.fric_speed_set;
    shoot_control.fric_motor[1].speed_set = shoot_control.fric_speed_set;
    shoot_control.fric_motor[2].speed_set = shoot_control.fric_speed_set;
    shoot_control.fric_motor[3].speed_set = -shoot_control.fric_speed_set;
    for (i = 0; i < 4; i++)
    {
        PID_calc(&shoot_control.fric_speed_pid[i], shoot_control.fric_motor[i].speed, shoot_control.fric_motor[i].speed_set);
        shoot_current->fric_give_current[i] = (int16_t)(shoot_control.fric_speed_pid[i].out);
    } 
}


/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    if (switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
     
        shoot_control.shoot_mode = SHOOT_STOP;
        
    }
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_Q)
        {
          shoot_control.change_fric = 1;
        }
        else if(shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_E)
        {
          shoot_control.change_fric = 0;
        }
        
        if (shoot_control.change_fric)
        {
            shoot_control.shoot_mode = SHOOT_READY_FRIC;
        }
        else
        {
            shoot_control.shoot_mode = SHOOT_STOP;
        }
        

        if(shoot_control.last_state == 0 && shoot_control.shoot_rc->mouse.press_l)
        {
            shoot_control.shoot_mode = ANGLE_SHOOT;
        }
        else if(shoot_control.shoot_rc->mouse.press_r)
        {
            shoot_control.shoot_mode = SPEED_SHOOT;
        }
        else if(!shoot_control.shoot_rc->mouse.press_r && !shoot_control.move_flag)
        {
            SHOOT_STOP;
        }
        
        shoot_control.last_state == shoot_control.shoot_rc->mouse.press_l;
    }
    else if (switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        shoot_control.shoot_mode = SPEED_SHOOT;
    }

    //如果云台状态是 无力状态，就关闭射击
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

   
}
/**
  * @brief          射击数据更新
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
    uint8_t i = 0;
    //更新拨弹盘电机速度
    shoot_control.trigger_speed = shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_ANGULAR_SPEED;
    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 51圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd > HALF_ECD_RANGE)
    {
        shoot_control.ecd_count--;
    }
    else if (shoot_control.shoot_motor_measure->ecd - shoot_control.shoot_motor_measure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -FULL_COUNT;
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT;
    }

    //计算输出轴角度
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.shoot_motor_measure->ecd) * MOTOR_ECD_TO_ANGLE;
    //更新摩擦轮电机速度
    for (i = 0; i < 4; i++)
    {
        shoot_control.fric_motor[i].speed = shoot_control.fric_motor[i].fric_motor_measure->speed_rpm * MOTOR_RPM_TO_LINEAR_SPEED;
    }
  
}

static void trigger_motor_turn_back(void)  //防堵转
{
    if( shoot_control.block_time >= BLOCK_TIME)
    {
        shoot_control.trigger_speed_set = (-5);
    }

    if(fabs(shoot_control.trigger_speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

static void shoot_bullet_control(void)
{

        //每次拨动 1/3PI的角度
    if (shoot_control.move_flag == 0)
    {
        shoot_control.set_angle = rad_format(shoot_control.angle + PI / 3);
        shoot_control.move_flag = 1;
    }
    //到达角度判断
    if (rad_format(shoot_control.set_angle - shoot_control.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        shoot_control.trigger_speed_set = TRIGGER_SPEED - 4.0f;
        trigger_motor_turn_back();
    }
    else
    {
        shoot_control.move_flag = 0;
    }
}



