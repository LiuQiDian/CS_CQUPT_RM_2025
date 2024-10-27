//??????????? USB ??????????????????
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb ‰≥ˆ¥ÌŒÛ–≈œ¢
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "voltage_task.h"
#include "gimbal_task.h"
#include "chassis_task.h"


#include "referee.h"
static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;

extern gimbal_control_t gimbal_control;
extern robot_status_t robot_status;
extern chassis_move_t chassis_move;
extern power_heat_data_t power_heat_data;
extern game_robot_HP_t game_robot_HP;
extern shoot_data_t shoot_data;


//gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd

//chassis_move.motor_chassis[0].give_current

void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();


    while(1)
    {
        osDelay(1000);
			usb_printf("limit=%d	power=%f	buffer=%d\r\n",robot_status.chassis_power_limit,power_heat_data.chassis_power,power_heat_data.buffer_energy);
			 // usb_printf("%d\r\n",robot_state.robot_id);
				//usb_printf("%d\r\n",power_heat_data_t.chassis_power);
				//usb_printf("%d\r\n",game_robot_HP_t.red_5_robot_HP);
//				usb_printf("vx_set=%d\r\n",chassis_move.vx_set);
//				usb_printf("vy_set=%d\r\n",chassis_move.vy_set);
//				usb_printf("wz_set=%d\r\n",chassis_move.wz_set);
//				usb_printf("[0].speed_set=%d\r\n",chassis_move.motor_chassis[0].speed_set);
//				usb_printf("[1].speed_set=%d\r\n",chassis_move.motor_chassis[1].speed_set);
//				usb_printf("[2].speed_set=%d\r\n",chassis_move.motor_chassis[2].speed_set);
//				usb_printf("[3].speed_set=%d\r\n",chassis_move.motor_chassis[3].speed_set);

//					usb_printf("rc.X_channel=%d\r\n",chassis_move.chassis_RC->rc.ch[CHASSIS_X_CHANNEL]);
//					usb_printf("rc.Y_channel=%d\r\n",chassis_move.chassis_RC->rc.ch[CHASSIS_Y_CHANNEL]);		

//			usb_printf("vx.out=%d\r\n",chassis_move.chassis_cmd_slow_set_vx.out);
//			usb_printf("vy.out=%d\r\n",chassis_move.chassis_cmd_slow_set_vy.out);


		}
		

}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf, len);
}
