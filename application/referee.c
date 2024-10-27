/*�������Ҫʵ���������ϵͳͨ����ص����ݴ����ܡ��������˸����������ص����ݽṹ�����ڽ��պͽ�������ϵͳ���͵����ݰ������ṩ��һЩ��������ȡ�ض��ı������ݺ�״̬��Ϣ��*/
/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.c/h
  * @brief      Updated the referee system Communication Protocol 1.6.1
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/3/6         YZX             1.���²���ϵͳͨ��Э����1.6.1�����������Զ�������UI��
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"
#include "detect_task.h"
#include "bsp_buzzer.h"



frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_status;
game_result_t game_result;
game_robot_HP_t game_robot_HP;


event_data_t field_event;
ext_supply_projectile_action_t supply_projectile_action_t;
ext_supply_projectile_booking_t supply_projectile_booking_t;
referee_warning_t referee_warning;


robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t game_robot_pos_t;
buff_t buff_musk_t;
air_support_data_t robot_energy_t;
hurt_data_t robot_hurt_t;
shoot_data_t shoot_data;
ext_bullet_remaining_t bullet_remaining_t;
robot_interaction_data_t student_interactive_data_t;

ext_robot_command_t robot_command_t;



void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));


    memset(&field_event, 0, sizeof(event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));


    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(buff_t));
    memset(&robot_energy_t, 0, sizeof(air_support_data_t));
    memset(&robot_hurt_t, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));


    memset(&student_interactive_data_t, 0, sizeof(robot_interaction_data_t));

    memset(&robot_command_t,0, sizeof(ext_robot_command_t));

}

void referee_data_solve(uint8_t *frame)
{
    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
    buzzer_on(100,300);
	
    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_status, frame + index, sizeof(game_status_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index, sizeof(event_data_t));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index, sizeof(ext_supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index, sizeof(robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index, sizeof(buff_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index, sizeof(air_support_data_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index, sizeof(hurt_data_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data_t, frame + index, sizeof(robot_interaction_data_t));
        }
        break;
				case ROBOT_COMMAND_CMD_ID:
				{
							memcpy(&robot_command_t, frame + index,  sizeof(ext_robot_command_t));
                }	
        break;				
        default:
        {
            break;
        }
    }
}

void get_chassis_power_and_buffer(float *power, uint16_t *buffer)
{
    *power = power_heat_data.chassis_power;
    *buffer = power_heat_data.buffer_energy;

}


uint8_t get_robot_id(void)
{
    return robot_status.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_status.shooter_barrel_heat_limit;
    *heat0 = power_heat_data.shooter_17mm_1_barrel_heat;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_status.shooter_barrel_heat_limit;
    *heat1 = power_heat_data.shooter_17mm_2_barrel_heat;  //�� 2 �� 17mm ���������ǹ������
}

uint8_t get_team_color(void) //�������ڱ��ڴ��ҡ�
{
       switch (robot_status.robot_id)
    {
        case 1:
        {
           return 0;
        }
        break;
        case 2:
        {
           return 0;
        }
        break;
        case 3:
        {
           return 0;
        }
        break;
        case 4:
        {
           return 0;
        }
        break;
        case 5:
        {
           return 0;
        }
        break;
        case 6:
        {
           return 0;
        }
        break;
        case 7:
        {
           return 0;
        }
        break;
        case 8:
        {
           return 0;
        }
        break;
        case 9:
        {
           return 0;
        }
        break;
        case 10:
        {
           return 0;
        }
        break;
        case 11:
        {
           return 0;
        }
        break;
        case 101:
        {
           return 1;
        }
        break;
        case 102:
        {
           return 1;
        }
        break;

        case 103:
        {
           return 1;
        }
        break;
        case 104:
        {
           return 1;
        }
        break;
        case 105:
        {
           return 1;
        }
        break;
        case 106:
        {
           return 1;
        }
        break;
        case 107:
        {
           return 1;
        }
        break;
        case 108:
        {
           return 1;
        }
        break;
        case 109:
        {
           return 1;
        }
        break;
        case 110:
        {
           return 1;
        }
        break;
        case 111:
        {
           return 1;
        }
        break;

    }
}

uint16_t get_shoot_heat(void)  //˫ǹ���ڱ�
{
    if(power_heat_data.shooter_17mm_1_barrel_heat>power_heat_data.shooter_17mm_2_barrel_heat)
    {
        return power_heat_data.shooter_17mm_1_barrel_heat;
    }
    else
    {
        return power_heat_data.shooter_17mm_2_barrel_heat;
    }
}
