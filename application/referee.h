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
#ifndef REFEREE_H
#define REFEREE_H
#include "main.h"
#include "protocol.h"

typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
    BLUE_HERO       = 11,
    BLUE_ENGINEER   = 12,
    BLUE_STANDARD_1 = 13,
    BLUE_STANDARD_2 = 14,
    BLUE_STANDARD_3 = 15,
    BLUE_AERIAL     = 16,
    BLUE_SENTRY     = 17,
} robot_id_t;
typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;
typedef __packed struct //0x0001
{
    uint8_t game_type : 4; //��������
    uint8_t game_progress : 4; //��ǰ�����׶�
    uint16_t stage_remain_time; //��ǰ�׶�ʣ��ʱ�䣬��λ����
    uint64_t SyncTimeStamp; //UNIX ʱ�䣬����������ȷ���ӵ�����ϵͳ�� NTP ����������Ч
} game_status_t;

typedef __packed struct //0x0002
{
    uint8_t winner; //0��ƽ�� 1���췽ʤ�� 2������ʤ��
} game_result_t;
typedef __packed struct //0x0003
{
    uint16_t red_1_robot_HP;  //�� 1 Ӣ�ۻ�����Ѫ�������û�����δ�ϳ����߱����£���Ѫ��Ϊ 0
    uint16_t red_2_robot_HP;  //�� 2 ���̻�����Ѫ��
    uint16_t red_3_robot_HP;  //�� 3 ����������Ѫ��
    uint16_t red_4_robot_HP;  //�� 4 ����������Ѫ��
    uint16_t red_5_robot_HP;  //�� 5 ����������Ѫ��
    uint16_t red_7_robot_HP;  //�� 7 �ڱ�������Ѫ��
    uint16_t red_outpost_HP;  //�췽ǰ��վѪ��
    uint16_t red_base_HP;     //�췽����Ѫ��
    uint16_t blue_1_robot_HP; //�� 1 Ӣ�ۻ�����Ѫ��
    uint16_t blue_2_robot_HP; //�� 2 ���̻�����Ѫ��
    uint16_t blue_3_robot_HP; //�� 3 ����������Ѫ��
    uint16_t blue_4_robot_HP; //�� 4 ����������Ѫ��
    uint16_t blue_5_robot_HP; //�� 5 ����������Ѫ��
    uint16_t blue_7_robot_HP; //�� 7 �ڱ�������Ѫ��
    uint16_t blue_outpost_HP; //����ǰ��վѪ��
    uint16_t blue_base_HP;    //��������Ѫ��
} game_robot_HP_t;
typedef __packed struct //0x0101
{
    uint32_t event_data;
} event_data_t;

typedef __packed struct //0x0102
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103
{
    uint8_t reserved;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_booking_t;

typedef __packed struct //0x0104
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

typedef __packed struct //0x105
{
    uint8_t dart_remaining_time;
    uint16_t dart_info;
} dart_info_t;
typedef __packed struct //0x0201
{
	uint8_t robot_id;                             //�������� ID
    uint8_t robot_level;                          //�����˵ȼ�
    uint16_t current_HP;                          //�����˵�ǰѪ��
    uint16_t maximum_HP;                          //������Ѫ������
    uint16_t shooter_barrel_cooling_value;        //������ǹ������ÿ����ȴֵ
    uint16_t shooter_barrel_heat_limit;           //������ǹ����������
    uint16_t chassis_power_limit;                 //�����˵��̹�������
    uint8_t power_management_gimbal_output : 1;   //��Դ����ģ�����������gimbal �������0 Ϊ�������1 Ϊ 24V ���
    uint8_t power_management_chassis_output : 1;  //chassis �������0 Ϊ�������1 Ϊ 24V ���
    uint8_t power_management_shooter_output : 1;  //shooter �������0 Ϊ�������1 Ϊ 24V ���
} robot_status_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_voltage;             //��Դ����ģ��� chassis �������ѹ����λ��mV��
    uint16_t chassis_current;             //��Դ����ģ��� chassis �������������λ��mA��
    float chassis_power;                  //���̹��ʣ���λ��W��
    uint16_t buffer_energy;               //������������λ��J��
    uint16_t shooter_17mm_1_barrel_heat;  //�� 1 �� 17mm ���������ǹ������
    uint16_t shooter_17mm_2_barrel_heat;  //�� 2 �� 17mm ���������ǹ������
    uint16_t shooter_42mm_barrel_heat;    //42mm ���������ǹ������
} power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;                          //��������λ�� x ���꣬��λ��m
    float y;                          //��������λ�� y ���꣬��λ��m
    float angle;                      //�������˲���ģ��ĳ��򣬵�λ���ȡ�����Ϊ 0 ��
} robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t recovery_buff;           //�����˻�Ѫ���棨�ٷֱȣ�ֵΪ 10 ��ʾÿ��ָ�Ѫ�����޵� 10%��
    uint8_t cooling_buff;            //������ǹ����ȴ���ʣ�ֱ��ֵ��ֵΪ 5 ��ʾ 5 ����ȴ��
    uint8_t defence_buff;            //�����˷������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩
    uint8_t vulnerability_buff;      //�����˸��������棨�ٷֱȣ�ֵΪ 30 ��ʾ-30%�������棩
    uint16_t attack_buff;            //�����˹������棨�ٷֱȣ�ֵΪ 50 ��ʾ 50%�������棩
} buff_t;

typedef __packed struct //0x0205
{
    uint8_t airforce_status;         //���л�����״̬��0 Ϊ������ȴ��1 Ϊ��ȴ��ϣ�2 Ϊ���ڿ���֧Ԯ��
    uint8_t time_remain;             //��״̬��ʣ��ʱ�䣨��λΪ���룬����ȡ��������ȴʱ��ʣ�� 1.9 ��ʱ����ֵΪ 1������ȴʱ��Ϊ 0����δ���п���֧Ԯ�����ֵΪ 0
} air_support_data_t;

typedef __packed struct //0x0206
{
    uint8_t armor_id : 4;             //����Ѫԭ��Ϊװ��ģ�鱻���蹥������ײ�������߻����ģ������ʱ���� 4 bit ��ɵ���ֵΪװ��ģ������ģ��� ID ��ţ�������ԭ���¿�Ѫʱ������ֵΪ 0
    uint8_t HP_deduction_reason : 4;  //Ѫ���仯����
} hurt_data_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;             //��������
    uint8_t shooter_number;          //������� ID��
    uint8_t launching_frequency;     //�������٣���λ��Hz��
    float initial_speed;             //������ٶȣ���λ��m/s��
} shoot_data_t;
typedef __packed struct //0x0208
{
 uint16_t projectile_allowance_17mm; //17mm ������������
 uint16_t projectile_allowance_42mm; //42mm ������������
 uint16_t remaining_gold_coin;       //ʣ��������
} projectile_allowance_t;
typedef __packed struct //0x0209
{
 uint32_t rfid_status;
} rfid_status_t;
typedef __packed struct
{
    uint8_t bullet_remaining_num;
} ext_bullet_remaining_t;
typedef __packed struct //0x020A
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;
typedef __packed struct //0x020B
{
 float hero_x;                      //����Ӣ�ۻ�����λ�� x �����꣬��λ��m
 float hero_y;                      //����Ӣ�ۻ�����λ�� y �����꣬��λ��m
 float engineer_x;                  //�������̻�����λ�� x �����꣬��λ��m
 float engineer_y;                  //�������̻�����λ�� y �����꣬��λ��m
 float standard_3_x;                //���� 3 �Ų���������λ�� x �����꣬��λ��m
 float standard_3_y;                //���� 3 �Ų���������λ�� y �����꣬��λ��m
 float standard_4_x;                //���� 4 �Ų���������λ�� x �����꣬��λ��m
 float standard_4_y;                //���� 4 �Ų���������λ�� x �����꣬��λ��m
 float standard_5_x;                //���� 5 �Ų���������λ�� x �����꣬��λ��m
 float standard_5_y;                //���� 5 �Ų���������λ�� y �����꣬��λ��m
} ground_robot_position_t;
typedef __packed struct //0x020C
{
 uint8_t mark_hero_progress;
 uint8_t mark_engineer_progress;
 uint8_t mark_standard_3_progress;
 uint8_t mark_standard_4_progress;
 uint8_t mark_standard_5_progress;
 uint8_t mark_sentry_progress;
} radar_mark_data_t;
typedef __packed struct //0x020D
{
 uint32_t sentry_info;
} sentry_info_t;
typedef __packed struct //0x020E
{
 uint8_t radar_info;
} radar_info_t;
typedef __packed struct //0x0301
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[113];
} robot_interaction_data_t;

typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

typedef __packed struct
{
uint16_t mouse_x;
uint16_t mouse_y;
uint16_t mouse_z;
uint8_t left_button_down;
uint8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_robot_command_t;


extern game_robot_HP_t game_robot_HP;
extern robot_status_t robot_status;
extern game_status_t game_status;
extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern void get_chassis_power_and_buffer(float *power, uint16_t *buffer);
extern uint16_t get_shoot_heat(void);
extern uint8_t get_robot_id(void);
extern uint8_t get_team_color(void);
extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);
#endif
