#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)              // 依据电机手册参数限值
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
#define DM_KP_MIN (-100.0f)
#define DM_KP_MAX 100.0f
#define DM_KD_MIN (-100.0f)
#define DM_KD_MAX 100.0f

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;                     // 电机速度(rad/s)
    float last_position;
    float position;
    float torque;                       // 电机力矩(N.m)
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
}DM_Motor_Measure_s;

// 达妙电机CAN发送结构体
typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;

// 达妙电机MIT参数结构体
typedef struct
{
    float position_des;   // rad
    float velocity_des;   // rad/s
    float torque_des;     // N*m
    float kp;
    float kd;
} DMMotor_MIT_Config_s;

typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;

    DMMotor_MIT_Config_s mit_config;
    DMMotor_Send_s motor_send_mailbox;

    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);

/**
 * @brief 切换反馈的目标来源,如将角速度和角度的来源换为IMU(小陀螺模式常用)
 *
 * @param motor 要切换反馈数据来源的电机
 * @param loop  要切换反馈数据来源的控制闭环
 * @param type  目标反馈模式
 */
void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type);

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_task内通过osDelay()确定控制频率
 */
void DMMotorControl();

/**
 * @brief 该函数作为外部接口传入MIT参数
 */



/**
 * @brief 原控制函数，每个电机有独立的 RTOS 任务（DMMotorTask），通过 DMMotorControlInit() 为每个电机创建线程，
 *        任务内直接发送 CAN 报文。
 *        是多任务并发模式。
 */
/*
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
*/
#endif // !DMMOTOR