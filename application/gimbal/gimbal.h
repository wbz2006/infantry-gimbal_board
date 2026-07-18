#ifndef GIMBAL_H
#define GIMBAL_H

typedef struct {
    float yaw_speed;   // 对应 α̇
    float pitch_speed; // 对应 β̇
} GimbalComp_t;

/**
 * @brief 初始化云台,会被RobotInit()调用
 * 
 */
void GimbalInit();

/**
 * @brief 云台任务
 * 
 */
void GimbalTask();

#endif // GIMBAL_H