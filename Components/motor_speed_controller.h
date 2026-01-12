#pragma once
// 电机转速控制器 20250115
#include "system.h"
#include "simple_math.h"

typedef struct {
    vfloat kp[4];      // 比例增益（每个电机独立）
    vfloat ki[4];      // 积分增益
    vfloat kd[4];      // 微分增益
    vfloat max_output;  // 最大输出限制（归一化）
    vfloat min_output;   // 最小输出限制（归一化）
} motor_speed_controller_param_t;

// 初始化转速控制器
void motor_speed_controller_init(void);

// 设置控制器参数
void motor_speed_controller_set_param(motor_speed_controller_param_t *param);

// 更新转速控制器
// motor_index: 电机索引 0-3
// speed_actual: 实际转速 (RPM或ERPM，根据电调反馈格式)
// speed_target: 目标转速 (RPM或ERPM)
// dt: 时间步长 (s)
// 返回: 油门增量 (-1.0 ~ 1.0)
vfloat motor_speed_controller_update(uint8_t motor_index, vfloat speed_actual, vfloat speed_target, vfloat dt);

// 重置所有电机的积分项
void motor_speed_controller_reset(void);

// 重置指定电机的积分项
void motor_speed_controller_reset_motor(uint8_t motor_index);

// 获取当前转速误差
vfloat motor_speed_controller_get_error(uint8_t motor_index);
