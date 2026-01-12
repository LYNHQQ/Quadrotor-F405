#pragma once
// 高度控制器 20250115
#include "system.h"
#include "simple_math.h"

typedef struct {
    vfloat kp;      // 比例增益
    vfloat ki;      // 积分增益
    vfloat kd;      // 微分增益
    vfloat max_output;  // 最大输出限制
    vfloat min_output;   // 最小输出限制
} altitude_controller_param_t;

// 初始化高度控制器
void altitude_controller_init(void);

// 设置控制器参数
void altitude_controller_set_param(altitude_controller_param_t *param);

// 更新高度控制器
// altitude: 当前高度 (m)
// altitude_target: 目标高度 (m)
// dt: 时间步长 (s)
// 返回: 油门增量 (-1.0 ~ 1.0)
vfloat altitude_controller_update(vfloat altitude, vfloat altitude_target, vfloat dt);

// 重置积分项
void altitude_controller_reset(void);

// 获取当前高度误差
vfloat altitude_controller_get_error(void);
