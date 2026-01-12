#pragma once
// 四旋翼电机动力分配器 20250710 Wakkk
#include "simple_math.h"

// 基础动力分配（无转速控制）
void mixer_update(vfloat roll, vfloat pitch, vfloat yaw, vfloat thrust, vfloat *output);

// 带转速控制的动力分配
// roll, pitch, yaw: 归一化三轴扭矩 -1~1
// thrust: 归一化油门 0~1
// speed_target: 目标转速数组 (RPM或ERPM) [4]
// speed_control_enable: 是否启用转速控制
// dt: 时间步长 (s)
// output: 电机归一化推力输出 0~1 [4]
void mixer_update_with_speed_control(
    vfloat roll, vfloat pitch, vfloat yaw, vfloat thrust,
    vfloat *speed_target, bool speed_control_enable, vfloat dt,
    vfloat *output
);
