#include "altitude_controller.h"

static altitude_controller_param_t alt_ctrl_param;
static vfloat altitude_error_integral = 0.0f;
static vfloat altitude_error_last = 0.0f;
static vfloat altitude_error = 0.0f;

#define INTEGRAL_LIMIT (0.5f)  // 积分限幅

void altitude_controller_init(void)
{
    altitude_error_integral = 0.0f;
    altitude_error_last = 0.0f;
    altitude_error = 0.0f;
    
    // 默认参数
    alt_ctrl_param.kp = 0.5f;
    alt_ctrl_param.ki = 0.1f;
    alt_ctrl_param.kd = 0.2f;
    alt_ctrl_param.max_output = 0.3f;  // 最大油门增量30%
    alt_ctrl_param.min_output = -0.3f;  // 最小油门增量-30%
}

void altitude_controller_set_param(altitude_controller_param_t *param)
{
    alt_ctrl_param = *param;
}

vfloat altitude_controller_update(vfloat altitude, vfloat altitude_target, vfloat dt)
{
    if(dt <= 0.0f) return 0.0f;
    
    // 计算高度误差
    altitude_error = altitude_target - altitude;
    
    // 比例项
    vfloat p_term = alt_ctrl_param.kp * altitude_error;
    
    // 积分项
    altitude_error_integral += altitude_error * dt;
    // 积分限幅
    if(altitude_error_integral > INTEGRAL_LIMIT) {
        altitude_error_integral = INTEGRAL_LIMIT;
    } else if(altitude_error_integral < -INTEGRAL_LIMIT) {
        altitude_error_integral = -INTEGRAL_LIMIT;
    }
    vfloat i_term = alt_ctrl_param.ki * altitude_error_integral;
    
    // 微分项
    vfloat d_term = 0.0f;
    if(dt > 0.0f) {
        vfloat error_derivative = (altitude_error - altitude_error_last) / dt;
        d_term = alt_ctrl_param.kd * error_derivative;
        altitude_error_last = altitude_error;
    }
    
    // PID输出
    vfloat output = p_term + i_term + d_term;
    
    // 输出限幅
    if(output > alt_ctrl_param.max_output) {
        output = alt_ctrl_param.max_output;
    } else if(output < alt_ctrl_param.min_output) {
        output = alt_ctrl_param.min_output;
    }
    
    return output;
}

void altitude_controller_reset(void)
{
    altitude_error_integral = 0.0f;
    altitude_error_last = 0.0f;
    altitude_error = 0.0f;
}

vfloat altitude_controller_get_error(void)
{
    return altitude_error;
}
