#include "motor_speed_controller.h"

static motor_speed_controller_param_t speed_ctrl_param;
static vfloat speed_error_integral[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static vfloat speed_error_last[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static vfloat speed_error[4] = {0.0f, 0.0f, 0.0f, 0.0f};

#define INTEGRAL_LIMIT (0.3f)  // 积分限幅

void motor_speed_controller_init(void)
{
    for(int i = 0; i < 4; i++) {
        speed_error_integral[i] = 0.0f;
        speed_error_last[i] = 0.0f;
        speed_error[i] = 0.0f;
    }
    
    // 默认参数（所有电机相同）
    for(int i = 0; i < 4; i++) {
        speed_ctrl_param.kp[i] = 0.01f;   // 比例增益，需要根据实际调整
        speed_ctrl_param.ki[i] = 0.005f;   // 积分增益
        speed_ctrl_param.kd[i] = 0.001f;  // 微分增益
    }
    speed_ctrl_param.max_output = 0.2f;   // 最大油门增量20%
    speed_ctrl_param.min_output = -0.2f;  // 最小油门增量-20%
}

void motor_speed_controller_set_param(motor_speed_controller_param_t *param)
{
    speed_ctrl_param = *param;
}

vfloat motor_speed_controller_update(uint8_t motor_index, vfloat speed_actual, vfloat speed_target, vfloat dt)
{
    if(motor_index >= 4 || dt <= 0.0f) return 0.0f;
    
    // 计算转速误差
    speed_error[motor_index] = speed_target - speed_actual;
    
    // 比例项
    vfloat p_term = speed_ctrl_param.kp[motor_index] * speed_error[motor_index];
    
    // 积分项
    speed_error_integral[motor_index] += speed_error[motor_index] * dt;
    // 积分限幅
    if(speed_error_integral[motor_index] > INTEGRAL_LIMIT) {
        speed_error_integral[motor_index] = INTEGRAL_LIMIT;
    } else if(speed_error_integral[motor_index] < -INTEGRAL_LIMIT) {
        speed_error_integral[motor_index] = -INTEGRAL_LIMIT;
    }
    vfloat i_term = speed_ctrl_param.ki[motor_index] * speed_error_integral[motor_index];
    
    // 微分项
    vfloat d_term = 0.0f;
    if(dt > 0.0f) {
        vfloat error_derivative = (speed_error[motor_index] - speed_error_last[motor_index]) / dt;
        d_term = speed_ctrl_param.kd[motor_index] * error_derivative;
        speed_error_last[motor_index] = speed_error[motor_index];
    }
    
    // PID输出
    vfloat output = p_term + i_term + d_term;
    
    // 输出限幅
    if(output > speed_ctrl_param.max_output) {
        output = speed_ctrl_param.max_output;
    } else if(output < speed_ctrl_param.min_output) {
        output = speed_ctrl_param.min_output;
    }
    
    return output;
}

void motor_speed_controller_reset(void)
{
    for(int i = 0; i < 4; i++) {
        speed_error_integral[i] = 0.0f;
        speed_error_last[i] = 0.0f;
        speed_error[i] = 0.0f;
    }
}

void motor_speed_controller_reset_motor(uint8_t motor_index)
{
    if(motor_index >= 4) return;
    speed_error_integral[motor_index] = 0.0f;
    speed_error_last[motor_index] = 0.0f;
    speed_error[motor_index] = 0.0f;
}

vfloat motor_speed_controller_get_error(uint8_t motor_index)
{
    if(motor_index >= 4) return 0.0f;
    return speed_error[motor_index];
}
