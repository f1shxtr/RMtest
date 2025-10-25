#include "pid.h"
#include "main.h"
PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    out_max_ = out_max;
    d_filter_k_ = d_filter_k;
}

// 重置PID状态
void PID::reset(void) {
    ref_ = fdb_ = 0.0f;
    err_sum_ = err_ = last_err_ = 0.0f;
    pout_ = iout_ = dout_ = last_dout_ = 0.0f;
    output_ = 0.0f;
}

// 计算PID输出
float PID::calc(float ref, float fdb) {
    ref_ = ref;
    fdb_ = fdb;
    err_ = ref_ - fdb_;
    pout_ = kp_ * err_;
    err_sum_ += err_;
    if (err_sum_ > i_max_)
        err_sum_ = i_max_;
    else if (err_sum_ < -i_max_)
        err_sum_ = -i_max_;
    iout_ = ki_ * err_sum_;
    float derivative = err_ - last_err_;
    dout_ = last_dout_ + d_filter_k_ * (kd_ * derivative - last_dout_);
    output_ = pout_ + iout_ + dout_;
    if (output_ > out_max_)
        output_ = out_max_;
    else if (output_ < -output_)
        output_ = -output_;
    last_err_ = err_;
    last_dout_ = dout_;
    return output_;
}