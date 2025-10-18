//
// Created by 嘉佳 on 2025/10/3.
//

#ifndef LIGHT_M3508_H
#define LIGHT_M3508_H
#include "PID.h"
#include "main.h"
class M3508Motor {
private:
    const float ratio_;

    float angle_ = 0.f;
    float delta_angle_ = 0.f;
    float ecd_angle_ = 0.f;
    float last_ecd_angle_ = 0.f;
    float delta_ecd_angle_ = 0.f;
    float rotate_speed_ = 0.f;
    float current_ = 0.f;
    float temp_ = 0.f;

    PID spid_, ppid_;
    float target_angle_, fdb_angle_;
    float target_speed_, fdb_speed_, feedforward_speed_;
    float feedforward_intensity_, output_intensity_;
    enum {
        TORQUE,
        SPEED,
        POSITION_SPEED,
    } control_method_;

public:
    explicit M3508Motor(const float ratio):
        ratio_(ratio),
        target_angle_(0.0f),
        fdb_angle_(0.0f),
        target_speed_(0.0f),
        fdb_speed_(0.0f),
        feedforward_speed_(0.0f),
        feedforward_intensity_(0.0f),
        output_intensity_(0.0f),
        control_method_(POSITION_SPEED) {
        spid_.reset();
        ppid_.reset();
    }
    void can_rx_msg_callback(const uint8_t rx_data[8]);
    void SetPosition(float target_position, float feedforward_speed, float feedforward_intensity);
    void SetSpeed(float target_speed, float feedforward_intensity);
    void SetIntensity(float intensity);
    void handle();
};
#endif