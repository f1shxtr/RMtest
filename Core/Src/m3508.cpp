#include "m3508.h"
#include "can.h"
#include "main.h"
#include <cmath>
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint32_t* pTxMailbox;
float linear_mapping(const int in, const int in_min, const int in_max, const float out_min, const float out_max) {
    return out_min + (in - in_min) * ((out_max - out_min) / (in_max - in_min));
};

void M3508Motor::can_rx_msg_callback(const uint8_t rx_data[8]) {
    last_ecd_angle_ = ecd_angle_;
    const auto ecd_angle = static_cast<uint16_t>((rx_data[0] << 8) | rx_data[1]);
    ecd_angle_ = linear_mapping(ecd_angle, 0, 8191, 0.0, 360.0);
    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ > 180.0) {
        delta_ecd_angle_ -= 360.0;
    } else if (delta_ecd_angle_ < -180.0) {
        delta_ecd_angle_ += 360.0;
    }
    delta_angle_ = delta_ecd_angle_ / ratio_;
    angle_ += delta_angle_;
    const auto rotate_speed = static_cast<int16_t>((rx_data[2] << 8) | rx_data[3]);
    rotate_speed_ = rotate_speed;
    const auto current = static_cast<int16_t>((rx_data[4] << 8) | rx_data[5]);
    current_ = linear_mapping(current, -16384, 16384, -20, 20);
    temp_ = rx_data[6];
}

void M3508Motor::SetPosition(float target_position, float feedforward_speed, float feedforward_intensity) {
    target_angle_ = target_position;
    feedforward_speed_ = feedforward_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = POSITION_SPEED;
}

void M3508Motor::SetSpeed(float target_speed, float feedforward_intensity) {
    target_speed_ = target_speed;
    feedforward_intensity_ = feedforward_intensity;
    control_method_ = SPEED;
}

void M3508Motor::SetIntensity(float intensity) {
    output_intensity_ = intensity;
    control_method_ = TORQUE;
}

void M3508Motor::handle() {
    feedforward_intensity_ = FeedforwardIntensityCalc(angle_);
    const float I_to_intensity = 16384.0f / 20.0f; // ≈819.2 intensity per A
    int16_t intensity = feedforward_intensity_ * I_to_intensity;
    tx_data[0] = (intensity >> 8) & 0xFF; // 高字节
    tx_data[1] = intensity & 0xFF; // 低字节
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, pTxMailbox);
    // 更新反馈值
    fdb_angle_ = angle_;
    fdb_speed_ = rotate_speed_;

    switch (control_method_) {
        case TORQUE:

            break;

        case SPEED:
            // 速度环 PID 计算
            output_intensity_ = spid_.calc(target_speed_ + feedforward_speed_, fdb_speed_) + feedforward_intensity_;
            break;

        case POSITION_SPEED:
            target_speed_ = ppid_.calc(target_angle_, fdb_angle_) + feedforward_speed_;
            output_intensity_ = spid_.calc(target_speed_, fdb_speed_) + feedforward_intensity_;
            break;
    }
}

float M3508Motor::FeedforwardIntensityCalc(float current_angle) {
    const float m = 0.5f; // kg
    const float g = 9.81f; // m/s^2
    const float L = 0.05524f; // m
    const float Kt = 0.3f; // N·m/A

    float torque = m * g * L * sinf(current_angle * 3.1415926f / 180.0f);

    float intensity = torque / Kt;

    return intensity;
}
