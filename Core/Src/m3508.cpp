#include "m3508.h"
#include "main.h"

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
