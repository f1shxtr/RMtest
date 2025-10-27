#include "can.h"
#include "gpio.h"
#include "m3508.h"
#include "main.h"
#include "tim.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
extern uint8_t stop_flag;
extern float target_angle;
extern float target_speed;
extern uint8_t stop_flag;
uint32_t* pTxMailbox;
M3508Motor Motor(3591 / 187);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    }
    if (rx_header.StdId == 0x201) {
        Motor.target_angle_ = target_angle;
        Motor.can_rx_msg_callback(rx_data);
        float feedforward_intensity = Motor.FeedforwardIntensityCalc(Motor.angle_);
        //Motor.SetPosition(target_angle,0.0f,feedforward_intensity);
        Motor.SetSpeed(target_speed, feedforward_intensity);
        Motor.handle();
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == htim6.Instance) {
        if (!stop_flag) {
            HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, pTxMailbox);
        }
        //Motor.SetIntensity(0);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == BUTTON_Pin_Pin) {
        stop_flag = !stop_flag;
    }
}
void poweroff() {
    Motor.SetIntensity(0);
    tx_data[0] = 0x00;
    tx_data[1] = 0x00;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, pTxMailbox);
}
