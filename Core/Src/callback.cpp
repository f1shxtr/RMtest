#include "can.h"
#include "gpio.h"
#include "m3508.h"
#include "main.h"
#include "tim.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint8_t tx_data[8];
extern uint8_t rx_data[8];
uint32_t* p_tx_mailbox;
M3508Motor motor(19.2);
void HAL_CAN_RxFifo0MsgPendingCallback(const CAN_HandleTypeDef* hcan) {
    if (hcan->Instance == CAN1) {
        HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);
    }
    if (rx_header.StdId == 0x204) {
        motor.can_rx_msg_callback(rx_data);
    }
}
void HAL_TIM_PeriodElapsedCallback(const TIM_HandleTypeDef* htim) {
    if (htim->Instance == htim6.Instance) {
        tx_data[7] = 0x00;
        HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, p_tx_mailbox);
    }
}