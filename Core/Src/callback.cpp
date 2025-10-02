#include "main.h"
#include "gpio.h"
#include "usart.h"

extern uint8_t rx_msg[4];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart7) {
        uint8_t tx_msg[4];
        for (int i = 0; i < 4; i++) {
            tx_msg[i] = rx_msg[i];
        }
        HAL_UART_Transmit_IT(&huart7,tx_msg,4);
        HAL_UART_Receive_IT(&huart7, rx_msg, 4);
    }
}