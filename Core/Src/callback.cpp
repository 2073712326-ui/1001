//
// Created by qjy on 2026/10/3.
//
#include "can_user.h"
#include "Motor.hp"


Motor motor (19.2f);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    motor.canRxMsgCallback(data);
}

uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan,CAN_RxHeaderTypeDef *rx_header,rx_data);
    if (hcan !=&hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data) != HAL_OK) return;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance !=TIM6) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel1(&hcan1) >0) {
        HAL_CAN_AddTxMessage(&hcan1,&tx_header,motor.getTxData(),&can_tx_mailbox);

    }
    motor.canRxMsgCallback(data);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {}