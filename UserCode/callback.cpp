//
// Created by qjy on 2026/10/3.
//
#include "M2006.h"
#include "can_user.h"
#include "Motor.hpp"
#include "tim.h"


extern CAN_HandleTypeDef hcan1;
static float gravityTorqueCalc (float output_angle)

M2006 motor(5,36.0f,POSITION_SPEED);
constexpr uint32_t kMotor1FeedbackId =0x202;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t data[8];
    if (hcan !=&hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,data) != HAL_OK) return;
    // if (rx_header.IDE == CAN_ID_STD&&rx_header.RTR == CAN_RTR_DATA&&rx_header.DLC == 8&&rx_header.StdId == kMotor1FeedbackId) {
    if (rx_header.StdId == 0x202){
    motor.canRxMSgCallback(data);
    }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {// 1kHz定时器中断
    if (htim->Instance !=TIM6) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) >0) {
        HAL_CAN_AddTxMessage(&hcan1,&tx_header,motor.getTXData(),&can_tx_mailbox);
    if (htim == htim6 )


        M2006 handle();
    }
}

