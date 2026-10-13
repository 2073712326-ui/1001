//
// Created by qjy on 2026/10/3.
//
#include <cstdint>

#include "stm32f4xx_hal_can.h"
#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H

#endif //CAN_MOTOR_H
class Motor {
public:
    explicit Motor(float ratio);
    void canRxMSgCallback(const uint8_t rx_data[8]);
    float angle()const;
    float speedRpm()const;
    float currentAmps()const;
    float remperature()const;
    float hasFeedback()const;
    void setTxCurrent(float amperes,uint8_t motor_id);
    uint8_t getTXData();
private:
    const float ratio_;//减速比
    float ecd_angle_=0.0;//机械角度
    float speedRpm_=0.0;//转速
    float currentA_=0.0;//转矩电流
    float tempC_=0.0;//温度
    float angle_=0.0;//输出轴累计角度
    float last_ecd_=0.0;//上一帧角度，用来求差
    bool received_ = false;//第一帧特殊处理
    uint8_t tx_data_[8];//待发送
    static constexpr uint16_t kEncoderRange =8192;
}







;