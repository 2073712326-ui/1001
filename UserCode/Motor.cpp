//
// Created by qjy on 2026/10/3.
//
#include "Motor.hpp"

//1.折8字节为字段（字节序+符号）
void Motor::canRxMSgCallback(const uint8_t rx_data[8]) {
    uint16_t ecd_angle =(rx_data[0]<<8)|rx_data[1];
    int8_t speed_rpm=(rx_data[2]<<8)|rx_data[3];
    int8_t currentA=(rx_data[4]<<8)|rx_data[5];
    uint8_t tempC=rx_data[6];


    //2.角度换算
    ecd_angle_ =ecd_angle*360/8192;
    speedRpm_ =speed_rpm;
    currentA_=currentA*20/16384;
    tempC_     = tempC;
    //3.环形

    if (received_ == false) {
        last_ecd_=ecd_angle;
        received_=true;
    }

    uint16_t diff=ecd_angle-last_ecd_;
    if (diff > (kEncoderRange /2)) {
        diff -= kEncoderRange;
    }else if (diff < -kEncoderRange / 2) {
        diff += kEncoderRange;
    }
    //4.
    float diff_angle=diff*360/kEncoderRange;
    angle_+=(diff_angle/ratio_);
}
void Motor::setTxCurrent(float amperes,uint8_t motor_id) {
    if (amperes > 20.0) {
        currentA_=20;
    }
    else if (amperes < -20.0) {
        currentA_=-20.0;
    }
    int16_t raw=static_cast<uint16_t>(amperes*16384/20);
    uint8_t index=static_cast<uint8_t>(motor_id-1)*2;
    tx_data_[index]= static_cast<uint8_t>(raw >> 8) & 0xFF;
    tx_data_[index+1] =static_cast<uint8_t> (raw & 0xFF);

}
uint8_t* Motor::getTXData(){
    return tx_data_;

}



