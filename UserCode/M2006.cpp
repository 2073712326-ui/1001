//
// Created by qjy on 2026/10/11.
//
#include "M2006.h"
#include <cmath>



M2006::M2006(uint8_t escid, float ratio, ControlMethod control_method):escid(escid),
ppid(0, 0, 0, 0, 0, 0),
spid(0, 0, 0, 0, 0, 0),
ratio_(ratio),
control_method_(control_method),
torque_constant_(0.18 * ratio /36.0f){
    ppid.reset();
    spid.reset();
}



void M2006::setPosition(float target_position, float feedforward_speed, float feedforward_intensity){
    control_method_=POSITION_SPEED;//双环控制模式
    target_angle_=target_position;//target_position里保存目标角度
    feedforward_speed_=feedforward_speed;//feedforward_speed里保存前向速度
    feedforward_intensity_=feedforward_intensity;//feedforward_intensity里保存前馈
}
void M2006::setSpeed(float target_speed, float feedforward_intensity){
    control_method_=SPEED;//速度控制模式
    target_speed_=target_speed;
    feedforward_intensity_=feedforward_intensity;
}
void M2006::setTorque(float torque) {
    control_method_=TORQUE;//力矩控制模式
    target_torque_=torque;
}

const uint8_t* M2006::getTXData(){//
    return tx_data_;
}

void M2006::packTxData() {//准备数据
    if (!std::isfinite(command_raw_)) command_raw_ = 0.f;
    if (command_raw_ > 10000.f) command_raw_ = 10000.f;
    if (command_raw_ < -10000.f) command_raw_ = -10000.f;//单位mA


    if (escid < 1 || escid > 8) return;
    const uint8_t index = static_cast<uint8_t>(((escid - 1) % 4 )* 2);
    const uint16_t raw = static_cast<uint16_t>(command_raw_);
    tx_data_[index] = static_cast<uint8_t>(raw >> 8);
    tx_data_[index + 1] = static_cast<uint8_t>(raw & 0xFF);
}

void M2006::canRxMSgCallback(const uint8_t rx_data[8]) {//解包函数
    const uint16_t ecd_angle =static_cast<uint16_t>((rx_data[0] << 8) | rx_data[1]);
    const int16_t speed_rpm = static_cast<int16_t>((rx_data[2]<<8)|rx_data[3]);
    const int16_t torque_raw = static_cast<int16_t>((rx_data[4]<<8)|rx_data[5]);

    ecd_angle_=ecd_angle * 360.f/kEncoderRange;
    speedRpm_=speed_rpm;
    feedback_torque_raw_ = torque_raw;

    if (!received_) {

        last_ecd_angle_ = ecd_angle_;
        received_ = true;
        return;
    }

    float delta_ecd_angle = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle> 180.f ) {
        delta_ecd_angle-=360.f;
    }else if (delta_ecd_angle<-180.f) {
        delta_ecd_angle+=360.f;
    }
    delta_angle_ = delta_ecd_angle/ ratio_;
    angle_+=delta_angle_;
    last_ecd_angle_ = ecd_angle_;



}
void M2006::handle() {
    if (stop_flag == true) {
        output_torque_=0.f;
        ppid.reset();
        spid.reset();
        packTxData();
        return;


    }


    fdb_angle_=angle_;
    fdb_speed_=speedRpm_;
    const float m = 0.150f; // 负载质量
    const float r = 0.065f; // 负载重心到输出轴距离
    const float g = 9.81f; // 重力加速度
    const float PI = 3.14159265358979323846f;
    float rad = fdb_angle_ * PI / 180.0f;//弧度制转换

    float feedforward_intensity_= m * r * g * sin(rad);//算出前馈

    target_speed_ = ppid.calc(target_angle_, fdb_angle_) + feedforward_speed_;

    output_torque_ = spid.calc(target_speed_, fdb_speed_)
                 + feedforward_intensity_;

    command_raw_ = target_torque_/0.18;

    packTxData();
}









