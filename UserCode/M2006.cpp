//
// Created by qjy on 2026/10/11.
//
#include "M2006.h"
#include <cmath>

M2006::M2006(uint8_t escid, float ratio, ControlMethod control_method):escid(escid),ppid(0, 0, 0, 0, 0, 0),
     spid(0, 0, 0, 0, 0, 0),ratio_(ratio),control_method_(control_method){
    ppid.reset();
    spid.reset();


}
void M2006::setPosition(float target_position, float feedforward_speed, float feedforward_intensity){
    control_method_=POSITION_SPEED;//双环控制模式
    target_angle_=target_position;
    feedforward_speed_=feedforward_speed;
    feedforward_intensity_=feedforward_intensity;
}
void M2006::setSpeed(float target_speed, float feedforward_intensity){
    control_method_=SPEED;
    target_speed_=target_speed;
    feedforward_intensity_=feedforward_intensity;
}
void M2006::setTorque(float torque) {
    control_method_=TORQUE;
    target_torque_=torque;
}

void M2006::canRxMSgCallback(const uint8_t rx_data[8]) {
    uint16_t ecd_angle = static_cast<uint16_t>((rx_data[0]<<8)|rx_data[1]);
    int16_t speed_rpm = static_cast<int16_t>((rx_data[2]<<8)|rx_data[3]);
    int16_t output_torque = static_cast<int16_t>((rx_data[4]<<8)|rx_data[5]);
    last_ecd_angle_=ecd_angle;
    ecd_angle_=ecd_angle*360.f/8192.f;
    speedRpm_=speed_rpm;
    output_torque_=output_torque;

    float delta_angle_ecd = ecd_angle_ - last_ecd_angle_;
    if (delta_angle_ecd> 180.f ) {
        delta_angle_ecd+=360.f;
    }else if (delta_angle_ecd<-180.f) {
        delta_angle_ecd+=-360.f;
    }
    delta_angle_ = delta_angle_ecd / ratio_;
    angle_+=delta_angle_;

}
void set
void M2006::handle() {
    if (stop_flag == true) {
        out_torche_=0.f;
        ppid.reset();
        spid.reset();
        return;

    }
    fdb_angle_=ecd_angle_;
    fdb_speed_=speedRpm_;
    float rad = ecd_angle_ * M_PI / 180.0f;
    float feedforward_intensity_= kTorque * cos(rad);
    float target_speed_ =ppid.calc(target_angle_, fdb_angle_)+ feedforward_speed_;
    float intensity_float_ = spid.calc(target_speed_, fdb_speed_)+feedforward_intensity_;
    float fdb_intensity_=intensity_float_+feedforward_intensity_;
    output_torque_ = intensity_float_ + fdb_intensity_;
}









}