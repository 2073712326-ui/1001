//
// Created by qjy on 2026/10/11.
//

#ifndef CAN_M2006_H
#define CAN_M2006_H
#include "pid.h"
#include "stm32f4xx_hal.h"

typedef enum ControlMethod
{
    POSITION_SPEED,
    SPEED,
    TORQUE,
}ControlMethod_e;//控制模式

class M2006
{
public:
    M2006(uint8_t escid,float ratio,ControlMethod control_method);
    void handle();//handle循环函数
    void setPosition(float target_position,float feedforward_speed,float feedforward_intensity);//保存目标角度
    void setSpeed(float target_speed,float feedforward_intensity);//保存目标速度
    void setIntensity(float intensity);//保存电机驱动控制量
    void setTorque(float torque);//保存输出力矩
    void canRxMSgCallback(const uint8_t rx_data[8]);//收包
    bool stop_flag = true;


private:
    const uint8_t escid;//电调号
    PID ppid,spid;
    float target_angle_=0.0f,fdb_angle_= 0.0f;//
    float target_speed_=0.0f,fdb_speed_=0.0f,feedforward_speed_=0.0f;
    float target_torque_=0.0f;
    float output_torque_=0.0f,feedfoward_torque_=0.0f;
    ControlMethod_e control_method_ ;
    float feedforward_intensity_=0.0f,feedback_intensity_=0.0f;

    uint8_t esd_id_;
    const float ratio_;//减速比
    float angle_ = 0.0f; // 累计角度
    float delta_angle_ = 0.0f; // 角度变化量
    float ecd_angle_=0.0;//机械角度
    float speedRpm_=0.0;//转速
    float out_torche_=0.0;//转矩
    float last_ecd_angle_=0.0;//上一帧角度，用来求差 不带减速比
    static constexpr uint16_t kEncoderRange =8192;




};
#endif //CAN_M2006_H
