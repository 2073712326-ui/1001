//
// Created by qjy on 2026/10/11.
//

#ifndef CAN_M2006_H
#define CAN_M2006_H
#include "pid.h"
#include <cstdint>

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
    void setPosition(float target_position,float feedforward_speed,float feedforward_intensity);//保存目标位置
    void setSpeed(float target_speed,float feedforward_intensity);//保存目标速度
    void setIntensity(float intensity);//保存电机驱动控制量
    void setTorque(float torque);//保存输出力矩
    void canRxMSgCallback(const uint8_t rx_data[8]);//解包函数
    const uint8_t* getTXData();//收包
    bool stop_flag =false;


private:
    void packTxData();
    const uint8_t escid;//电调号
    PID ppid,spid;
    float target_angle_=0.0f,fdb_angle_= 0.0f;//目标角度、实际角度
    float target_speed_=0.0f,fdb_speed_=0.0f,feedforward_speed_=0.0f;//外环给的目标速度和电机读取的当前实际速度
    float target_torque_=0.0f;
    float output_torque_=0.0f,feedfoward_torque_=0.0f;
    ControlMethod_e control_method_ ;
    float feedforward_intensity_=0.0f,feedback_intensity_=0.0f;//前馈控制量、反馈控制量

    uint8_t esd_id_;
    const float ratio_;//减速比
    float angle_ = 0.0f; // 累计角度
    float delta_angle_ = 0.0f; // 角度变化量
    float ecd_angle_=0.0;//机械角度
    float speedRpm_=0.0;//转速
    float out_torque_=0.0;//转矩
    float last_ecd_angle_=0.0;//上一帧角度，用来求差 不带减速比
    static constexpr uint16_t kEncoderRange =8192;
    uint8_t tx_data_[8]={};//CAN发送缓冲区
    int16_t feedback_torque_raw_ = 0;
    float command_raw_=0.0f;
    bool received_=false;



};
#endif //CAN_M2006_H
