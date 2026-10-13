//
// Created by qjy on 2026/10/11.
//

#include "pid.h"

#include <cstdint>
#include <sys/select.h>
PID::PID(float kp, float ki, float kd, float i_max,float out_max,float d_filter_k) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->i_max = i_max;
    this->out_max = out_max;
    this->d_filter_k=d_filter_k;

}

void PID::reset() {
    previous_error=0;
    previous_d=0;
    err_sum_=0;
}

float PID::calc(float ref,float fdb) {
    float err=ref-fdb;
    float p_out = kp*err;

    err_sum_+=err;
    if (err_sum_>i_max) err_sum_=i_max;
    if (err_sum_<-i_max) err_sum_=-i_max;
    float i_out=ki*err_sum_;

    float raw_d=err-previous_error;
    previous_error=err;
    float d_out=d_filter_k * raw_d + previous_d * (1 - d_filter_k);
    previous_d=d_out;
    d_out *= kd;

    float output=p_out+d_out+i_out;
    if (output>out_max) output=out_max;
    if (output<-out_max) output=-out_max;
    return output;
}












