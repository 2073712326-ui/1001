//
// Created by qjy on 2026/10/11.
//

#ifndef CAN_PID_H
#define CAN_PID_H





class PID {
public:
    PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k);
    void reset();
    float calc(float ref,float fdb);


private:
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;
    float d_filter_k;
    float err_sum_;
    float previous_error;
    float previous_d;
};



#endif //CAN_PID_H
