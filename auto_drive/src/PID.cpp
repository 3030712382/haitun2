#include"PID.hpp"

PID::PID() : Kp(0), Ki(0), Kd(0) ,last_error(0.0), sum_error(0.0), delta_error(0.0), output_limit(0.0){}
PID::~PID(){}
double PID::PID_Calculate(double measured_value, double target_value) {
    
    now_error = measured_value - target_value;

    delta_error =  now_error  - last_error;
    last_error = now_error;
    sum_error += now_error;
    output_limit = Kp*now_error + Ki*sum_error*delta_t+ Kd*delta_error/delta_t;
    return output_limit;
}