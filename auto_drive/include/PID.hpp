class PID
{
private:
    double now_error;//当前误差
    double last_error;//上一次误差
    double sum_error;//误差积分
    
    double delta_error;//误差的误差
    double output_limit;//输出
public:
    double Kp, Ki, Kd;//Kp, Ki, Kd系数
    double delta_t = 0.02; // Sample time in seconds
    PID();
    ~PID();
    double PID_Calculate(double measured_value, double target_value);
    double Calculate_angle(double measured_value, double target_value);//角度控制
};
