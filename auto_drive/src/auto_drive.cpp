#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <auto_drive/PIDConfig.h>
#include <auto_drive/States.h>
#include "PID.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include<geographic_msgs/GeoPoint.h>
#include<geodesy/utm.h>
#include<geodesy/wgs84.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
enum AUTO{
    STOP=0,
    MANUAL,
    SPEED,
    HEADING,
    AUTOLINE,
};
std::string MODES[5]={"STOP", "MANUAL", "SPEED", "HEADING", "AUTOLINE"};
class Node {
public:
    Node(float linear_scaling, float angular_scaling, std::string boat_type)
        : linear_scaling(linear_scaling), angular_scaling(angular_scaling), boat_type(boat_type), mode(0) {
        // 初始化ROS发布者和订阅者
        if (boat_type == "single") {
            moddle_thrust_angle_pub = nh.advertise<std_msgs::Float32>("angle_cmd", 10);
            moddle_thrust_cmd_pub = nh.advertise<std_msgs::Float32>("thrust_cmd", 10);
            ROS_INFO("Initializing single-motor boat");
        } else if (boat_type == "double") {
            left_pub = nh.advertise<std_msgs::Float32>("left_cmd", 10);
            right_pub = nh.advertise<std_msgs::Float32>("right_cmd", 10);
            left_thrust_angle = nh.advertise<std_msgs::Float32>("left_angle", 10);
            right_thrust_angle = nh.advertise<std_msgs::Float32>("right_angle", 10);
            ROS_INFO("Initializing double-motor boat");
        } else {
            ROS_ERROR("Invalid boat type. Supported types: single, double");
        }
        //发布者 发布状态
        state_pub = nh.advertise<auto_drive::States>("euler_angles", 10);
        // 设置OpenCV窗口
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        // 订阅主题
        joy_sub = nh.subscribe("/joy", 10, &Node::joyCallback, this);//遥控器
        image_sub = nh.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 10, &Node::imageCallback, this);
        imu_sub = nh.subscribe("/wamv/sensors/imu/imu/data", 10, &Node::imuCallback,this);//imu姿态
        pose_sub = nh.subscribe("/wamv/sensors/gps/gps/fix",10,&Node::gpsCallback,this);//GPS原始数据
        velocity_sub = nh.subscribe("/wamv/sensors/gps/gps/fix_velocity",10,&Node::velCallback,this);//GPS原始数据
        //客户端
        wamv_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        
        wamv_state.request.model_name = "wamv";
        // 动态参数服务器
        dynamic_reconfigure::Server<auto_drive::PIDConfig>::CallbackType f;
        f = boost::bind(&Node::pidConfigCallback, this, _1, _2);
        server.setCallback(f);
    }
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        double Vx,Vy,Vz = 0;
        double latitude = 0, longitude = 0, altitude = 0;//经纬度高度
        double target_head,target_vel = 0;
        int MODE = 0;
        geographic_msgs::GeoPoint geo_point;
        geodesy::UTMPoint utm_point;
private:

    ros::NodeHandle nh;
    ros::Publisher state_pub;//发布无人艇的状态：位置与姿态角度
    ros::Publisher moddle_thrust_angle_pub, moddle_thrust_cmd_pub;//单个推进器船
    ros::Publisher left_pub, right_pub, left_thrust_angle, right_thrust_angle;//双推双桨船
    ros::Subscriber joy_sub, image_sub,imu_sub,pose_sub,velocity_sub;//订阅话题消息：遥控，图像，IMU姿态，GPS位置, 速度
    ros::ServiceClient wamv_client;//参数服务器获取无人艇的gazebo状态
    gazebo_msgs::GetModelState wamv_state;//无人艇的gazebo状态
    dynamic_reconfigure::Server<auto_drive::PIDConfig> server;

    std_msgs::Float32 thrust_angle_msg, thrust_cmd_msg;
    std_msgs::Float32 left_msg, right_msg, left_angle_msg, right_angle_msg;
    // 创建并发布角度消息
    auto_drive::States states_msg;

    float linear_scaling;
    float angular_scaling;
    std::string boat_type;
    std::string window_name = "Image Window";
    bool mode;
    std::vector<int> last_buttons;//存储上一次按键的值
    PID auto_head_pid,auto_vel_pid;

    void pidConfigCallback(auto_drive::PIDConfig &config, uint32_t level) {
        auto_head_pid.Kp = config.HeadKp;
        auto_head_pid.Ki = config.HeadKi;
        auto_head_pid.Kd = config.HeadKd;
        auto_vel_pid.Kp = config.VelKp;
        auto_vel_pid.Ki = config.VelKi;
        auto_vel_pid.Kd = config.VelKd;
        target_head = config.targethead;
        target_vel = config.targetvel;
        MODE = config.MODE;
        ROS_INFO("MODE=%d,targethead = %f,targetvel = %f", config.MODE,target_head,target_vel);
        ROS_INFO("VelKp=%f, VelKi=%f, VelKd=%f", config.VelKp, config.VelKi, config.VelKd);
        ROS_INFO("HeadKp=%f, HeadKi=%f, HeadKd=%f", config.HeadKp, config.HeadKi, config.HeadKd);
    }
    //解析GPS数据并转换坐标系
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        latitude = msg->latitude;//经度
        longitude = msg->longitude;//纬度
        altitude = msg->altitude;//高度
        geo_point.latitude = latitude;
        geo_point.longitude = longitude;
        geodesy::fromMsg(geo_point,utm_point);
        // ROS_INFO("UTM坐标系 : North=%f, East=%f, Zone=%c", utm_point.northing, utm_point.easting, utm_point.band);
    
        if (wamv_client.call(wamv_state))//参数状态
        {
            // ROS_INFO("WAMV Position: x = %f, y = %f, z = %f", wamv_state.response.pose.position.x, wamv_state.response.pose.position.y, wamv_state.response.pose.position.z);
            states_msg.x =  wamv_state.response.pose.position.x;
            states_msg.y =  wamv_state.response.pose.position.y;
            states_msg.z =  wamv_state.response.pose.position.z;
            // ROS_INFO("WAMV Orientation: x = %f, y = %f, z = %f, w = %f", wamv_state.response.pose.orientation.x, wamv_state.response.pose.orientation.y, wamv_state.response.pose.orientation.z, wamv_state.response.pose.orientation.w);
        }
    }
    //IMU四元数转 弧度角度
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Eigen::Quaterniond quaternion( msg->orientation.x, msg->orientation.y, msg->orientation.z,msg->orientation.w);
        // Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
        //         roll = euler[0]* 180.0 / M_PI;
        // pitch = euler[1]* 180.0 / M_PI;
        // yaw = euler[2]* 180.0 / M_PI;
        tf::Quaternion quaternion( msg->orientation.x, msg->orientation.y, msg->orientation.z,msg->orientation.w);
        tf::Matrix3x3 euler( quaternion ); // Roll, Pitch, Yaw
        euler.getRPY(roll,pitch,yaw);
        // states_msg.angles.push_back( yaw);
        // states_msg.angles.push_back( roll);
        // states_msg.angles.push_back( pitch);
        states_msg.yaw = yaw* 180.0 / M_PI;
        states_msg.pitch = pitch* 180.0 / M_PI;
        states_msg.roll = roll* 180.0 / M_PI;
        state_pub.publish(states_msg);
        // ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
    }
    void velCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
    {
        states_msg.vx = Vx = msg->vector.x;
        states_msg.vy = Vy = msg->vector.y;
        states_msg.vz = Vz = msg->vector.z;
    }
    void joyCallback(const sensor_msgs::Joy::ConstPtr& data) {
        if(MODE ==AUTO::MANUAL )
        {
        // 处理手柄数据 
        // # 0：左Y横  ，1：左X竖 ， 2：左按压 ；；；3：右Y横 ， 4：右X竖直  ，5：右按压 -1～1 default 1
        float lin_speed = data->axes[1] * linear_scaling;//油门
        float ang_speed = data->axes[3] * angular_scaling;//角度 -M_PI / 6.0
        std::vector<int> buttons = data->buttons;

        if (buttons[0] == 1 && last_buttons.size() > 0 && last_buttons[0] == 0) {
            mode = !mode;
            ROS_INFO("MANUAL Mode: %s", mode ? "start" : "stop");//ROS打印
        }//当前按键按下 上一次按键放开 模式切换

        
        pub_cmd(lin_speed,ang_speed);//推力分配
        // 存储上一次按键的值
        last_buttons = buttons;
        }
        else
        {
            autodrive();
        }
        // ROS_INFO("MODE: %s",MODES[MODE]);
    }
    void pub_cmd(double linspeed, double angspeed)
    {//推力分配与命令发布
        if (boat_type == "single") {//单桨船
            thrust_angle_msg.data = angspeed;
            thrust_cmd_msg.data = linspeed;
            if (mode) {
                ROS_INFO("Middle motor: %.2f, Rudder angle: %.2f", thrust_cmd_msg.data, thrust_angle_msg.data);
                moddle_thrust_angle_pub.publish(thrust_angle_msg);
                moddle_thrust_cmd_pub.publish(thrust_cmd_msg);
            }
        } else if (boat_type == "double") {//双桨船
            left_msg.data = linspeed + 0.6 * angspeed;
            right_msg.data = linspeed - 0.6 * angspeed;
            left_angle_msg.data = 0;
            right_angle_msg.data = 0;
            
            if (mode) {
                ROS_INFO("Left motor: %.2f, Right motor: %.2f", left_msg.data, right_msg.data);
                left_pub.publish(left_msg);
                right_pub.publish(right_msg);
                left_thrust_angle.publish(left_angle_msg);
                right_thrust_angle.publish(right_angle_msg);
            }
    }
    }
    void autodrive(void) {
    //autohead
    if(MODE ==AUTO::HEADING )//3
    {
        mode = 1;
        double output = auto_head_pid.Calculate_angle(yaw,target_head);
        if(output>M_PI / 6.0)
                output = M_PI / 6.0;
        if(output<-M_PI / 6.0)
                output = -M_PI / 6.0;
        double lin_speed = target_vel;
        double ang_speed = output;
        pub_cmd(lin_speed,ang_speed);
    }
    else if(MODE ==AUTO::SPEED )//2
    {
        mode = 1;
        double output = auto_vel_pid.Calculate_angle(Vx,target_vel);
        if(output>M_PI / 6.0)
                output = M_PI / 6.0;
        if(output<-M_PI / 6.0)
                output = -M_PI / 6.0;
        double lin_speed = output;
        double ang_speed = target_head;
        pub_cmd(lin_speed,ang_speed);
    }  
    else mode = 0;
    }
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        // 处理图像数据
        cv::Mat cv_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow(window_name, cv_img);
        cv::resizeWindow(window_name, 640, 360);  // 调整窗口大小
        cv::waitKey(1);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "auto_drive");
    setlocale(LC_CTYPE, "zh_CN.utf8");
    float linear_scaling = 1.0;
    float angular_scaling = -M_PI / 6.0;
    std::string boat_type = argv[1];
    ROS_INFO("Init %s boat",boat_type);

    Node node(linear_scaling, angular_scaling, boat_type);

    ros::spin();
    return 0;
}
