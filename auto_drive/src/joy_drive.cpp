#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Float32.h>
#include <cmath>

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

        // 设置OpenCV窗口
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        // 订阅主题
        joy_sub = nh.subscribe("/joy", 10, &Node::joyCallback, this);
        image_sub = nh.subscribe("/wamv/sensors/cameras/front_right_camera/image_raw", 10, &Node::imageCallback, this);
    }

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& data) {
        // 处理手柄数据 
        // # 0：左Y横  ，1：左X竖 ， 2：左按压 ；；；3：右Y横 ， 4：右X竖直  ，5：右按压 -1～1 default 1
        float lin_speed = data->axes[1] * linear_scaling;//油门
        float ang_speed = data->axes[3] * angular_scaling;//角度
        std::vector<int> buttons = data->buttons;

        if (buttons[0] == 1 && last_buttons.size() > 0 && last_buttons[0] == 0) {
            mode = !mode;
        }//当前按键按下 上一次按键放开 模式切换

        ROS_INFO("Mode: %s", mode ? "start" : "stop");//ROS打印

        if (boat_type == "single") {//单桨船
            thrust_angle_msg.data = ang_speed;
            thrust_cmd_msg.data = lin_speed;
            ROS_INFO("Middle motor: %.2f, Rudder angle: %.2f", thrust_cmd_msg.data, thrust_angle_msg.data);
            if (mode) {
                moddle_thrust_angle_pub.publish(thrust_angle_msg);
                moddle_thrust_cmd_pub.publish(thrust_cmd_msg);
            }
        } else if (boat_type == "double") {//双桨船
            left_msg.data = lin_speed + 0.6 * ang_speed;
            right_msg.data = lin_speed - 0.6 * ang_speed;
            left_angle_msg.data = 0;
            right_angle_msg.data = 0;
            ROS_INFO("Left motor: %.2f, Right motor: %.2f", left_msg.data, right_msg.data);
            if (mode) {
                left_pub.publish(left_msg);
                right_pub.publish(right_msg);
                left_thrust_angle.publish(left_angle_msg);
                right_thrust_angle.publish(right_angle_msg);
            }
        }

        last_buttons = buttons;
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        // 处理图像数据
        cv::Mat cv_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::imshow(window_name, cv_img);
        cv::resizeWindow(window_name, 640, 360);  // 调整窗口大小
        cv::waitKey(1);
    }

    ros::NodeHandle nh;
    ros::Publisher moddle_thrust_angle_pub, moddle_thrust_cmd_pub;
    ros::Publisher left_pub, right_pub, left_thrust_angle, right_thrust_angle;
    ros::Subscriber joy_sub, image_sub;

    std_msgs::Float32 thrust_angle_msg, thrust_cmd_msg;
    std_msgs::Float32 left_msg, right_msg, left_angle_msg, right_angle_msg;

    float linear_scaling;
    float angular_scaling;
    std::string boat_type;
    std::string window_name = "Image Window";
    bool mode;
    std::vector<int> last_buttons;//存储上一次按键的值
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joy_drive");
    float linear_scaling = 1.0;
    float angular_scaling = -M_PI / 2.0;
    std::string boat_type = "single";

    Node node(linear_scaling, angular_scaling, boat_type);

    ros::spin();
    return 0;
}
