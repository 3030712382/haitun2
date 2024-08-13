#!/home/hp-t4/anaconda3/envs/aloha/bin/python
# -*- coding:UTF-8 -*-
# import sys
# if sys.version_info[0] < 3:
#     raise Exception("Please run this script with Python 3")

import rospy
from sensor_msgs.msg import Joy, Image, Imu, NavSatFix
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32
from auto_drive.cfg import PIDConfig
from auto_drive.msg import States
from dynamic_reconfigure.server import Server
from geographic_msgs.msg import GeoPoint
from geodesy.utm import fromMsg
from gazebo_msgs.srv import GetModelState
import math
from pyproj import Proj, transform
from math import pi
import numpy as np
import sys

is_python2 = sys.version_info[0] < 3

if is_python2:
    # Python 2-specific code
    print("Running Python 2")
    from tf.transformations import euler_from_quaternion#不适用于python3
else:
    # Python 3-specific code
    print("Running Python 3")

class Node:
    def __init__(self, linear_scaling, angular_scaling, boat_type):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.boat_type = boat_type
        self.mode = 0
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.bridge = CvBridge()
        self.cv_img = None#更新摄像头画面
        # Initialize ROS publishers and subscribers
        if self.boat_type == "single":
            self.moddle_thrust_angle_pub = rospy.Publisher("angle_cmd", Float32, queue_size=10)
            self.moddle_thrust_cmd_pub = rospy.Publisher("thrust_cmd", Float32, queue_size=10)
            rospy.loginfo("Initializing single-motor boat")
        elif self.boat_type == "double":
            self.left_pub = rospy.Publisher("left_cmd", Float32, queue_size=10)
            self.right_pub = rospy.Publisher("right_cmd", Float32, queue_size=10)
            self.left_thrust_angle = rospy.Publisher("left_angle", Float32, queue_size=10)
            self.right_thrust_angle = rospy.Publisher("right_angle", Float32, queue_size=10)
            rospy.loginfo("Initializing double-motor boat")
        else:
            rospy.logerr("Invalid boat type. Supported types: single, double")
            return
        self.last_buttons = []
        self.auto_head_pid = PID(0, 0, 0)
        #动态参数服务器
        self.srv = Server(PIDConfig, self.pid_config_callback)

        # 服务客户端
        self.wamv_client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # 状态消息
        self.states_msg = States()
        self.window_name = 'Image Window'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.startWindowThread()
        #发布者
        self.state_pub = rospy.Publisher("euler_angles", States, queue_size=10)
        #订阅
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self.image_sub = rospy.Subscriber("/wamv/sensors/cameras/front_right_camera/image_raw", Image, self.image_callback)
        self.imu_sub = rospy.Subscriber("/wamv/sensors/imu/imu/data", Imu, self.imu_callback)
        self.pose_sub = rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.gps_callback)


    def pid_config_callback(self, config, level):
        self.auto_head_pid.Kp = config.Kp
        self.auto_head_pid.Ki = config.Ki
        self.auto_head_pid.Kd = config.Kd
        rospy.loginfo("Reconfigure Request: Kp=%f, Ki=%f, Kd=%f" % (config.Kp, config.Ki, config.Kd))
        return config

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        utm_proj = Proj(proj="utm", zone=33, ellps="WGS84", south=False)
        utm_x, utm_y = utm_proj(self.longitude, self.latitude)
        # rospy.loginfo("UTM坐标系: East=%f, North=%f" % (utm_x, utm_y))
        try:
            response = self.wamv_client('wamv', '')
            # rospy.loginfo("WAMV Position: x = %f, y = %f, z = %f" % (
            #     response.pose.position.x, response.pose.position.y, response.pose.position.z))
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)



    def imu_callback(self, msg):
        orientation_q = msg.orientation

        (roll, pitch, yaw) = self.quaternion_to_euler(orientation_q)
        roll = roll * 180.0 / pi
        pitch = pitch * 180.0 / pi
        yaw = yaw * 180.0 / pi

        self.states_msg.yaw = yaw
        self.states_msg.pitch = pitch
        self.states_msg.roll = roll
        self.state_pub.publish(self.states_msg)

    def quaternion_to_euler(self, orientation_q):
        if is_python2:
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        else:
            q = np.array([orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])
            t0 = +2.0 * (q[3] * q[0] + q[1] * q[2])
            t1 = +1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])
            yaw = np.arctan2(t0, t1)

            t2 = +2.0 * (q[3] * q[1] - q[2] * q[0])
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch = np.arcsin(t2)

            t3 = +2.0 * (q[3] * q[2] + q[0] * q[1])
            t4 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
            roll = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw])
    def joy_callback(self, data):
        lin_speed = data.axes[1] * self.linear_scaling
        ang_speed = data.axes[3] * self.angular_scaling
        buttons = data.buttons

        if buttons[0] == 1 and self.last_buttons and self.last_buttons[0] == 0:
            self.mode = not self.mode
            rospy.loginfo("Mode: %s" % ("start" if self.mode else "stop"))
        self.pub_cmd(lin_speed, ang_speed)
        self.last_buttons = buttons

    def pub_cmd(self, lin_speed, ang_speed):
        if self.boat_type == "single":
            thrust_angle_msg = Float32()
            thrust_cmd_msg = Float32()
            thrust_angle_msg.data = ang_speed
            thrust_cmd_msg.data = lin_speed
            if self.mode:
                rospy.loginfo("Middle motor: %.2f, Rudder angle: %.2f" % (thrust_cmd_msg.data, thrust_angle_msg.data))
                self.moddle_thrust_angle_pub.publish(thrust_angle_msg)
                self.moddle_thrust_cmd_pub.publish(thrust_cmd_msg)
        elif self.boat_type == "double":
            left_msg = Float32()
            right_msg = Float32()
            left_angle_msg = Float32()
            right_angle_msg = Float32()
            left_msg.data = lin_speed + 0.6 * ang_speed
            right_msg.data = lin_speed - 0.6 * ang_speed
            left_angle_msg.data = 0
            right_angle_msg.data = 0
            if self.mode:
                rospy.loginfo("Left motor: %.2f, Right motor: %.2f" % (left_msg.data, right_msg.data))
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                self.left_thrust_angle.publish(left_angle_msg)
                self.right_thrust_angle.publish(right_angle_msg)

    # def image_callback(self, msg):
    #     print("Image callback")
    #     cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #     cv2.imshow(self.window_name, cv_img)
    #     cv2.resizeWindow(self.window_name, 640, 360)
    #     cv2.waitKey(1)
    def image_callback(self, msg):
        rospy.loginfo("Image callback")
        print("Image callback")
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            rospy.loginfo("width:%.3f,height:%.3f",self.cv_img.shape[0], self.cv_img.shape[1])
            if self.cv_img is not None:
                rospy.logwarn("Received image frame")
                # cv2.imshow(self.window_name, self.cv_img)
                cv2.imwrite("image.jpg", self.cv_img)
                if is_python2:#python2的时候可以imshow 
                    cv2.imshow(self.window_name, self.cv_img)
                    cv2.resizeWindow(self.window_name, 640, 360)
                    cv2.waitKey(1)
            else:
                rospy.logwarn("Received empty image frame")
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))

    def autodrive(self):
        output = self.auto_head_pid.PID_Calculate(self.yaw, 30)
        lin_speed = 0
        ang_speed = lin_speed + output
        self.pub_cmd(lin_speed, ang_speed)

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.sum_error = 0
        self.delta_t = 0.1

    def PID_Calculate(self, measured_value, target_value):
        now_error = measured_value - target_value
        delta_error = now_error - self.last_error
        self.last_error = now_error
        self.sum_error += now_error
        output = self.Kp * now_error + self.Ki * self.sum_error * self.delta_t + self.Kd * delta_error / self.delta_t
        return output

if __name__ == '__main__':
    rospy.init_node('auto_drive')
    linear_scaling = 1.0
    angular_scaling = -pi / 6.0
    boat_type = rospy.get_param("~boat_type", "double")
    rospy.loginfo("Init %s boat", boat_type)
    node = Node(linear_scaling, angular_scaling, boat_type)
    rospy.spin()
