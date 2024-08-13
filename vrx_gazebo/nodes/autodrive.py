#!/usr/bin/env python
# -*- coding:UTF-8 -*-
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32
from math import pi
class Node():
    def __init__(self, linear_scaling, angular_scaling,boat_type = "single"):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.boat_type = boat_type
        # 初始化摄像头相关内容
        self.bridge = CvBridge()
            # 调整窗口大小
        self.window_name = 'Image Window'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.last_buttons = []#存储上一时刻的按键状态
        self.MODE = ["stop","start"]
        self.mode = 0 #模式
        if self.boat_type == "single":
            self.moddle_thrust_angle_pub = rospy.Publisher("angle_cmd", Float32, queue_size=10)
            self.moddle_thrust_cmd_pub = rospy.Publisher("thrust_cmd", Float32, queue_size=10)
            self.thrust_angle_msg = Float32()
            self.thrust_cmd_msg = Float32()
            rospy.loginfo("Initializing single-motor boat")
        elif self.boat_type == "double":
            self.left_pub = rospy.Publisher("left_cmd", Float32, queue_size=10)
            self.right_pub = rospy.Publisher("right_cmd", Float32, queue_size=10)
            self.left_thrust_angle = rospy.Publisher("left_angle", Float32, queue_size=10)
            self.right_thrust_angle = rospy.Publisher("right_angle", Float32, queue_size=10)
            self.left_msg = Float32()
            self.right_msg = Float32()
            self.left_angle_msg = Float32()
            self.right_angle_msg = Float32()
            rospy.loginfo("Initializing double-motor boat")
        else:
            rospy.logerr("Invalid boat type. Supported types: single, double")
            return

    def image_callback(self, msg):
        #摄像头图片显示
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = cv_img.shape[:2]  # 获取图像的高度和宽度
        rospy.loginfo("{},{}".format(height,width))
        cv2.resizeWindow(self.window_name, 640, 360)  # 设置窗口大小
        cv2.imshow(self.window_name, cv_img)
        cv2.waitKey(1)

    def callback(self, data):
        # rospy.loginfo("Received Joy data")

        # Assuming axes[1] is linear and axes[3] is angular control
        # 0：左Y横  ，1：左X竖 ， 2：左按压 ；；；3：右Y横 ， 4：右X竖直  ，5：右按压 -1～1 default 1
        lin_speed = data.axes[1] * self.linear_scaling #右X竖直 油门
        ang_speed = data.axes[3] * self.angular_scaling#右Y横
        Buttons = data.buttons#按键 按下为1 松开为0    0A 1B
        
        if Buttons[0] == 1 and self.last_buttons[0] == 0:#当前A按键按下 上一次没有按下
            self.mode = not self.mode
        rospy.loginfo(self.MODE[self.mode])
        if self.boat_type == "single":
            #单桨单舵
            self.thrust_angle_msg.data = ang_speed
            self.thrust_cmd_msg.data = lin_speed
            rospy.loginfo("Middle motor: {:.2f},Rudder angle: {:.2f}".format(self.thrust_cmd_msg.data, self.thrust_angle_msg.data))
            if self.MODE[self.mode] == "start":
                self.moddle_thrust_angle_pub.publish(self.thrust_angle_msg)
                self.moddle_thrust_cmd_pub.publish(self.thrust_cmd_msg)
        elif self.boat_type == "double":
            #双桨双舵
            self.left_msg.data = lin_speed + 0.6*ang_speed
            self.right_msg.data = lin_speed - 0.6*ang_speed
            self.left_angle_msg.data = 0
            self.right_angle_msg.data = 0
            # rospy.loginfo(f"Left motor: {self.left_msg.data}, Right motor: {self.right_msg.data}")
            rospy.loginfo("Left motor: {:.2f}, Right motor: {:.2f}".format(self.left_msg.data, self.right_msg.data))
            if self.MODE[self.mode] == "start":
                self.left_pub.publish(self.left_msg)
                self.right_pub.publish(self.right_msg)
                self.left_thrust_angle.publish(self.left_angle_msg)
                self.right_thrust_angle.publish(self.right_angle_msg)
        self.last_buttons = Buttons

if __name__ == '__main__':
    rospy.init_node('joy2drive', anonymous=True)
    linear_scaling = rospy.get_param('~linear_scaling', 1)
    angular_scaling = rospy.get_param('~angular_scaling', -pi/2.0)
    boat_type = rospy.get_param('~boat_type', "single")
    rospy.loginfo("boat_type:%s",boat_type)
    node = Node(linear_scaling, angular_scaling,boat_type)

    rospy.Subscriber("/joy", Joy, node.callback)
    rospy.Subscriber("/wamv/sensors/cameras/front_right_camera/image_raw", Image, node.image_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
