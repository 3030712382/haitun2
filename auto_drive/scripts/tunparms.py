# -*-encoding:utf-8-*-
import struct
import sys,time
from mainwindow import Ui_MainWindow
import serial
import serial.tools.list_ports
import matplotlib
import numpy as np
from PyQt5 import QtCore
from PyQt5.Qt import *
matplotlib.use("Qt5Agg")
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from PyQt5 import QtWidgets
from matplotlib.lines import Line2D
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5 import NavigationToolbar2QT as NavigationToolbar
from PyQt5.QtWidgets import (QWidget, QApplication, QComboBox, QLabel, QPushButton, QHBoxLayout,
                             QVBoxLayout, QToolTip, QMessageBox)
from PyQt5.QtGui import (QIcon, QFont)
import threading
import rospy
import rosgraph
from auto_drive.msg import States
from pid import PID#控制
# import matplotlib as mpl
# print(mpl.get_cachedir())
from record_data import hdf5data
class Mplplot(FigureCanvas):
    def __init__(self, parent=None, width=5, height=3, dpi=100):
        # normalized for 中文显示和负号
        plt.rcParams['font.sans-serif'] = ['SimHei']
        plt.rcParams['axes.unicode_minus'] = False
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        super(Mplplot, self).__init__(self.fig)
        self.setParent(parent)
        self.axes = self.fig.add_subplot(111) # 111 表示 1 行 1 列，第一张曲线图
        FigureCanvas.setSizePolicy(self, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(self)
    def points(self,x_data, y_data):
        plt.scatter(x_data, y_data, marker='.', color='g',
                    label='男性', alpha=0.5)
        # plt.scatter(male_athletes['Height'], male_athletes['Weight'], s=male_athletes['Age'], marker='^', color='g',
        #             label='男性', alpha=0.5)
        # plt.scatter(female_athletes['Height'], female_athletes['Weight'], color='r', alpha=0.5,
        #             s=female_athletes['Age'], label='女性')

    def setline(self, x_data, y_data,y2_data = None):
        self.line = Line2D(x_data, y_data,label='当前值',color='b')  # 绘制 2D 折线图
        if y2_data !=None:
            self.line2 = Line2D(x_data, y2_data,label='目标值',color='r')  # 绘制 2D 折线图
            self.axes.grid(True)  # 添加网格
            self.axes.set_title('动态曲线 可视化')  # 设置标题
            # 设置 xy 轴最大最小值 , 找到 x_data, y_data 最大最小值
            self.axes.set_xlim(np.min(x_data), np.max(x_data))
            MINy = min(np.min(y_data),np.min(y2_data))
            MAXy = max(np.max(y_data),np.max(y2_data)) + 10
            print((MINy, MAXy))
            self.axes.set_ylim(MINy, MAXy)  # y 轴稍微多一点，会好看一点
            self.axes.set_xlabel('时间')  # 设置坐标名称
            self.axes.set_ylabel('中线值')
            # 在曲线下方填充颜色
            # self.ax.fill_between(x_data, y_data, color='g', alpha=0.1)
            # self.ax.legend([self.line], ['sinx'])  # 添加图例
            # ------------------------------------------------------#
            self.axes.add_line(self.line)
            self.axes.add_line(self.line2)
            self.axes.legend(loc = 'upper right')
        else:
            # ------------------ 调整折线图基本样式 ---------------------#
            # self.line.set_ls('--')  # 设置连线
            # self.line.set_marker('*') # 设置每个点
            # self.line.set_color('red')  # 设置线条颜色
            self.axes.grid(True)  # 添加网格
            self.axes.set_title('动态曲线 可视化')  # 设置标题
            # 设置 xy 轴最大最小值 , 找到 x_data, y_data 最大最小值
            self.axes.set_xlim(np.min(x_data), np.max(x_data))
            self.axes.set_ylim(np.min(y_data), np.max(y_data) + 5)  # y 轴稍微多一点，会好看一点
            self.axes.set_xlabel('时间')  # 设置坐标名称
            self.axes.set_ylabel('中线值')
            # 在曲线下方填充颜色
            # self.ax.fill_between(x_data, y_data, color='g', alpha=0.1)
            # self.ax.legend([self.line], ['sinx'])  # 添加图例
            # ------------------------------------------------------#
            self.axes.add_line(self.line)
            self.axes.legend(loc = 'upper right')

class MyMainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self,parent=None):
        super(MyMainWindow,self).__init__(parent)#初始化父类
        self.setupUi(self)#继承Ui_MainWindow
        self.port_list = 0
        self.is_serial_open = False
        self.move = False
        self.mode = 0
        self.MODES = ["停止模式","遥控模式","航向模式","航速模式","模仿模式"]
        self.pbMode.setText(self.MODES[self.mode])
        self.speed = 0
        self.angle = 0
        self.auto_head_pid = PID(0.3, 0, 0.001)
        self.auto_speed_pid = PID(0, 0, 0)

        self.maxout = 0
        self.minout = 0
        self.targetxx = 180
        self.nowxx = 0
        
        
        # self.__tail = 0x0A  # tail
        # 数据协议标识帧

        #绘图
        # # # 加载相位振动波形
        self.phase_fig = Mplplot(width=5, height=3, dpi=72)
        self.fig_ntb = NavigationToolbar(self.phase_fig, self)
        self.gridlayout10 = QGridLayout(self.showlb)
        self.gridlayout10.addWidget(self.phase_fig)
        self.gridlayout10.addWidget(self.fig_ntb)
        self.verticalLayout.addLayout(self.gridlayout10)

        # # 准备数据，绘制曲线
        self.y_data = [self.nowxx]#list(np.arange(-10, 10, 0.1))
        self.x_data = [0]#list(np.sin(self.y_data)*180)#list(np.arange(-10, 10, 0.1))
        self.y2_data = [self.targetxx]#[self.targetxx]
        self.phase_fig.setline(self.x_data, self.y_data,self.y2_data)
        # 数据显示线程
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.showpig)
        # self.timer.start(0)
        self.timer.blockSignals(False)
        # rospy.init_node('plot_topics')
        self.populate_topics()#查询话题
        self.subscriber = None
    def showpig(self):
        # self.x_data.pop(0)
        self.y_data.append(self.nowxx)
        self.nowline.setText(str(round(self.nowxx,2)))#_translate("MainWindow", "开始")
        self.x_data.append(len(self.y_data)-1)
        self.y2_data.append(self.targetxx)
        self.phase_fig.line.set_xdata(self.x_data)  # 更新数据
        self.phase_fig.line.set_ydata(self.y_data)  # 更新数据
        try:
            self.phase_fig.line2.set_xdata(self.x_data)  # 更新数据
            self.phase_fig.line2.set_ydata(self.y2_data)  # 更新数据
            self.phase_fig.axes.set_xlim(np.min(self.x_data), np.max(self.x_data))
            self.phase_fig.axes.set_ylim(min(np.min(self.y_data),np.min(self.y2_data))-2, max(np.max(self.y2_data),np.max(self.y_data))+2)
            self.phase_fig.draw()  # 重新画图
            # print(self.x_data)
        except Exception as e:
            print(e)

    def get_serial_setting(self):#设置串口号与波特率
        self.ser.port = self.port
        self.ser.baudrate = int(self.bps)
    # 定义槽函数
    def populate_topics(self):#查询topics
        master = rosgraph.Master('/rostopic')
        topics = master.getPublishedTopics('')
        topiclist = []
        self.cbport.clear()
        for topic in topics:
            self.cbport.addItem(topic[0])
            topiclist.append(topic[0])
        if "/euler_angles" in topiclist:
            self.cbport.setCurrentText("/euler_angles")
        for bps in ["x","y","z","yaw","roll","pitch","vx","vy","vz","U"]:
            self.cbbps.addItem(bps)
        self.cbbps.setCurrentText("yaw")
    def update_plot(self,msg):#
        current = self.cbbps.currentText()
        if current=="x":
            self.nowxx = msg.x
        if current=="y":
            self.nowxx = msg.y
        if current=="z":
            self.nowxx = msg.z
        if current=="yaw":
            self.nowxx = msg.yaw
        if current=="roll":
            self.nowxx = msg.roll
        if current=="pitch":
            self.nowxx = msg.pitch
        if current=="vx":
            self.nowxx = msg.vx
        if current=="vy":
            self.nowxx = msg.vy
        if current=="vz":
            self.nowxx = msg.vz
        if current=="U":
            self.nowxx = msg.U
        self.showpig()
    def openport(self):#
        topic_name = self.cbport.currentText()
        if self.subscriber:
            self.subscriber.unregister()
        self.subscriber = rospy.Subscriber(topic_name, States, self.update_plot)
        self.is_serial_open = True  # 更新串口状态
        self.pbOpenseri.setText(u'暂停')  # 更新按钮名称
        self.pbOpenseri.setStyleSheet("background-color: green; color: black;")
    def closeport(self):#关闭串口
        # 更新串口状态
        self.is_serial_open = False
        self.pbOpenseri.setText(u'绘制')  # 更新串口打开按钮名称
        self.pbOpenseri.setStyleSheet("background-color: white; color: black;")
        if self.subscriber:
            self.subscriber.unregister()
    def open_close_button_handle(self):#处理打开或关闭串口按键事件
        # 判断串口是否已打开,没打开的话打开,打开的话关闭
        if self.is_serial_open:
            self.closeport()
        else:
            self.openport()
    def updatepid(self):#设定按钮 设置PID参数并下发
        if self.MODES[self.mode] == "航向模式":
            self.auto_head_pid.Kp = float(self.dsOutKpHeadHigh.value())
            self.auto_head_pid.Kd = float(self.dsOutKdHeadHigh.value())
            self.auto_head_pid.Ki = float(self.dsOutKiHeadHigh.value())
            self.auto_head_pid.clear_error()
        if self.MODES[self.mode] == "航速模式":
            self.auto_speed_pid.Kp = float(self.dsOutKpHeadHigh.value())
            self.auto_speed_pid.Kd = float(self.dsOutKdHeadHigh.value())
            self.auto_speed_pid.Ki = float(self.dsOutKiHeadHigh.value())
            self.auto_speed_pid.clear_error()
        self.maxout = float(self.dsOutMaxOutHeadHigh.value())
        self.minout = float(self.dsOutMinOutHeadHigh.value())
        self.targetxx = float(self.targetmiddleline.value())
        self.speed = float(self.Speed.value())
        self.angle = float(self.Rudder.value())
        print(self.MODES[self.mode])
        print("Kp: ",self.dsOutKpHeadHigh.value())
        print("Ki: ",self.dsOutKiHeadHigh.value())
        print("Kd: ",self.dsOutKdHeadHigh.value())
        print("Maxout: ",self.dsOutMaxOutHeadHigh.value())
        print("Minout: ",self.dsOutMinOutHeadHigh.value())
        print("target : ", self.targetmiddleline.value())

    def setpid(self):#设定按钮 设置PID参数并下发
        self.speed = float(self.Speed.value())
        self.angle = float(self.Rudder.value())
        self.kp = float(self.dsOutKpHeadHigh.value())
        self.ki = float(self.dsOutKiHeadHigh.value())
        self.kd = float(self.dsOutKdHeadHigh.value())
        self.maxout = float(self.dsOutMaxOutHeadHigh.value())
        self.minout = float(self.dsOutMinOutHeadHigh.value())
        self.targetxx = float(self.targetmiddleline.value())
        print("Kp: ",self.dsOutKpHeadHigh.value())
        print("Ki: ",self.dsOutKiHeadHigh.value())
        print("Kd: ",self.dsOutKdHeadHigh.value())
        print("Maxout: ",self.dsOutMaxOutHeadHigh.value())
        print("Minout: ",self.dsOutMinOutHeadHigh.value())
        print("target line: ", self.targetmiddleline.value())
        return
    def clearline(self):#保存按钮 打印PID参数
        self.y_data = [self.nowxx]#list(np.arange(-10, 10, 0.1))
        self.x_data = [0]#list(np.sin(self.y_data)*180)#list(np.arange(-10, 10, 0.1))
        self.y2_data = [self.targetxx]#[self.targetxx]
        self.showpig()
        return
    def carrun(self,):
        #向小车发送速度speed与角度angle以及模式
        print(1)
        # 根据角度进行转向
    def send1(self, angle, idx):
        # 单个舵机运动
        print("send ok")
    def switchmode(self):
        self.mode += 1
        if(self.mode>=5):
            self.mode = 0
        self.pbMode.setText(self.MODES[self.mode])
        self.speed = 0#停车
        self.angle = 0#舵角归中
        if self.MODES[self.mode] == "航向模式":
            self.targetmiddleline.setMaximum(180.0)
            self.targetmiddleline.setMinimum(-180.0)
            self.targetmiddleline.setSingleStep(1.0)
            self.Speed.setStyleSheet("background-color: green; color: black;")
            self.Rudder.setStyleSheet("background-color: white; color: black;")
        elif self.MODES[self.mode] == "航速模式":
            self.targetmiddleline.setMaximum(5.0)
            self.targetmiddleline.setMinimum(-5.0)
            self.targetmiddleline.setSingleStep(0.1)
            self.Rudder.setStyleSheet("background-color: green; color: black;")
            self.Speed.setStyleSheet("background-color: white; color: black;")  
        else:
            self.Rudder.setStyleSheet("background-color: white; color: black;")
            self.Speed.setStyleSheet("background-color: white; color: black;")  
    def debugrudder(self):
        if self.MODES[self.mode] == "调试模式":
            self.speed = float(self.Speed.value())
            self.angle = float(self.Rudder.value())
            # print(self.angle)
    def startmove(self):
        if self.MODES[self.mode]=="调试模式":
            self.speed = float(self.Speed.value())
            self.angle = float(self.Rudder.value())
            #print(self.angle)
        else:
            self.move = ~(self.move)
            if(self.move):
                self.pbstartstop.setText(u'停止')  # 更新按钮名称
                self.pbstartstop.setStyleSheet("background-color: green; color: black;")
                self.speed = float(self.Speed.value())
                self.angle = float(self.Rudder.value())
                # if self.timer.isActive():
                #     print("定时器")
                # else:
                    # self.timer.start(0)
            else:
                self.pbstartstop.setText(u'开始')  # 更新按钮名称
                self.pbstartstop.setStyleSheet("background-color: white; color: black;")
                self.speed = 0
                self.angle = 0
                # if self.timer.isActive():
                #     self.timer.stop()
if __name__ == '__main__':
    app = QApplication(sys.argv)  # 创建应用程序对象
    MainWindow = MyMainWindow()  # 创建主窗口
    MainWindow.show()  # 显示主窗口

    # th1 = threading.Thread(target=MainWindow.showpig())  # 串口读取线程
    # th1.start()
    sys.exit(app.exec_())  # 在主线程中退出
