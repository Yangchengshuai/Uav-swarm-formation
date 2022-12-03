#!/usr/bin/python
# -*- coding: UTF-8 -*-

import socket
import numpy
import threading

import rospy
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
# @Description: 接收来自主机的命令并处理;在飞机上运行

from Yformation_dict import formation_dict_3 as formation_dict


class Follower_uav(object):

    def __init__(self, num=3, uavid=0):
        self.id = uavid
        self.uav_num = num
        self.cmd_leader_avoid = numpy.zeros((self.uav_num, 3))
        self.leaderPose = numpy.zeros((1, 3))
        self.my_avoid = numpy.zeros((1, 3))
        # self.origin_formation = formation_dict["line"]#这是一个类似[[1 0 0] [2 0 0]]的数组
        self.pose = PoseStamped()
        self.cmd_vel_enu = TwistStamped()
        self.Kp = 0.35
        self.Kp_avoid = 2.0
        self.vel_max = 0.5
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=1)
        self.vel_enu_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)  #发布东北天下的控制命令;Twist是指的线速度和角速度
        self.arming_client_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_FCU_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    def pose_callback(self, msg):
        self.pose = msg  #无人机当前位置

    def ListtoArray(self, str_a):
        e = str_a.split(",")
        arr = list(map(float, e))
        f = numpy.array(arr)
        x = numpy.array(f).reshape(self.uav_num + 2, 3)
        return x

    #线程1：获取数据和命令
    def receive_cmd(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        PORT = 1115
        s.bind(('', PORT))
        print('Listening for broadcast at ', s.getsockname())
        while True:
            data, address = s.recvfrom(65535)
            self.cmd_leader_avoid = self.ListtoArray(data)
            self.leaderPose = self.cmd_leader_avoid[1]
            self.my_avoid = self.cmd_leader_avoid[self.id + 2]
            # print(self.cmd_leader_avoid)
        s.close()  #必须加这句，不加这句将导致端口不可重用

    def setLandMode(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
            isLanding = landService(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        except rospy.ServiceException, e:
            print "service land call failed: %s. The vehicle cannot land " % e

    def target_formation(self):
        if self.cmd_leader_avoid[0][0]==5:  #forward line
            print("leader forward")
            self.cmd_vel_enu.twist.linear.x = self.Kp * (1.6 - self.pose.pose.position.x)
            self.cmd_vel_enu.twist.linear.y = self.Kp * (0 - self.pose.pose.position.y)
            self.cmd_vel_enu.twist.linear.z = self.Kp * (1.1 - self.pose.pose.position.z)
        if self.cmd_leader_avoid[0][0]==6:  #back line
            print("leader back")
            
            self.cmd_vel_enu.twist.linear.x = self.Kp * (-0.5 - self.pose.pose.position.x) 
            self.cmd_vel_enu.twist.linear.y = self.Kp * (0 - self.pose.pose.position.y)
            self.cmd_vel_enu.twist.linear.z = self.Kp * (1.1 - self.pose.pose.position.z)
        if self.cmd_leader_avoid[0][0]==7:  #left
            print("leader left")
            
            self.cmd_vel_enu.twist.linear.x = self.Kp * (0 - self.pose.pose.position.x)
            self.cmd_vel_enu.twist.linear.y = self.Kp * (0.65 - self.pose.pose.position.y)
            self.cmd_vel_enu.twist.linear.z = self.Kp * (1.1 - self.pose.pose.position.z)
        if self.cmd_leader_avoid[0][0]==8:  #right
            print("leader right")
            
            self.cmd_vel_enu.twist.linear.x = self.Kp * (0 - self.pose.pose.position.x)
            self.cmd_vel_enu.twist.linear.y = self.Kp * (-0.5 - self.pose.pose.position.y)
            self.cmd_vel_enu.twist.linear.z = self.Kp * (1.1 - self.pose.pose.position.z)
        cmd_vel_magnitude = (self.cmd_vel_enu.twist.linear.x**2 + self.cmd_vel_enu.twist.linear.y**2 + self.cmd_vel_enu.twist.linear.z**2)**0.5
        if cmd_vel_magnitude > 3**0.5 * self.vel_max:
            self.cmd_vel_enu.twist.linear.x = self.cmd_vel_enu.twist.linear.x / cmd_vel_magnitude * self.vel_max
            self.cmd_vel_enu.twist.linear.y = self.cmd_vel_enu.twist.linear.y / cmd_vel_magnitude * self.vel_max
            self.cmd_vel_enu.twist.linear.z = self.cmd_vel_enu.twist.linear.z / cmd_vel_magnitude * self.vel_max
        self.vel_enu_pub.publish(self.cmd_vel_enu)

    #线程2：监控命令
    def cmd(self):
        rate = rospy.Rate(100)
        while True:
            list_set = [5,6,7,8]
            list_set = set(list_set)
            if self.cmd_leader_avoid[0][0] == 0:
                print("init")
            elif self.cmd_leader_avoid[0][0] == 1:
                print("takeoff")
                if (self.arming_client_srv(True)):
                    print("Vehicle arming succeed!")
                if (self.set_FCU_mode_srv(custom_mode='OFFBOARD')):
                    print("Vehicle offboard succeed!")
                else:
                    print("Vehicle offboard failed!")
                self.cmd_vel_enu.twist.linear.x = 1.2 * (0 - self.pose.pose.position.x)  #直接用线速度不知道是否可以 答：可以，下面加上了判断是否超过最大线速度
                self.cmd_vel_enu.twist.linear.y = 1.2 * (0 - self.pose.pose.position.y)
                self.cmd_vel_enu.twist.linear.z = 1.2 * (1.1 - self.pose.pose.position.z)
                self.vel_enu_pub.publish(self.cmd_vel_enu)  #发布东北天下的坐标系指令
                try:
                    rate.sleep()
                except:
                    continue

            elif self.cmd_leader_avoid[0][0] == 2:
                print("land")
                self.setLandMode()
            elif self.cmd_leader_avoid[0][0] == 3:
                print("line")
            elif self.cmd_leader_avoid[0][0] == 4:
                print("triangle")
            elif self.cmd_leader_avoid[0][0] in list_set:
                self.target_formation()
                try:
                    rate.sleep()
                except:
                    continue

    def process(self):
        #args是关键字参数，需要加上名字，写成args=(self,)
        th1 = threading.Thread(target=Follower_uav.receive_cmd, args=(self, ))
        th2 = threading.Thread(target=Follower_uav.cmd, args=(self, ))
        th2.start()
        th1.start()


if __name__ == '__main__':
    rospy.init_node('leader')
    commander = Follower_uav()
    commander.process()
