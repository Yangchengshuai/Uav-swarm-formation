#!/usr/bin/python
# -*- coding: UTF-8 -*-

# @Description:
#   在飞机上执行
#   1.订阅本机的位置，通过UDP传动给控制端电脑 笔记本ip为：192.168.3.27 端口10000

import socket
import rospy
from geometry_msgs.msg import PoseStamped


class send():

    def __init__(self, uavId=0):
        self.id = uavId
        self.pose = PoseStamped()
        self.pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback, queue_size=100)

    #无人机当前位置
    def pose_callback(self, msg):
        self.pose = msg

    #发送线程，将本机数据发送给处理机
    def Send(self):
        rospy.init_node('follower1')
        rate = rospy.Rate(100)
        addr = ('192.168.3.27', 10000)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        while 1:
            data = str(self.pose.pose.position.x) + "," + str(self.pose.pose.position.y) + "," + str(self.pose.pose.position.z)  #raw_input()
            if not data:
                break
            s.sendto(data, addr)
            print "success send :", data
            try:
                rate.sleep()
            except:
                continue
        s.close()


if __name__ == '__main__':
    s = send(0)
    s.Send()
