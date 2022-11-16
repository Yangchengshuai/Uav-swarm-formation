#!/usr/bin/python
# -*- coding: UTF-8 -*-

import socket
import numpy
import threading
import rospy
import time

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import CommandBool, SetMode
from Yformation_dict import formation_dict_3 as formation_dict
from Yformation_dict import startPosition_3 as startPosition


class Follower_uav(object):

    def __init__(self, num=3, uavId=0):
        #无人机ID和编队飞机总数量
        self.id = uavId
        self.uav_num = num
        # key键盘命令
        self.key = 0

        #避障数组初始化
        self.vehicles_avoid_control = numpy.zeros(self.uav_num, 3)
        self.my_avoid = numpy.zeros(1, 3)
        # 期望编队数组初始化
        self.expect_formation = numpy.zeros(1, 3)  #是一个1*3数组

        #储存各无人机的位置信息
        self.pose = numpy.zeros(self.vehicle_num, 3)

        # 位置信息
        self.follower1_pose = PoseStamped()
        self.leader_Pose = PoseStamped()
        self.follower2_Pose = PoseStamped()

        self.cmd_vel_enu = TwistStamped()
        #飞行参数
        self.height=1.1 #起飞高度
        self.Kt=1.0 #起飞油门系数
        self.Kp = 0.35 #飞行响应系数
        self.Kp_avoid = 0.3
        self.vel_max = 0.5

        #ROS话题、服务
        self.follower1_pose_sub = rospy.Subscriber("/follower1/mavros/local_position/pose", PoseStamped, self.follower1_pose_callback, queue_size=1)
        self.leaderPose_sub= rospy.Subscriber("/leader/mavros/local_position/pose", PoseStamped, self.leaderPose_callback, queue_size=1)
        self.follower2_Pose_sub= rospy.Subscriber("/follower2/mavros/local_position/pose", PoseStamped, self.follower2_pose_callback, queue_size=1)
        self.vel_enu_pub = rospy.Publisher("/leader/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=1)  #发布东北天下的控制命令;Twist是指的线速度和角速度
        self.arming_client_srv = rospy.ServiceProxy("/leader/mavros/cmd/arming", CommandBool)
        self.set_FCU_mode_srv = rospy.ServiceProxy("/leader/mavros/set_mode", SetMode)

    def follower1_pose_callback(self, msg):
        self.follower1_pose = msg  #follower1当前位姿
    def leaderPose_callback(self, msg):
        self.leader_Pose = msg  #leader当前位姿
    def follower2_pose_callback(self, msg):
        self.follower2_pose = msg  #follower2当前位姿


    #线程1：获取任务命令
    def receive_cmd(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        PORT = 1115
        s.bind(('', PORT))
        print('Listening for broadcast at ', s.getsockname())
        while True:
            data, address = s.recvfrom(65535)
            self.key = int(data)  #字符串转整型
        s.close()  #必须加这句，不加这句将导致端口不可重用

    def setLandMode(self):
        rospy.wait_for_service('/leader/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/leader//mavros/cmd/land', CommandTOL)
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

        self.my_avoid = self.vehicles_avoid_control[self.id]
        self.cmd_vel_enu.twist.linear.x = self.cmd_vel_enu.twist.linear.x + self.Kp_avoid * self.my_avoid[0]
        self.cmd_vel_enu.twist.linear.y = self.cmd_vel_enu.twist.linear.y + self.Kp_avoid * self.my_avoid[1]
        self.cmd_vel_enu.twist.linear.z = self.cmd_vel_enu.twist.linear.z + self.Kp_avoid * self.my_avoid[2]
        cmd_vel_magnitude = (self.cmd_vel_enu.twist.linear.x**2 + self.cmd_vel_enu.twist.linear.y**2 + self.cmd_vel_enu.twist.linear.z**2)**0.5
        if cmd_vel_magnitude > 3**0.5 * self.vel_max:
            self.cmd_vel_enu.twist.linear.x = self.cmd_vel_enu.twist.linear.x / cmd_vel_magnitude * self.vel_max
            self.cmd_vel_enu.twist.linear.y = self.cmd_vel_enu.twist.linear.y / cmd_vel_magnitude * self.vel_max
            self.cmd_vel_enu.twist.linear.z = self.cmd_vel_enu.twist.linear.z / cmd_vel_magnitude * self.vel_max
        self.vel_enu_pub.publish(self.cmd_vel_enu)
    

    #计算避障数组
    def calculateAvoid(self):
        avoid_radius = 1.0
        aid_vec1 = [1, 0, 0]
        aid_vec2 = [0, 1, 0]
        # 对pose数组赋值，赋值的顺序，与输出的避障数组对应的
        self.pose[0] = [self.leader_Pose.pose.position.x,self.leader_Pose.pose.position.y,self.leader_Pose.pose.position.z] + startPosition["leader"]
        self.pose[1] = [self.follower1_pose.pose.position.x,self.follower1_pose.pose.position.y,self.follower1_pose.pose.position.z] + startPosition["follower1"]
        self.pose[2] = [self.follower2_pose.pose.position.x,self.follower2_pose.pose.position.y,self.follower2_pose.pose.position.z] + startPosition["follower2"]
        
        # 计算避障数组
        while not rospy.is_shutdown():
            for i in range(self.vehicle_num):  #对于每一架无人机
                position1 = self.pose[i]
                for j in range(1, self.vehicle_num - i):
                    position2 = self.pose[i + j]
                    dir_vec = position1 - position2
                    k = 1 - numpy.linalg.norm(dir_vec) / avoid_radius
                    if k > 0:  #这时候说明已经在危险区域了
                        cos1 = dir_vec.dot(aid_vec1) / (numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec1))  #dot表示点积；距离的x/半径值*1
                        cos2 = dir_vec.dot(aid_vec2) / (numpy.linalg.norm(dir_vec) * numpy.linalg.norm(aid_vec2))  #            距离的y/半径值*1
                        if abs(cos1) < abs(cos2):  #哪边更接近
                            avoid_control = k * numpy.cross(dir_vec, aid_vec1) / numpy.linalg.norm(numpy.cross(
                                dir_vec, aid_vec1))  #k*直线距离和aid的叉乘除以对应的范数，相当于归一化了；cross,返回两个向量的叉积,还是一个向量；向量的大小为这两个向量组成的平行四边形面积，夹角也可以算出来
                        else:
                            avoid_control = k * numpy.cross(dir_vec, aid_vec2) / numpy.linalg.norm(numpy.cross(dir_vec, aid_vec2))

                        self.vehicles_avoid_control[i] = numpy.array([
                            self.vehicles_avoid_control[i][0] + avoid_control[0], self.vehicles_avoid_control[i][1] + avoid_control[1],
                            self.vehicles_avoid_control[i][2] + avoid_control[2]
                        ])  #两个相近无人机其中一个朝正向
                        self.vehicles_avoid_control[i + j] = numpy.array([
                            self.vehicles_avoid_control[i + j][0] - avoid_control[0], self.vehicles_avoid_control[i + j][1] - avoid_control[1],
                            self.vehicles_avoid_control[i + j][2] - avoid_control[2]
                        ])  #两个相近无人机另一个朝反向
            time.sleep(0.05)


    #线程2：监控命令
    def cmd(self):
        rate = rospy.Rate(100)
        while True:
            if self.key == 0:
                print("init")
            elif self.key == 1:
                print("takeoff")
                if (self.arming_client_srv(True)):
                    print("Vehicle arming succeed!")
                if (self.set_FCU_mode_srv(custom_mode='OFFBOARD')):
                    print("Vehicle offboard succeed!")
                else:
                    print("Vehicle offboard failed!")
                self.cmd_vel_enu.twist.linear.x = self.Kt * (0 - self.pose.pose.position.x)  #直接用线速度不知道是否可以 答：可以，下面加上了判断是否超过最大线速度
                self.cmd_vel_enu.twist.linear.y = self.Kt * (0 - self.pose.pose.position.y)
                self.cmd_vel_enu.twist.linear.z = self.Kt * (self.height - self.pose.pose.position.z)
                self.vel_enu_pub.publish(self.cmd_vel_enu)  #发布东北天下的坐标系指令
                try:
                    rate.sleep()
                except:
                    continue

            elif self.key == 2:
                print("land")
                self.setLandMode()
            elif self.key == 3:
                print("line")
            elif self.key == 4:
                print("triangle")
            elif self.key in [5,6,7,8]:
                self.calculateAvoid()
                self.target_formation()
                self.vehicles_avoid_control = numpy.zeros((self.vehicle_num, 3))  #清空躲避数组
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
