
#!/usr/bin/python
# -*- coding: UTF-8 -*-

#!/usr/bin/env python
import socket
import threading
import numpy
import rospy
import time


class Commander(object):

    def __init__(self, num=3):
        self.cmd = 0
        self.vehicle_num = num
        self.vehicles_avoid_control = numpy.zeros((self.vehicle_num, 3))  #多维数组
        self.pose = numpy.zeros((self.vehicle_num, 3))  #[Vector3()] * self.vehicle_num

    # 0.278317540884,-1.25857508183,-0.802574515343
    def ListtoArray(self, str_a):
        e = str_a.split(",")
        arr = list(map(float, e))
        f = numpy.array(arr)
        return f

    #线程1：服务端,接收各个飞机的位置数据
    def Recv(self):
        address = ('192.168.3.27', 10000)
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(address)
        for i in range(0, self.vehicle_num):  #启动时初始位置（0，0，0）（0，1，0）（0，2，0）
            self.pose[i][1] = i
        while 1:
            data, addr = s.recvfrom(2048)
            if not data:
                break
            if addr[0] == '192.168.3.28':  #"0号ip" 这是leader机
                self.pose[0] = self.ListtoArray(data)
            elif addr[0] == '192.168.3.30':  #"1号ip":
                self.pose[1] = self.ListtoArray(data)
                self.pose[1][1] = self.pose[1][1] + 1  #对应的y值，转换为leader坐标系
            elif addr[0] == '192.168.3.29':  # "2号ip":
                self.pose[2] = self.ListtoArray(data)
                self.pose[2][1] = self.pose[2][1] -1  #对应的值，转换为leader坐标系
        s.close()

    #线程2：计算躲避数组。注意：所有飞机（0,0,0） 所以计算会出现nan（指不存在的数）
    def avoid(self):
        avoid_radius = 1.0
        aid_vec1 = [1, 0, 0]
        aid_vec2 = [0, 1, 0]
        for i in range(0, self.vehicle_num):  #启动时初始位置（0，0，0）（0，1，0）（0，2，0）
            self.pose[i][1] = i
        # self.init_position()
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
            self.Send()
            self.vehicles_avoid_control = numpy.zeros((self.vehicle_num, 3))  #躲避命令控制数组清空
            time.sleep(0.02)

    #线程3：获取键盘命令
    def get(self):
        while True:
            print("输入:0.quit  1.takeoff  2.land   3.line   4.triangle  5.leader forward 6.leader back 7.left move 8.right move")
            str = input("请输入：")
            self.cmd = str
            print "你输入的内容是: ", self.cmd

    #将array转换为字符串，例如[   1 2213    3 1234    5    6] 到  1,2213,3,1234,5,6
    def arrayToStr(self, data):
        b = data.flatten().tolist()
        d = [str(x) for x in b]  #number list to string list
        c = ','.join(d)
        return c

    def listToStr(self, list_a):
        d = [str(x) for x in list_a]  #number list to string list
        c = ','.join(d)  #list to str
        return c

    #广播避障数组和命令数组
    def Send(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        PORT = 1115
        network = '<broadcast>'
        data = str(self.cmd) + "," + "0" + "," + "0" + "," + self.listToStr(self.pose[0].tolist()) + "," + self.listToStr(
            self.vehicles_avoid_control.flatten().tolist())  #是一个一维数组字符串
        s.sendto(data.encode('utf-8'), (network, PORT))
        # print(data)
        s.close()

    def process(self):
        #args是关键字参数，需要加上名字，写成args=(self,)
        th1 = threading.Thread(target=Commander.Recv, args=(self, ))  #该线程获取每一架无人机的位置并赋值在对应数组 测试完成
        th2 = threading.Thread(target=Commander.avoid, args=(self, ))  #该线程计算出躲避速度和命令广播出去
        th3 = threading.Thread(target=Commander.get, args=(self, ))  #该线程键盘命令
        th1.start()
        th2.start()
        th3.start()


if __name__ == '__main__':
    commander = Commander()
    commander.process()