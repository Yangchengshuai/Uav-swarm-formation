#!/usr/bin/python
# -*- coding: UTF-8 -*-
# @Author: chengshuai
# @Date: 2022-11.16

#!/usr/bin/env python
import socket
import threading
import numpy
import rospy
import time


class Commander(object):

    def __init__(self):
        self.cmd = 0
        
    #获取键盘命令
    def get(self):
        while True:
            print("指令列表: 0.quit  1.takeoff  2.land   3.line   4.triangle  5.leader forward 6.leader back 7.left move 8.right move")
            str = input("请输入：")
            self.cmd = str
            print "你输入的内容是: ", self.cmd
            #发送指令
            self.Send()
            time.sleep(0.02)

   

    #广播键盘命令
    def Send(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        PORT = 1115
        network = '<broadcast>'
        data = str(self.cmd)
        s.sendto(data.encode('utf-8'), (network, PORT))
        # print(data)
        s.close()

    # def process(self):
    #     #args是关键字参数，需要加上名字，写成args=(self,)
    #     th1 = threading.Thread(target=Commander.get, args=(self, ))  #该线程键盘命令
    #     th1.start()



if __name__ == '__main__':
    commander = Commander()
    commander.get()
