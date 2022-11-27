
# 无人机集群控制方案
## 集群控制简介
多机器人群体控制系统一般分为**集中式控制结构**和**分布式控制结构**。

![img](https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/contorlType.jpg)

**集中式控制结构：**
一个主控单元，集中掌握了环境中全局信息和所有机器人的信息，进行集中式处理任务与资源分配由主控单元合理分配给每一个机器人，每个机器人只需负责数据的输入和输出，数据的存储和控制处理由主控执行。结构较为简单，系统管理方便。
**缺点：**（1）计算复杂度增大，反应速度慢，机器人工作效率降低；（2)主控单元出现故障，会导致整个系统陷入瘫痪。

**分布式控制结构**：系统不存在控制与被控关系，机器人之间对等，互相信息交互。自主处理实时数据并根据数据规划出一条路径。分布式控制结构可以增加机器人的数目，灵活性较高，适用于动态环境下的工作空间。

**缺点**：缺乏全局时钟性，机器人之间的协调合作难，获取信息有限难以实现全局最优解。

**混合式控制结构**：一个主控单元控制所有机器人，获取机器人当前位置、环境信息、处理数据和制定策略等，为每一个机器人提供最新的全局信息，而每一个机器人采用分布式控制结构，都是一个独立平等的“个体”，与其他机器人实现信息的交流和互换，由主控单元获取的全局信息自主规划路径。混合式结构中即包含统一全局的中央管理模块，也采用分布式结构中每个成员机器人之间的通信方式。可以看出混合式结构集中了集中式与分布式结构的优点，这样的控制结构避免了成员机器人之间的任务冲突同时又加强了合作。

## 本系统所用方案优劣式介绍
**方案介绍**：本系统采用混合式集群控制模式，主控机接收处理所有无人机的状态信息，并且发送任务指令给到无人机。无人机分为leader机和follower机，leader机一架， follower机若干架。无人机接收主控机的任务指令，自主处理实时数据并根据数据规划出一条路径。

**优点**：本机的机载编队算法，具有效率高、节省资源等特点，能够为使用者节省更多的开销，更加快速使单体无人机达到目标点。通讯模块主要采用socket通讯，传输数据为字节级，传输数据可自定义，数据量小，传输数据时间短，性能高。避障模块采用基于球形几何体的编队避障算法，具有高效、低算力的特性，避障算法会对两个无人机施加一个弹簧的力，将两个无人机分离，再结合原始规划路径实现高灵敏避障。

**缺点**：避障模块只有机间避障，没有对障碍物的路径规划模块，需要后续进行优化。定位模块因为采用的是双目视觉，所以对光照变化和动态场景有一定的敏感性。


## 硬件方案
### 飞行平台介绍
飞行平台的硬件包括一个笔记本（主控机）和若干架无人机，无人机搭载的传感器包括：T265双目相机、JETSON Xavier NX板载计算机、深度相机D435i、WiFi模块，内置基于PX4-ROS的控制模块和SLAM、编队、避障等多种算法。适用于无GPS环境下基于视觉的无人机编队定位/导航/避障算法的验证与开发。



### 无人机集群编队
#### 通信硬件系统
目前机器人常用的通讯架构包括B/S，C/S，MQTT，ROS通讯等。Browser/Server，这个也是我们在日常生活当中经常用到一个服务架构模式，最常见的就是我们在浏览网站的时候，我们以Browser的身份向Server服务器发起访问，从而看到我们想看到的内容。而Client/Server中Socket套接字是C/S架构的实现方式，主要是TCP/UDP通讯，socket通信机制，可以应用于计算机不同进程之间通信，包括同一台计算机和网络中不同计算机之间的进程之间通信，结果就是支持分布在网络中的各个服务端/客户端的通信。MQTT（消息队列遥测传输协议），是一种基于发布/订阅模式的”轻量级”通讯协议，该协议构建于TCP/IP协 议上。ROS通讯包含的话题、服务、动作机制三种通讯方式。
本系统通讯采用WiFi无线通讯，主控机与无人机需要安装WiFi模块，在同一WiFi路由下进行socket字节流通讯和ROS话题、服务通讯。另外可以扩展添加UWB机间测距信号，用相对距离来保持编队队形。

#### 软件系统结构
![img](https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/Distributed_uav_swarm.png)

![img](https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/topic.png)

**软件运行流程**：

```shell
# startup.sh文件内容
gnome-terminal -x bash -c "source $HOME/mavros_ws/devel/setup.bash;roslaunch px4_realsense_bridge bridge.launch;exec bash"
sleep 3s
gnome-terminal -x bash -c  "roslaunch mavros px4.launch;exec bash"
sleep 3s
gnome-terminal -x bash -c  "cd /home/unionsys/Desktop;python follower_all.py;exec bash"
sleep 2s
gnome-terminal -x bash -c  "cd /home/unionsys/Desktop;python Yfollower1_300-3.py;exec bash"
wait
exit 0

```

1. 无人机开启t265定位模块

2. 无人机开启mavros通信模块，进行PX4飞控和ROS机的通信

3. 无人机开启多进程，发送自身位置坐标和等待接受主控机的指令信息。
4. 笔记本（主控机）开启键盘控制程序

 

**使用步骤：**

1.  设置IP地址，已便主控机与无人机进行通讯
2. 设置好编队飞行时期望的从机相对于主机的位置，可根据无人机的数量自定义编队形状

![\[外链图片转存失败,源站可能有防盗链机制,建议将图片保存下来直接上传(img-DSa8WJQg-1668588615413)(https://vipkshttps12.wiz.cn/editor/fa89fad0-2e2f-11ec-8668-3755f68e41df/d8957571-cc1d-4a95-8e64-92a8ae1d5cab/resources/8QT1SUulbA0YqNPNxXJgXwTU3ukjnPsr_IAR4AO7Tpo.png?token=W.dJxBp3XaPW50CsJoP0PwSuBIYjDNTAW7C1dSgTs4iBkLEGzdlbawu3fiO3rPhLk)\]](https://img-blog.csdnimg.cn/adf5d517d23d4ae7969e4d340dbc99d4.png)

3. 将无人机按照程序设定摆好位置，开机
4. 在无人机上运行startup.sh文件，启动通讯模块和定位模块
5. 将无人机调成position模式
6. 笔记本（主控机）上运行commander.py文件，开启键盘控制

#### 集群编队方案
**定位：**

T265是一个尺寸小、功耗低的完整的嵌入式SLAM解决方案，它将视觉特征与惯性测量单元 (IMU) 结合起来，使用视觉惯性里程计来计算其在3D空间中的方向和位置，亦即六自由度。T265直接输出位置信息，定位精度答厘米级。

根据定位方式的不同， 还可以拓展其他集群编队的方案，例如是GPS(RTK)编队、UWB编队和动捕编队等

**避障：**

在高速运动过程当中无人机可能会和其他无人机路线相交叉，一些无人机采取将本机的路线广播出去，其它无人机再针对每一架无人机做出机间避障路径，这是效率低的，并且会增加无人机算力消耗和增加编队网络中的数据传输量，导致延迟高，控制不灵敏等特点。机载算法搭载基于球形几何体的编队避障算法，每个无人机只需将其本机实时目标位置广播出去，其他无人机进入主体无人机球形几何体就会触发避障算法，避障算法会对两个无人机施加一个弹簧的力，将两个无人机分离，再结合原始规划路径实现高灵敏避障算法。

 

**通讯方式：**

![\[外链图片转存失败,源站可能有防盗链机制,建议将图片保存下来直接上传(img-C1ow7Mio-1668588615413)(https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/communication1.jpg)\]](https://img-blog.csdnimg.cn/bcf7cea2c3304f1f9c23a55348ab2410.jpeg)


主控机与无人机之间用socket创建UDP连接,无人机将自身位置数据利用UDP点对点的传给主控机进行处理，主控机将任务指令、leader位置信息和避障数组封装，采用广播的方式传给各无人机，无人机根据接受的信息进行相对于leader机坐标系下的编队任务飞行。

这种通讯系统，leader机与follower机之间没有直接通讯，而是通过笔记本主控机中转获得，即leader将位置信息发送给笔记本，笔记本将leader位置信息、计算出的避障数组、任务指令三种数据信息封装为数组广播出去，follower机接收广播内容进而得到leader机位置信息和任务指令等。具体代码参考：[socket通讯实现混合式集群编队](https://github.com/Yangchengshuai/Uav-swarm-formation/tree/main/script)

为什么要通过笔记本中转呢？而不是主从机直接ROS通讯呢？ROS是支持分布式运行节点，leader机与follower机可以用ROS节点的方式相互通信。第一点原因是采用socket通讯，传输数据为字节级，传输数据可自定义，数据量小，传输数据时间短，性能高。第二点原因是因为当时测试ROS节点通讯的时候，没有解决mavros话题重命名的问题，测试没有通过。



**拓展通讯方式：ROS通讯实现分布式集群编队**

![\[外链图片转存失败,源站可能有防盗链机制,建议将图片保存下来直接上传(img-pFZNJy9z-1668588615414)(https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/communication2.jpg)\]](https://img-blog.csdnimg.cn/202d268a8b564357a6e0a39a209c5a6d.jpeg)


目前的ROS1就是用中心点的通信方式，也就是说ROS Master是整个ROS节点的中心，组成一个星状网络结构，这个ROS Master是和其他节点的通信桥梁。有点不好的因素是ROS Mater这个中心节点出现异常，整个组网就失效了。

利用ROS分布式消息的模式，比如leader机与follower机发送指令，只需要加一个前缀/leader/**** 或者/follower/****,就可以读取或者发送控制指令到follower机无人机。ROS系统会维护一个所有无人机的消息指令队列(Topic),根据索引消息指令队列，就可以实现集群内部的所有无人机的传感器消息和控制指令的共享。

到目前为止已经解决mavros话题重命名的问题，完成了通讯测试，无人机全部数据由本机实时处理，实现分布式编队集群，实机编队测试正在进行中。 具体代码参考：[ROS通讯实现分布式集群编队](https://github.com/Yangchengshuai/Uav-swarm-formation/tree/main/Distributed)




## 仿真环境方案
参考[XTDrone](https://www.yuque.com/xtdrone/manual_cn)仿真环境搭建
无人机编队视频展示：[四旋翼集群编队](https://b23.tv/wfWW7J7)
参考文档链接：[集群仿真](https://www.yuque.com/u29056275/ut351y/hxszio69mwy5v4t5)


## 系统使用说明及功能描述
### 功能描述
 可实现编队变换、直线编队飞行、三角编队飞行、机间避障四大功能。

 

### 使用说明
1.  无人机起飞前的摆放位置是在程序中预先设定好的，可根据自己需要改动
2.  无人机需要保证在position模式下，才可以运行自动起飞程序，如果无法进入position模式，可以查看t265定位是否成功。
3.  无人机起飞前务必确保定位没有飘，可通过输出定位话题信息查看定位情况。
4.  程序中可调节Kp大小来调整无人机反应速度，后面可以将P调节优化为PD调节，或者如下图所示，更改位置误差与期望速度的函数关系为APM开方控制+限幅度，无人机控制会更加顺滑。

![\[外链图片转存失败,源站可能有防盗链机制,建议将图片保存下来直接上传(img-CD7zlksV-1668588615414)(https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/PV.jpeg)\]](https://img-blog.csdnimg.cn/71571c98af1c4524ab575809e6b4bedb.jpeg)


5. 无人机在自动启飞模式下无法切换为手动控制，如果想将无人机切换成手动控制需要将无人机切换到其他模式下方可手动控制飞行或降落。

6. 分布式集群代码已经进行优化，方便进行二次开发。详情可见：[Uav-swarm-formation](https://github.com/Yangchengshuai/Uav-swarm-formation/tree/main/Distributed)
7. 后面需要把路径规划模块加进去进行避障
8. 在本系统中ROS通讯的分布式集群的测试中发现，偶尔会出现接收不到任务指令的情况或者主机从机接收的指令存在时间差，初次分析是因为笔记本发送的指令采用的是socket广播的形式，所以无人机接受可能会发生丢失数据的情况。改进方向：要发送的任务指令不采用socket广播的形式，而是以一个ROS话题的形式进行发布，无人机通过订阅指令话题，接受任务指令。





## 集群编队飞行视频
[无人机集群编队飞行](https://www.bilibili.com/video/BV1jd4y1y7d8/?spm_id_from=333.337.search-card.all.click&vd_source=8c9644ebf9f2a4932abb6080dd498157)
### 直线编队飞行
![img](https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/output.gif)
### 三角编队飞行
![img](https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/output1.gif)

![img](https://github.com/Yangchengshuai/Uav-swarm-formation/blob/main/image/output2.gif)
