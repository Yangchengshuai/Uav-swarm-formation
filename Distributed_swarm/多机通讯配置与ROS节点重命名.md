## ROS多机通讯
1.  修改hosts文件。主机要把自己和所有从机名称和IP写入文件。从机只需要填写自己和主机的名字和IP![img](https://vipkshttps12.wiz.cn/editor/fa89fad0-2e2f-11ec-8668-3755f68e41df/f9e6daa1-46b8-4077-a0cd-248265e433f6/resources/jnO1layOONvqY6ztTTweKT-i-dT86ZPJ4ki_l68Hv9w.png?token=W.Iofm4D98QYyzahSWJqCPRks8Sxz-nqA_Y81HeUyoxu2ZFoxPxHVVPlDImGb5WaI)


2.  修改主从机bashrc文件

\#export ROS_HOSTNAME=本机设备名    

\#export ROS_MASTER_URI=http://主机设备名:11311  

export ROS_HOSTNAME=li-yuke      

export ROS_MASTER_URI=http://li-yuke:11311     



## ROS节点名称重名

参考：http://www.autolabor.com.cn/book/ROSTutorials/5/42-rosjie-dian-ming-cheng-zhong-ming.html

在ROS的网络拓扑中，是**不可以出现重名的节点**的，因为假设可以重名存在，那么调用时会产生混淆，这也就意味着，不可以启动重名节点或者同一个节点启动多次。

如何解决重命名？在ROS中给出的解决策略是**使用命名空间**或**名称重映射**。

**命名空间就是为名称添加前缀，名称重映射是为名称起别名**。这两种策略都可以解决节点重名问题，两种策略的实现途径有多种:

- rosrun 命令

- launch 文件

- 编码实现

以上三种途径都可以通过命名空间或名称重映射的方式，来避免节点重名

### launch文件设置命名空间与重映射

介绍 launch 文件的使用语法时，在 node 标签中有两个属性: name 和 ns，二者分别是用于实现名称重映射与命名空间设置的。使用launch文件设置命名空间与名称重映射也比较简单。

<launch>

   <node pkg="turtlesim" type="turtlesim_node" name="t1" />

   <node pkg="turtlesim" type="turtlesim_node" name="t2" />

   <node pkg="turtlesim" type="turtlesim_node" name="t1" ns="hello"/>

</launch>
