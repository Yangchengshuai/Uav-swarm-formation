# Uav-swarm-formation

# Note!
本次实验采用，三架多旋翼无人机，一台笔记本，实现了编队变换与编队飞行。采用状态机检测键盘指令的方式，控制无人机群实现编队变换和飞行。定位采用的是T265视觉定位，直接输出位置状态信息，此处可使用深度相机D435i进行为VIO（VINS-Fusion）定位，这种方案需要将摄像头中的同类无人机剔除掉，否则影响定位。笔记本与无人机间的采用Socket通讯。

# 直线编队飞行
![img](https://github.com/publicboyfriend/Uav-swarm-formation/blob/main/image/output.gif)


# 三角编队飞行
![img](https://github.com/publicboyfriend/Uav-swarm-formation/blob/main/image/output1.gif)
![img](https://github.com/publicboyfriend/Uav-swarm-formation/blob/main/image/output2.gif)

# 集群编队飞行视频
链接：https://www.bilibili.com/video/BV1jd4y1y7d8/?spm_id_from=333.337.search-card.all.click&vd_source=8c9644ebf9f2a4932abb6080dd498157
