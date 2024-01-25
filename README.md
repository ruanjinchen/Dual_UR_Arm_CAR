# 介绍
本项目完成了自主设计底座模型组装的UR机械臂（双臂）的驱动控制。在Ubuntu 22.04 ROS2 Humble下，使用Moveit2 API 进行控制🥰在实际开发过程中遇到了不少问题，包括UR官方的Driver、Description版本，ros2-control，Moveit2等问题，在此不做全部解释。有任何问题欢迎发邮件给我：ruanjinchen@foxmail.com
# 资料
ros2-control的文档：https://control.ros.org/humble/doc/getting_started/getting_started.html  
Moveit2的文档：https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html  
如果您使用VMware虚拟机，那么RAM至少分配16G，内核总数至少8核，这为了保障在22.04下编译等运行的流畅度  
# 环境配置步骤（从装好系统开始）
前提：安装完成了ubuntu-22.04.3-desktop-amd64  
  ## （虚拟机必看！）进入桌面之后，需要取消WaylandEnable=false的注释，才能完成PC与虚拟机之间文件夹的拖拽  
```
sudo gedit /etc/gdm3/custom.conf
```
  ## 安装ROS2 Humble和创建工作空间、编译需要的包
```
wget http://fishros.com/install -O fishros && . fishros
rosdepc update
sudo apt install git net-tools terminator
```
  ## 创建工作空间，并编译
```
mkdir -p ~/ur
cd ur
git clone https://github.com/ruanjinchen/Dual_UR_Arm_CAR.git
mv Dual_UR_Arm_CAR src
cd src/dual_ur_moveit_config
mv setup_assistant .setup_assistant
cd ..
cd ..
```
  ## 如果安装了rosdepc,就用rosdepc，没有就用rosdep
```
sudo rosdepc init
rosdepc update
rosdepc install --ignore-src --from-paths src -y
colcon build
echo " source ~/ur/install/local_setup.sh" >> ~/.bashrc
```
# 运行
首先，需要检查双臂的ip地址是否与PC在同一网段，并且分别记录左右机械臂的ip，此项目中为192.168.8.195和192.168.8.196.如果不一致，则需要修改/ur_driver/ur_robot_driver/launch/dual_ur_control.launch.py中的robot_ip。其次，需要给两个机械臂安装External Control，并且配置端口分别为50002和60002。最后就可以分别执行一下指令与机械臂建立通信与控制。
``` 
ros2 launch ur_robot_driver dual_ur_control.launch.py 
ros2 launch dual_ur_moveit_config dual_ur_moveit.launch.py 
```
有的时候会报错：...is of type {double}, setting it to {string} is not allowed，原因是[编码的问题](https://github.com/ros-planning/moveit2/issues/1049)，[感谢Gaël Écorchard的解答](https://github.com/ros-planning/moveit2/issues/1049#issuecomment-1047029751)，需要执行下面的指令:
```
LC_NUMERIC=en_US.UTF-8 ros2 launch dual_ur_moveit_config dual_ur_moveit.launch.py 
```
# 其他
API控制是读取的本地的Pose.json文件，所以在运行前需要确保机械臂安全的情况下，再启动API
```
ros2 launch dual_ur_moveit_api dual_ur_moveit_api.launch.py
``` 
