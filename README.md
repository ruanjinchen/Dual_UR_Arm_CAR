# Dual_UR_Arm_CAR

OS: Ubuntu 22.04 ROS2 Humble

ros2_control源码安装官网：https://control.ros.org/humble/doc/getting_started/getting_started.html#building-from-source
Moveit2源码安装官网：https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html
虚拟机RAM至少分配16G，内核总数至少8核

安装ubuntu-22.04.3-desktop-amd64
虚拟机设置：取消WaylandEnable=false的注释，才能完成PC与虚拟机之间文件夹的拖拽
sudo gedit /etc/gdm3/custom.conf

安装ROS2 Humble和创建工作空间、编译需要的包
wget http://fishros.com/install -O fishros && . fishros
rosdepc update
sudo apt install python3-vcstool git net-tools
创建工作空间
mkdir -p ~/ws_ur/src
cd ~/ws_ur/src

git clone https://github.com/ros-planning/moveit2_tutorials
vcs import < moveit2_tutorials/moveit2_tutorials.repos

cd ..
sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

wget https://raw.githubusercontent.com/ros-controls/control.ros.org/master/ros_controls.$ROS_DISTRO.repos
vcs import src < ros_controls.$ROS_DISTRO.repos
rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

source install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py

colcon build --symlink-install
source install/setup.bash
ros2 launch ur_moveit_config demo.launch.py

Moveit2的工作空间中：test1/src/ur_moveit_config/config/joint_limits.yaml文件中，默认生成的joint_limits是false，这个值是从URDF中直接读取得到的，在SolidWorks导出URDF的时候并未做这部分的限制。在Moveit2中，所有的    has_velocity_limits: true 和 has_acceleration_limits: true 需要手动改为true。max_velocity: 0.8 和 max_acceleration: 0.8 需要自己修改值（不能为0，为0动不了）




ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3 robot_ip:=192.168.8.195 launch_rviz:=true


ros2 launch ur_moveit_config ur_moveit.launch.py launch_rviz:=true
ros2 launch ur_moveit_config ur_moveit_backup.launch.py ur_type:=ur3 launch_rviz:=true





Can't locate move_group node in package moveit_ros_move_group
https://github.com/ros-planning/moveit/issues/200
将Moveit2源码复制到UR的工作空间，进行源码安装



executable 'servo_node_main' not found
https://github.com/ros-planning/moveit2/issues/2383
$ cd /home/user/ros2_ws/install/moveit_servo/lib/moveit_servo
$ ln -s servo_node servo_node_main



ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py



parameters=[{'robot_description': launch_ros.descriptions.ParameterValue( launch.substitutions.Command(['xacro ',os.path.join(turtlebot2_description_package,'robots/kobuki_conveyor.urdf.xacro')]), value_type=str)  }]
这是啥？根本用不了。




ros2 control list_controllers
