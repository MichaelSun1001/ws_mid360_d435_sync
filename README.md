# 环境依赖
1、修改了OpenCV环境，将原有的源码安装版本(4.5.4)卸载，并使用apt安装版本（4.2.0）

卸载命令：
sudo dpkg -r libopencv-python libopencv-samples libopencv opencv-licenses
sudo rm -rf /usr/local/include/opencv*
sudo make uninstall（这个是在4.5.4的源码目录下build文件夹中执行）

安装命令：
sudo apt install libopencv-dev
sudo apt install ros-noetic-compressed-image-transport
sudo apt install ros-noetic-compressed-depth-image-transport
sudo apt install ros-noetic-theora-image-transport

查看相关版本命令：
apt list --installed | grep opencv
apt-cache policy libopencv-dev
apt-cache policy libopencv-core4.2
grep -r opencv /etc/apt/sources.list.d/
ls /usr/local/lib/libopencv*
ls /usr/local/include/opencv*
ls /usr/local/include/opencv*

# 文件改动

## 增加文件夹ws_livox_mid360，该文件夹作用为mid360驱动

编译命令：
conda deactivate
./build.sh ROS1

驱动启动文件命令：

1、ROS运行可视化的驱动（点云是pointcloud类型）

cd ~/ws_livox_mid360
source devel/setup.sh
roslaunch livox_ros_driver2 rviz_MID360.launch

2、ROS运行无可视化的驱动（自定义msg）

cd ~/ws_livox_mid360
source devel/setup.sh
roslaunch livox_ros_driver2 msg_MID360.launch


## 增加文件夹ws_realsense-ros，该文件夹作用为realsense驱动

驱动启动命令：
source devel/setup.sh
roslaunch realsense2_camera demo_pointcloud.launch

## 增加文ws_mid360_d435_sync，该文件夹作用为realsense和mid360的软同步驱动

驱动启动命令：
source devel/setup.sh
roslaunch livox_ros_driver2 msg_MID360.launch

source devel/setup.sh
roslaunch livox_ros_driver2 msg_MID360_sync.launch


# 数据集录制

注意：注意执行顺序，并且注意launch文件执行路径

source devel/setup.sh
roslaunch livox_ros_driver2 msg_MID360.launch

source devel/setup.sh
roslaunch realsense2_camera demo_pointcloud.launch

source devel/setup.sh
roslaunch livox_ros_driver2 msg_MID360_sync.launch



rosbag record /synced_image /synced_lidar /livox/imu

<!-- rosbag record /camera/color/image_raw/compressed /camera/depth/image_rect_raw/compressed /camera/depth/color/points /livox/lidar /livox/imu -->
