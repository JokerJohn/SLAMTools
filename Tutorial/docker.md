## create docker with rviz

```bash
#!/bin/bash

# ----------------------------
# 配置参数
# ----------------------------
CONTAINER_NAME=ros2_auto
IMAGE_NAME=osrf/ros:humble-desktop
WORKSPACE_DIR=$HOME/auto_ws    # 你的代码路径
DATA_DIR=$HOME/data            # 你的数据路径

# ----------------------------
# X11 配置（支持 RViz2/Gazebo 图形界面）
# ----------------------------
xhost +local:docker

# ----------------------------
# 如果容器已经存在，先删除
# ----------------------------
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
    echo ">>> Removing old container: $CONTAINER_NAME"
    docker rm -f $CONTAINER_NAME
fi

# ----------------------------
# 运行容器
# ----------------------------
docker run -it \
    --name $CONTAINER_NAME \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$WORKSPACE_DIR:/root/auto_ws" \
    --volume="$DATA_DIR:/autocity/data" \
    $IMAGE_NAME \
    bash -c "mkdir -p /autocity/data/test1 && \
             source /opt/ros/humble/setup.bash && \
             cd /root/auto_ws && exec bash"

```

宿主机工作目录：$HOME/auto_ws， 数据路径：$HOME/data

docker中的对应目录：~/auto_ws，数据文件：/autocity/data

数据文件是共享目录的，可以在宿主机上操作。



创建好进入这个docker

```bash

# 启动这个已存在的容器（后台运行）
docker start ros2_auto

# 进入容器交互式终端
docker exec -it ros2_auto /bin/bash



#if we want to process this data folder which mounted in docker in my ubuntu
ls -ld   ~/data
sudo chown -R $USER:$USER ~/data
```

```
# 基础镜像：Ubuntu 22.04 + ROS 2 Humble
FROM ros:humble

# 设置非交互模式，防止 tzdata 等阻塞安装
ENV DEBIAN_FRONTEND=noninteractive

# 更新源并安装依赖
RUN apt update && apt install -y \
    libgoogle-glog-dev \
    nlohmann-json3-dev \
    vim \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# 设置默认 shell
SHELL ["/bin/bash", "-c"]

# 创建工作空间目录（可按需修改）
WORKDIR /root/ros2_ws

# 初始化 ros2 环境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```



```bash
apt update
apt install -y libgoogle-glog-dev
apt install -y nlohmann-json3-dev
```

```bash
export ROS_LOCALHOST_ONLY=1

source install/setup.bash

cd ~/auto_ws
rm -rf build/ install/ log/

# 一次性编译所有依赖的消息包
colcon build --symlink-install --packages-select mapping_msgs msfl_msgs shm_msgs

# 再编译 mapping_manager
colcon build --symlink-install --packages-select mapping_manager --cmake-args "-DGTSAM_USE_SYSTEM_EIGEN=ON"

colcon build --packages-select mapping_manager --cmake-args "-DGTSAM_USE_SYSTEM_EIGEN=ON"
```



#### ros2 bag convert to ros1

```bash
# the direct command
sudo pip install ros2bag-convert

ros2bag-convert xx.db3

#  when you have some
pip show ros2bag-convert


# db3 to mcp, you should set the convert_mcap.yaml 

apt install ros-humble-rosbag2-storage-mcap

ros2 bag convert -i /autocity/data/test1/2/sensor_data/check/check_0.db3 -o /autocity/data/test1/2/sensor_data/check/convert_mcap.yaml 

output_bags:
  - uri: ./check_0_mcap
    storage_id: mcap


#ros2 bag convert -i /autocity/data/test1/1/sensor_data/check/check_0.db3 -o /autocity/data/test1/1/sensor_data/check/convert_mcap.yaml 

# then convert mcp to rosbag

rosbags-convert --src check_0_mcap/ --dst /autocity/data/test1/2/sensor_data/check/output.bag --dst-typestore ros1_noetic

```



### Open3D Issue:

```
git clone https://github.com/isl-org/Open3D
# Only needed for Ubuntu
util/install_deps_ubuntu.sh

mkdir build
cd build
cmake ..

make install
```



```
# -- Fetching ISPC compiler   Request completely sent off
# closing connection #0
#         --- LOG END --- 

# this was caused by proxy, download it mannually
wget https://github.com/ispc/ispc/releases/download/v1.16.1/ispc-v1.16.1-linux.tar.gz
mv ispc-v1.16.1-linux.tar.gz /home/auto/software/Open3D/3rdparty_downloads/ispc/ispc-v1.16.1-linux.tar.gz
cmake ..

# sometimes we need to desable some libs to avoid some errors, e.g. uninstall 

```



## cloudcompare install issue:

```
# 若只需 PCD 支持，开启 QPCL 即可，关闭 qLASIO 和 qPDALIO 可大幅减少编译问题：
git pull
git submodule update --init --recursive

cmake .. -DPLUGIN_STANDARD_QPCL=ON -DPLUGIN_IO_QLAS=OFF -DPLUGIN_IO_QPDAL=OFF

rm -rf build/* 
mkdir build && cd build
```

