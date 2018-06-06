Installation:

```bash
sudo apt update && \
sudo apt -y install git vim cmake catkin
```

```bash
#Install ROS Kinetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 0xB01FA116 && \
sudo apt update && \
sudo apt -y install ros-kinetic-ros-core && \
sudo rosdep init && \
rosdep update && \
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
```bash
#Setup Catkin Workspace
mkdir -p ~/catkin_ws/src && \
cd ~/catkin_ws/src && \
catkin_init_workspace && \
catkin_make -C ~/catkin_ws/ && \
echo "alias cm='catkin_make -j $(nproc) -C ~/catkin_ws/'" >> ~/.bash_aliases &&\
source ~/.bashrc
```

```bash
# install ros packages
sudo apt -y install ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-mavlink ros-kinetic-mavros ros-kinetic-mavros-msgs \
    ros-kinetic-cmake-modules ros-kinetic-control-toolbox  ros-kinetic-joy
```

```bash
# install geoid's for mavros
sudo geographiclib-get-geoids minimal
```

```bash
# install movement library
cd ~/catkin_ws/src && \
git clone https://github.com/ksu-auv-team/movement_package.git && \
cm
```


### Running:
```roslaunch [tab]``` to seee options
