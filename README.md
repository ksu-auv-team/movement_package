### Installation:

```bash
sudo apt update && \
sudo apt -y install git vim cmake catkin
```

If you have any issues installing ROS, checkout the official [ROS Wiki](http://wiki.ros.org/melodic/Installation/Ubuntu)
```bash
#Install ROS Melodic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
sudo apt update && \
sudo apt install ros-melodic-desktop-full && \
sudo rosdep init && \
rosdep update && \
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
```

**If you are setting up the whole sub, you should follow the instructions on the [sub-utilities repo](https://github.com/ksu-auv-team/sub-utilities.git):**
```bash
TODO: Add Instructions from sub-utilities
```

**If you are only setting up this repo, not the whole sub, follow these instrucitons:**
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
sudo apt -y install ros-melodic-mavlink ros-melodic-mavros ros-melodic-mavros-msgs \
    ros-melodic-cmake-modules ros-melodic-control-toolbox ros-melodic-joy
```

```bash
# install geoid's for mavros
sudo geographiclib-get-geoids minimal
```


**If installing only this repo:**
```bash
# install movement library
cd ~/catkin_ws/src && \
git clone https://github.com/ksu-auv-team/movement_package.git && \
cm
```


### Running:
```roslaunch [tab]``` to seee options
