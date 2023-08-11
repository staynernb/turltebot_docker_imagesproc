FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=noetic
RUN apt update; apt install -y gnupg2

RUN apt-get install -y ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers         

RUN apt install -y ros-${ROS_DISTRO}-gazebo-* 
RUN apt install -y ros-noetic-dynamixel-sdk \
    ros-noetic-turtlebot3-msgs \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations

#To verify IPS
RUN apt install -y net-tools nano

RUN ln -s /usr/bin/python3 /usr/bin/python

# Creating and switching to user
ARG USER=rosuser
RUN useradd -ms /bin/bash $USER
RUN usermod -a -G root $USER
#Puting the user created in the sudoers     
RUN usermod -aG sudo $USER && echo "rosuser ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/rosuser
# Adding password if want
RUN echo 'root:123' | chpasswd  

WORKDIR /home/$USER/catkin_ws

USER $USER


#  Source environments
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/${USER}/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc
COPY ./bashrc_turtlebot_include /home/${USER}/.bashrc_turtlebot_include 
RUN echo "source /home/${USER}/.bashrc_turtlebot_include" >> /home/${USER}/.bashrc

# Optionals to run examples
RUN sudo apt install -y python3-tk
RUN sudo apt install -y pip
RUN pip install pandas

