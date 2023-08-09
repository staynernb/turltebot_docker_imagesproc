FROM osrf/ros:humble-desktop-full

# Changing apt to use Finland mirror for sources
COPY ./sources.list /etc/apt

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND noninteractive

# Installing cob libraries
RUN apt-get -y update

RUN apt-get update \
  && apt-get -y install \
    ros-${ROS_DISTRO}-gazebo-* \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-ros 

RUN apt -y install software-properties-common 
RUN add-apt-repository -y ppa:kisak/kisak-mesa

#RUN source /root/.bashrc
RUN /bin/bash -c "source /root/.bashrc"
RUN apt-get -y install \
    ros-${ROS_DISTRO}-turtlebot3 \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-turtlebot3-simulations \
    ros-${ROS_DISTRO}-dynamixel-sdk 
  #&& rm -rf /var/lib/apt/lists/*

RUN apt-get update
RUN apt-get -y upgrade

# Creating and switching to user
ARG USER=rosuser
RUN useradd -ms /bin/bash $USER
RUN usermod -a -G root $USER
#Puting the user created in the sudoers     
RUN usermod -aG sudo $USER && echo "rosuser ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/rosuser
# Adding password if want
RUN echo 'root:123' | chpasswd 

USER $USER

# Build turtlebot3_ws

RUN echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> /home/${USER}/.bashrc
RUN echo 'export TURTLEBOT3_MODEL=waffle_pi #TURTLEBOT3' >> /home/${USER}/.bashrc

#  Source environments
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/${USER}/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/${USER}/.bashrc && \
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> /home/${USER}/.bashrc

WORKDIR /home/${USER}/turtlebot3_ws
