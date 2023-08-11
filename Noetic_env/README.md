# Turtlebot docker

## Init the robot by any terminal
### User credentials of the robot:

- user: ubuntu
- pswd: turtlebot

### Ethernet cable on aalto
```
ssh ubuntu@130.233.123.205
```

### wifi (turtlebot as hostspot)
- ssid: turtlebot
- pswd: turtlebot

```
ssh ubuntu@192.168.220.1
```

### Robot connected to aalto_open (Default way)
```
ssh ubuntu@10.100.48.7
```

### Init robot
```
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

## HOW BUILD the docker

### Set up IP details
Change the file [bashrc_turtlebot_include](./bashrc_turtlebot_include) to your account details if necessary.

If you are connected to aalto network please check the registered issues.

***Recommended:*** 
```
docker-compose build
```

## Graphics Terminal Permission

***Its necessary give permission to the terminal that will run the docker to have graphics interface access inside the docker***

```
xhost +
```


## HOW RUN the docker

***Recommended:***
```
docker compose run turtlebot bash
```
*or if you want to use the GPU Nvidia graphics (if your computer have a nvidia board)*

```
docker compose run turtlebot-nvidia bash
```

*If you are using windows run:*

```
docker compose run turtlebot-win bash
```

*or windows with nvidia*

```
docker compose run turtlebot-win-nvidia bash
```

## Other ways to run the docker

```
docker run -it --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
         --device /dev/dri/   --ulimit nofile=1024 \
         --volume="/dev/dri:/dev/dri" \
         --mount type=bind,source="$(pwd)"/turtlebot_docker_shared,target=/app  \
         turtlebot-docker     bash
```
## Util commands for COB

### Teleoperation

```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Ros2 vizualize

```
rosrun rviz rviz -d `rospack find turtlebot3_description`/rviz/waffle_pi.rviz
```

### Mapping

```
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Save the map:
```
rosrun map_server map_saver -f ~/map
```

### Navigation

```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

### Simulation

```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Creating Python codes to interact with the robot

The easier way is create the python code inside the folder [Catkin_ws/src/beginner_tutorials](/Noetic_env/catkin_ws/src/beginner_tutorials/).

After create the file run:

```
cd ~/catkin_ws
source ./devel/setup.bash
```

So will be possible to run your file with:
```
rosrun beginner_tutorials $file_name
```

## Shared folder

The folder [turtlebot_docker_shared](/Noetic_env/turtlebot_docker_shared/) is shared with the docker, when the docker initialize, the files of this folder will be shared with the folder */app/* in the docker directory. 

If you save files in this folder using the docker, they will be persistent and not deleted when the docker initialize again. 

If you want to send files of the host to be used in the docker, is just needed put these files in the [turtlebot_docker_shared](/Noetic_env/turtlebot_docker_shared/) folder

## Notes
 - To run the docker the folder [turtlebot_docker_shared](/Noetic_env/turtlebot_docker_shared/) should exist in the directory.
 - The root password of the docker is: 123

## Issues registered:

 - To build the image may is necessary be logged in the aalto open, and not by ethernet cable to install some packets that may the aalto network block ubuntu packets. After build one time you can connect to the aalto network.
 - If the wifi of the turtlebot is appearing but not connecting, access the turtlebot by ethernet cable and run: - sudo systemctl start dnsmasq
 - nvidia not tested.
 - When you do git clone from this repository from a windows computer, please change the format of End Line sequence of [bashrc_turtlebot_include](./bashrc_turtlebot_include) from CRLF to LF.
