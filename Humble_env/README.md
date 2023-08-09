# Turtlebot docker Humble

## Init the robot by any terminal
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

### Init robot
```
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
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
docker compose run turtlebot-humble bash
```
*or if you want to use the GPU Nvidia graphics (if your computer have a nvidia board)*

```
docker compose run turtlebot-humble-nvidia bash
```

*If you are using windows run:*

```
docker compose run turtlebot-humble-win bash
```

*or windows with nvidia*

```
docker compose run turtlebot-humble-win-nvidia bash
```

## Other ways to run the docker

```
docker run -it --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1"  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
         --device /dev/dri/   --ulimit nofile=1024 \
         --volume="/dev/dri:/dev/dri" \
         --mount type=bind,source="$(pwd)"/turtlebot_docker_shared,target=/app  \
         turtlebot-humble-docker     bash
```
## Util commands for COB

### Teleoperation

```
ros2 run turtlebot3_teleop teleop_keyboard
```

### Ros2 vizualize

```
rqt
```

### Mapping

```
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

Save the map:
```
ros2 run nav2_map_server map_saver_cli -f ~/map
```

### Simulation

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## Shared folder

The folder [turtlebot_docker_shared](/Humble_env/turtlebot_docker_shared/) is shared with the docker, when the docker initialize, the files of this folder will be shared with the folder */app/* in the docker directory. 

If you save files in this folder using the docker, they will be persistent and not deleted when the docker initialize again. 

If you want to send files of the host to be used in the docker, is just needed put these files in the [turtlebot_docker_shared](/Humble_env/turtlebot_docker_shared/) folder

## Notes
 - To run the docker the folder [turtlebot_docker_shared](/Humble_env/turtlebot_docker_shared/) should exist in the directory.
 - The root password is: 123

## Issues registered:

 - To build the image may is necessary be logged in the aalto open, and not by ethernet cable to install some packets that may the aalto network block ubuntu packets. After build one time you can connect to the aalto network.
 - If the wifi of the turtlebot is appearing but not connecting, access the turtlebot by ethernet cable and run: - sudo systemctl start dnsmasq
 - The navigation is acutally not working in the humble envinroment, please use the Noetic one.
- When you do git clone from this repository from a windows computer, please change the format of End Line sequence of [bashrc_turtlebot_include](./bashrc_turtlebot_include) from CRLF to LF.
    
