# Turtlebot docker

Select the compatible docker environment with the configuration installed in the turtlebot SD card, and check the README.md inside each project, for while is available the distros: 
- [Humble](./Humble_env/)
- [Noetic](./Noetic_env/)


## Issues registered:

 - To build the image may is necessary be logged in the aalto open, and not by ethernet cable to install some packets that may the aalto network block ubuntu packets. After build one time you can connect to the aalto network.
 - If the wifi of the turtlebot is appearing but not connecting, access the turtlebot by ethernet cable and run: - sudo systemctl start dnsmasq
    
