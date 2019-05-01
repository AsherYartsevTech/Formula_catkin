# Formula_catkin


# ros on multiple machines cooking:

1.ssh listener
    roscore
    
2. ssh listener
    export DISPLAY=:0
    start node
3. ssh talker
    export ROS_MASTER_URI=http://<ip_address_listener>:11311
    export ROS_IP=<ip_adress_talker>
    export ROS_HOSTNAME=<ip_adress_talker>
    start node
    
