## bridge_node

Bridge from Aria to ROS. Requires the bridgeserver (from phd/code/pioneernet)
to be running on the pioneer.

Usage:
    rosrun pioneernet bridge_node
    

###Topics published:

topic|type|definition
-----|----|------
s0.. |sensor_msgs::Range|sonar data (units TBD)
light|lightsensor_gazebo::LightSensor|linear array of uchar RGB pixels from omni camera

###Topics subscribed to:

topic|type|definition
-----|----|----------
leftmotor|Float64|required left motor rotation (mm/sec)
rightmotor|Float64|required right motor rotation (mm/sec)

###Services

service|args|description
-------|----|-------
photo|none|take a picture "foo.jpg" on the robot


## pioneernet_node

Control the Pioneer with a network.
