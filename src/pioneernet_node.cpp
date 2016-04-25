/**
 * @file pioneernet_node.cpp
 * @brief  Brief description of file.
 *
 */
#include "ros/ros.h"
#include "backpropNoBiasHormone.h"

#include "lightsensor_gazebo/LightSensor.h"

int main(int argc,char *argv[]){
    ros::init(argc,argv,"pioneernet_node");
    ros::NodeHandle n;
    
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

}
