/**
 * @file showsonar_node.cpp
 * @brief  Brief description of file.
 *
 */

#include "ros/ros.h"

#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"

#include "renderWorld.h"
#include "renderPioneer.h"

#define NUM_SONARS 8
#define FARAWAY 5

SDL sdl(600,600);
SDLContext context(&sdl);

Wheely robot(0,0,-PI/2,16);
void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,int i){
    robot.sonarDists[i] = msg->range;
    if(robot.sonarDists[i]>FARAWAY)robot.sonarDists[i]=FARAWAY;
}

PioneerRenderer rRobot(&context);

int main(int argc,char *argv[]){
    ros::init(argc,argv,"showsonar_node");
    ros::NodeHandle n;
    ros::Subscriber s[NUM_SONARS];
    
    Matrix3x3 worldmat;
    double scalefactor = (sdl.getWidth()/2-5)/2;
    
    worldmat.scale(scalefactor,-scalefactor); // flip Y
    worldmat.translate(sdl.getWidth()/2,sdl.getHeight()/2);
    context.mult(worldmat);
    
    World world;
    world.add(&robot,&rRobot);
    
    for(int i=0;i<NUM_SONARS;i++){
        char buf[32];
        sprintf(buf,"s%d",i);
        s[i] = n.subscribe<sensor_msgs::Range>(buf,1000,
                                 boost::bind(sonarCallback,_1,i));
        robot.sonarDists[i]=FARAWAY;
    }
    sdl.open();
    ros::Rate rate(10);
    bool running=true;
    bool saveNextFrame=false;
    while(ros::ok() && running){
        ros::spinOnce();
        rate.sleep();
        
        sdl.beginFrame();
        world.render();
        int k=sdl.endFrame();
        if(saveNextFrame){
            sdl.saveBMP("sonar.bmp");
            saveNextFrame=false;
        }
        if(k<0)running=false;
        else switch(k){
        case 'q':running=false;break;
        case 's':saveNextFrame=true;break;
        default:break;
        }
        
    }
}
