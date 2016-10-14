/**
 * @file bridge_node.cpp
 * @brief A bridge from the Pioneer's ARIA system to the local
 * ROS system.
 * 1) reads IP packets coming from other side of the bridge, the Pioneer,
 *    and converts them to ROS topic publications.
 * 2) reads ROS topics, converts them to IP packets, sends them to the
 *    other side of the bridge.
 * This is started as a client, so needs a server on the Pioneer side
 * running.
 */

#include "ros/ros.h"

#include "lightsensor_gazebo/LightSensor.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "tcp.h"
#include "pioneerros.h"

#include <diamondapparatus/diamondapparatus.h>
#include <iostream>
#include <string>
#include <sstream>



#define ENABLEDIAMOND 0

// the port we connect to on the Pioneer
#define PORT 34312


ros::Publisher sonarPubs[NUM_SONARS];
ros::Publisher lightPub;

double sigWidth=1;
double sigCentre=0.5;
double thresh=0.3;
double sigAmount=0;

// used to log the pixel data
/*
FILE *logRings=NULL;
void logRing(const char *title,uint8_t *p,int n){
    if(!logRings)
        logRings=fopen("ringlog","w");
    fprintf(logRings,"%s: ",title);
    for(int i=0;i<n;i++){
        uint16_t v = p[i*3+0];
        v+=p[i*3+1];
        v+=p[i*3+2];
        fprintf(logRings,"%d,",v/3);
    }
    fprintf(logRings,"\n");
}
*/

inline double gloveconv(double f){
    return f/1024.0;
}
inline double sigmoid(double x){
    x=(x-sigCentre)/(sigWidth+0.001);
    return 1.0/(1.0+exp(-x));
}

#define KSIZE 5
#define HALFK ((KSIZE-1)/2)
//float k[]={0.38774,0.24477,0.06136}; // 5-kern, sigma=1
// 7-kernel, sigma=1.2, half of it, backwards ;)
float k[] = {0.0169,0.087,0.2236,0.3242};


void blur(uint8_t *out,uint8_t *p,int ch,int n){
    uint8_t mx=0;
    for(int i=0;i<n;i++){
        double t = 0;
        for(int j=-HALFK;j<=HALFK;j++){
            int px = (i+j+n)%n;
            t+= (double)(p[px*3+ch])*k[j<0?-j:j];
        }
        t /= (double)KSIZE;
        uint8_t bv=(uint8_t)t;
        out[i*3+ch]=bv;
        if(bv>mx)mx=bv;
    }
    // and normalise
    double normfac=255.0/(double)mx;
    for(int i=0;i<n;i++){
        double px = (double)(out[i*3+ch]);
        px *= normfac;
        out[i*3+ch]=(uint8_t)px;
    }
}



int processpixel(int p){
    double x = p;
//    printf("In: %f ",x);
    x = x/255.0;
//    printf("Scale: %f ",x);
    double orig = x;
    x=sigmoid(x);
//    printf("Sig: %f ",x);
    
    x=(sigAmount*x)+(1-sigAmount)*orig;
    
    if(x<thresh)x=0;
//    printf("Thr: %f ",x);
    
    return (int)(x*255.0);
}

// cheap low-pass: exponentially weighted moving average,
// "Brown's simple exponential smoothing". (see brown1963smoothing).
class LPF {
    double alpha;
    double p;
public:
    LPF(double a){ // the filter parameter. High=more persistence, lower freq
        alpha=a;
        p=0;
    }
    void reset(){
        p=0;
    }
    void setAlpha(double a){
        alpha=a;
    }
    double run(double v){
        p = alpha*p + (1.0-alpha)*v;
        return p;
    }
};
/// the client object

class BridgeClient : public TCPClient<MotorPacket,SensorPacket> {
    int seq;
    sensor_msgs::Range r; // a range message
    ros::Time lastTick;
    LPF *lpfs[NUM_PIXELS*3]; // PROBABLY UNUSED (tried to use, see 290916exp12
public:
    BridgeClient(const char *addr) : TCPClient<MotorPacket,SensorPacket>(addr,PORT){
        seq=0;
        r.header.frame_id = 1; // global frame (0 is no frame)
        r.radiation_type = 0; // ultrasonic
        r.field_of_view = 0.2618f; // 15 degrees
        r.min_range = 0;
        r.max_range = 5;
        lastTick = ros::Time::now();
        for(int i=0;i<NUM_PIXELS*3;i++){
            lpfs[i]=new LPF(0.0); // see above!
        }
    }
    
    virtual void process(){
        double time = (ros::Time::now() - lastTick).toSec();
        lastTick = ros::Time::now();
        printf("Processing, interval = %f\n",time);
        // turn the sensor packet into ROS topic publications
        r.header.seq = seq++;
        r.header.stamp = ros::Time::now();
        
        for(int i=0;i<NUM_SONARS;i++){
            r.range = resp.sonars[i]*0.001f;
            sonarPubs[i].publish(r);
        }
        printf("Sonars OK\n");
        printf("response size %ld\n",sizeof(resp));
        
        printf("Sigamt: %f  Sig: c=%f/w=%f  Thr:%f\n",sigAmount,
               sigCentre,sigWidth,thresh);
        
        // Now the light sensor.
        
        // Blur the data with a gaussian (exp 280916)
//        logRing("preblur",resp.pixels,NUM_PIXELS);
        uint8_t blurred[NUM_PIXELS*3];
        blur(blurred,resp.pixels,0,NUM_PIXELS);
        blur(blurred,resp.pixels,1,NUM_PIXELS);
        blur(blurred,resp.pixels,2,NUM_PIXELS);
//        logRing("postblur",blurred,NUM_PIXELS);
        
        // Now publish the data as RGB pixels
        
        lightsensor_gazebo::LightSensor ls;
//        fprintf(logRings,"thresh: ");
        for(int i=0;i<NUM_PIXELS;i++){
            // these are 0-255
            lightsensor_gazebo::Pixel pix;
            // see above, LPFs do nothing perhaps
            pix.r = lpfs[i*3+0]->run(blurred[i*3+0]);
            pix.g = lpfs[i*3+1]->run(blurred[i*3+1]);
            pix.b = lpfs[i*3+2]->run(blurred[i*3+2]);
            
            pix.r = processpixel(pix.r);
            pix.g = processpixel(pix.g);
            pix.b = processpixel(pix.b);
//            printf("%d: %d %d %d\n",i,pix.r,pix.g,pix.b);
            
//            fprintf(logRings,"%d,",(pix.r+pix.g+pix.b)/3);
            
            ls.pixels.push_back(pix);
        }
//        fprintf(logRings,"\n");
        printf("Light sensors OK\n");
        lightPub.publish(ls);
    }
};

BridgeClient *client;

void motorCallback(const std_msgs::Float64::ConstPtr& msg, int i){
    // turn the ROS topic data into data to send to the pioneer -
    // it will be sent at regular intervals to ensure the robot
    // connection is safe - if the server notices no packets arriving
    // it will stop the motors.
    
    // Note that the Pioneer's ARIA library works in mm/sec, but we
    // receive commands in m/sec (to make us compatible with how Gazebo
    // works)
    
    client->req.motors[i]=msg->data*1000.0;
}

bool photoCallbackPre(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& resp){
    client->req.command = COMMAND_PHOTO_PREBLUR;
    return true;
}

bool photoCallbackPost(std_srvs::Empty::Request& req,
                      std_srvs::Empty::Response& resp){
    client->req.command = COMMAND_PHOTO_POSTBLUR;
    return true;
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"bridge_node");
    ros::NodeHandle node;
    
#if(ENABLEDIAMOND)
    diamondapparatus::init();
#endif
    // publishers
    
    for(int i=0;i<NUM_SONARS;i++){
        std::ostringstream name;
        name << "s" << i;
        sonarPubs[i] = node.advertise<sensor_msgs::Range>(name.str(),100);
    }
    
    lightPub = node.advertise<lightsensor_gazebo::LightSensor>("light",100);
    
    // subscribers
    
    ros::Subscriber lsub = node.subscribe<std_msgs::Float64>("leftmotor",1000,
                                  boost::bind(motorCallback,_1,0));
    ros::Subscriber rsub = node.subscribe<std_msgs::Float64>("rightmotor",1000,
                                   boost::bind(motorCallback,_1,1));
    
    // services
    ros::ServiceServer servPre = node.advertiseService("photopre",photoCallbackPre);
    ros::ServiceServer servPost = node.advertiseService("photopost",photoCallbackPost);
    
    // get parameters
    
    std::string addr;
    if(!node.getParam("pioneeraddr",addr)){
        ROS_FATAL("no 'pioneeraddr' parameter set");
        return 1;
    }
    
    // subscribe to diamond knob control stuff
#if(ENABLEDIAMOND)
    diamondapparatus::subscribe("/glove/knobs");
#endif    
    try {
        // start the client and loop away!
        client = new BridgeClient(addr.c_str());
    } catch(TCPException e) {
        ROS_FATAL("TCP failure - %s",e.what());
        exit(1);
    }
    // clear the outgoing motor packet, though..
    client->req.motors[0]=0;
    client->req.motors[1]=0;
    client->req.command=0;
    
    ros::Rate rate(5);
    while(ros::ok()){
        printf("Send\n");
        ros::spinOnce();
        client->update(); // check sensor data
        client->request(); // send motor data
        client->req.command=0; // clear any command
        rate.sleep();
        
#if(ENABLEDIAMOND)
        diamondapparatus::Topic gloveTopic =
              diamondapparatus::get("/glove/knobs",GET_WAITNONE);
        if(gloveTopic.isValid()){
            sigCentre = gloveconv(gloveTopic[0].f());
            sigWidth = gloveconv(gloveTopic[1].f());
            thresh = gloveconv(gloveTopic[2].f());
            sigAmount = gloveconv(gloveTopic[3].f());
        }
#endif
    }
}
