/**
 * @file pioneernet_node.cpp
 * @brief  Brief description of file.
 *
 */

#include <getopt.h>
#include <angort/params.h>

#include "ros/ros.h"
#include "backpropNoBiasHormone.h"

#include "lightsensor_gazebo/LightSensor.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"

#include "power.h"

#define NUM_SONARS 8

struct option opts[]={
        {"net", required_argument, NULL, 'n'},
        {"time", required_argument, NULL, 't'},
        {"vars", required_argument, NULL, 'V'},
        {"constfile", required_argument, NULL, 'c'},
        {"hormone", required_argument, NULL, 'h'},
        {NULL,0,NULL,0},
};


double kM,kPower,kBase;
Value *mapFunc; // function to map charge to hormone or hormone input
Value *resetFunc; // function to reset values between runs
Parameters *params;

void getParams(const char *cf,char *varstr){
    params = new Parameters(cf,varstr);
    const char *err = params->check();
    if(err){
        fprintf(stderr,"Parameter fetch failed:%s\n",err);
        exit(1);
    }
    
    kBase = params->getFloat("kbase");
    kM = params->getFloat("km");
    kPower = params->getFloat("kpower");
    
    if(params->exists("hmapfunc"))
        mapFunc = params->getFunc("hmapfunc");
    else
        mapFunc = NULL;
    
    if(params->exists("resetfunc"))
        resetFunc = params->getFunc("resetfunc");
    else
        resetFunc = NULL;
    
    
}


double sonarDists[NUM_SONARS];
void sonarCallback(const sensor_msgs::Range::ConstPtr& msg,int i){
    sonarDists[i] = msg->range;
}



uint8_t pix[1000][3];
double monoPix[1000];
int numPixels =0;
double totalLight=0;
void lightCallback(const lightsensor_gazebo::LightSensor::ConstPtr& msg){
    numPixels = msg->pixels.size();
    totalLight=0;
    for(int i=0;i<numPixels;i++){
        
        pix[i][0] = msg->pixels[i].r;
        pix[i][1] = msg->pixels[i].g;
        pix[i][2] = msg->pixels[i].b;
        
        double p = pix[i][0]+pix[i][1]+pix[i][2];
        p *= 1.0/(256.0*3.0);
        monoPix[i]=p;
        totalLight += p;
    }
}

int main(int argc,char *argv[]){
    ros::init(argc,argv,"pioneernet_node");
    ros::NodeHandle n;
    ros::Subscriber s[NUM_SONARS];
    ros::Subscriber light;
    
    float maxtime = 1000000;
    bool manual=false;
    double inithormone=0;
    char *varString = NULL;
    
    char constFile[1024];
    const char *cfe = getenv("WHEELYCONSTFILE");
    strcpy(constFile,cfe?cfe:"constants.ang");
    
    for(int i=0;i<NUM_SONARS;i++){
        char buf[32];
        sprintf(buf,"s%d",i);
        s[i] = n.subscribe<sensor_msgs::Range>(buf,1000,
                                 boost::bind(sonarCallback,_1,i));
        sonarDists[i]=1000;
    }
    
    light = n.subscribe<lightsensor_gazebo::LightSensor>
          ("light",1000,lightCallback);
    
    BackpropNet *net = NULL;
    int optidx=0,c;
    while(c=getopt_long(argc,argv,"n:t:V:c:h:",opts,&optidx)){
        if(c<0)break;
        switch(c){
        case 't':
            maxtime = atof(optarg);
            break;
        case 'n':
            net = BackpropNet::loadNet(optarg);
            break;
        case 'V':
            varString = strdup(optarg);
            break;
        case 'c':
            strcpy(constFile,optarg);
            break;
        case 'h':
            manual=true;
            inithormone=atof(optarg);
            break;
        case '?':
            break;
        default:abort();
        }
    }
    getParams(constFile,varString);

    
    if(!net){
        fprintf(stderr,"Error: no network specified\n");
        abort();
    }
            
    
    ros::Publisher leftmotor = n.advertise<std_msgs::Float64>("leftmotor",1000);
    ros::Publisher rightmotor = n.advertise<std_msgs::Float64>("rightmotor",1000);
    
    printf("Net loaded (%d-%d-%d), running...\n",
           net->getLayerSize(0),
           net->getLayerSize(1),
           net->getLayerSize(2)
           );
    ros::Rate rate(10);
    
    double hormone = inithormone;
    
    int numins = net->getLayerSize(0);
    double *inp =new double [numins];
    for(int i=0;i<numins;i++)inp[i]=0;
    
    Power power(kBase,kM);
    
    if(resetFunc)
        params->runFunc(resetFunc);
    
    ros::Time lastTick = ros::Time::now();
    
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
        
        // update the network, get the outputs and feed them
        // to the robot
        net->setH(hormone);
        int inpidx=0;
        if(NUM_SONARS+numPixels>numins){
            fprintf(stderr,"Too many pixels? sonar=%d, pixels=%d, netins=%d\n",
                    NUM_SONARS,numPixels,numins);
            abort();
        }
        for(int i=0;i<NUM_SONARS;i++)
            inp[inpidx++]=sonarDists[i];
        for(int i=0;i<numPixels;i++){
            inp[inpidx++]=monoPix[i];
        }
        net->setInputs(inp);
        net->update();
        double *outs = net->getOutputs();
        std_msgs::Float64 m;
        m.data = outs[0];
        leftmotor.publish(m);
        m.data = outs[1];
        rightmotor.publish(m);
        // update the power management
        
        double time = (ros::Time::now() - lastTick).toSec();
        lastTick = ros::Time::now();
        
        power.update(time,totalLight*kPower,outs[0],outs[1]);
        
        // update the hormone
        if(!manual){
            if(mapFunc){
                params->pushFuncArg((float)power.charge);
                params->runFunc(mapFunc);
                hormone = params->popFuncResult();
            }
            else
                hormone = 1.0-power.charge;
        }
        else
            power.charge=1; // weird hormone, fix charge
        
        printf("Hormone: %f, charge %f\n",hormone,power.charge);
         
    }
}
