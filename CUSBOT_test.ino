#include<Wire.h> 
#include "CUSBOT.h"

CUSBOT Bot(3,4,2,5,6,7);
//CUSBOT Bot(4,3,2,5,6,7);
float oldTime = 0;
float tStart = 0;
float yd = 0;
float xd = 0;
float heading = 0;

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600);
  Wire.begin();
//  Bot.WIFI_init();
  Bot.IMU_init();
  tStart = (float)millis()/1000.0;
}

void loop()
{
  /* Testing segment, to make sure the robots are up and running*/
//  Bot.controlBot();
//  Bot.controlBot(1,0.0,'h'); //for now, you have to multiply omega by 0.7 for correct performance
//  Bot.openLoop(200.0);
//  Bot.openLoopSlave(230);

  /*Robot moving on a circle following a point mass*/
  // calculating the velocity components in x and y directions
  float v = 1;
  float f = 0.15;
  float t = (float)millis()/1000.0 - tStart;
  
  xd = v*cos(2*PI*f*t);
  yd = v*sin(2*PI*f*t);
  heading = atan2(yd,xd);
  oldTime = t;
 
//  float dt = t - oldTime;
//  if(dt > 0 && dt < 1.5)heading=0;
//  else if(dt > 1.5 && dt < 3)heading = -PI/2;
//  else if(dt >3 && dt < 4.5)heading = -PI;
//  else if(dt > 4.5 && dt <6)heading = PI/2;
//  else
//  {
//    heading = 0;
//    oldTime = t;
//  }
  
  // translating the x and y velocity components into velocity and directions
  //float heading = atan2(yd,xd)*180.0/PI;
//  Serial2.println(heading);
  Bot.controlBot(v,0,'h');
}
