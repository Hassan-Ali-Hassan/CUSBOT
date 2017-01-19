#include<Wire.h> 
#include "CUSBOT.h"

#define CASE 1
CUSBOT Bot(3,4,2,5,6,7);
//CUSBOT Bot(4,3,2,5,6,7);
float oldTime = 0;
float tStart = 0;
float yd = 0;
float xd = 0;
float heading = 0;
float t = 0;
float v = 0.7;
int counter1 = 100;
int counter2 = 100;


char a = 0;


void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200);
//  Serial2.begin(9600);
  Wire.begin();
  randomSeed(analogRead(0));
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
//  Bot.WIFI_init();
  Bot.IMU_init();
  Bot.setInitialPosition(0,0,-PI/2*0);
}

void loop()
{
  t = (float)millis()/1000.0;
  /* Testing segment, to make sure the robots are up and running*/
//  Bot.controlBot();
  Bot.controlBot(v,-3*PI/4,'h'); //for now, you have to multiply omega by 0.7 for correct performance
//  Bot.openLoop(200.0);
//  Bot.openLoopSlave(230);
  Bot.updatePositions2();
//  Bot.estimateDistance();
  Bot.estimatePosition();
  if(t>10)v = 0;
//  Bot.goInCircle();
  Serial.println(t-oldTime,6);
  oldTime = t;
}
