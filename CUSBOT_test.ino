#include<Wire.h> 
#include "CUSBOT.h"

#define CASE 1
#define AGENT 3

CUSBOT Bot(3,4,2,5,6,7);

float oldTime = 0;
float t = 0;
float v = 0.7;
float pos[6];
int numOfNeighbours = 0;

void setup()
{
  Serial.begin(115200);
//  Serial2.begin(115200);
  Serial2.begin(9600);
  Wire.begin();
  randomSeed(analogRead(0));
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
//  Bot.WIFI_init();
  Bot.IMU_init();

  #if AGENT == 1
//  Bot.setInitialPosition(120,275,0);
  Bot.setInitialPosition(195,120,0); //Note that positions are in cm and angles are in radians
  numOfNeighbours = 1;
  #elif AGENT == 2
//  Bot.setInitialPosition(195,120,0); //Note that positions are in cm and angles are in radians
  Bot.setInitialPosition(120,275,0);
  numOfNeighbours = 2;
  #elif AGENT == 3
  Bot.setInitialPosition(330,345,0);
  numOfNeighbours = 1;
  #endif
}

void loop()
{
  t = (float)millis()/1000.0;
  /* Testing segment, to make sure the robots are up and running*/
  Bot.controlBot();
//  Bot.controlBot(v,-3*PI/4,'h'); //for now, you have to multiply omega by 0.7 for correct performance
//  Bot.openLoop(200.0);
//  Bot.openLoopSlave(230);
//  Bot.updatePositions2();
//  Bot.estimateDistance();
//  Bot.estimatePosition();
//  if(t>10)v = 0;
//  Bot.goInCircle();
//  rendezvous();
//  Serial.println(t-oldTime,6);
  oldTime = t;
}

void rendezvous()
{
  /*
   * This function aims to let every robot in the network go towards its neighbours and
   * eventually hit each other.
   */
 float xdot = 0;
 float ydot = 0;
 int i = 0;
 float a[6];
 float vel = 0;
 float theta = 0;
 float yaw = 0;
 static boolean start = false;
 static boolean checkStart = true;
 float factor = 0.3;

 Bot.estimatePosition(); //tries to figure out where we are
 Bot.updatePositions2(); //tries to fgiure out where the neighbours are
 Bot.getPositions(a);  //just returns the positions.
 
 /* making sure that the neighbours' positions are not zero, as a signal to start*/
 if(!start && checkStart)
 {
  for(i = 1; i <= numOfNeighbours; i++)
  {
    if(a[2*i] <1|| a[2*i+1] <1) //this is just a condition to kickstart the function and let the robots begin moving towards each other
    {
      start = false;
      break;
    }
    start = true; //at this point, the for loop hasn't broken which means that other's positions are legit and we are allowed to start
    checkStart = false; //this is a latach, so once the condition is satisfied, it will continue working.
  }
 }
 if(start)
 {
   for(i = 1; i <=numOfNeighbours; i++)//we loop to 2 as the current case is of 3 robots, where every robot has at most two neighbours
   {
    xdot += (a[2*i] - a[0])*factor;
    ydot += (a[2*i+1] - a[1])*factor;
   }
   vel = sqrt(xdot*xdot + ydot*ydot);
   theta = atan2(ydot,xdot);
   Serial.print(theta);
   Serial.print("\t");
   Serial.println(vel);
   Bot.controlBot(vel,theta,'h');
 }
}

