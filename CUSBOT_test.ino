#include<Wire.h> 
#include "CUSBOT.h"

#define CASE 1
#define AGENT 1

CUSBOT Bot(3,4,2,5,6,7);

float oldTime = 0;
float t = 0;
float v = 0.7;
float pos[6];
int numOfNeighbours = 0;
float zref[4];

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

  #if AGENT == 1
//  Bot.setInitialPosition(120,275,0);
  Bot.setInitialPosition(195,120,0); //Note that positions are in cm and angles are in radians
  numOfNeighbours = 1;
//  zref[0] = 0.5; zref[1] = 0.866; //this vector contains the reference values of the interagent vectors, expressed in global axes
  zref[0] = -0.5; zref[1] = -0.866;
  
  #elif AGENT == 2
//  Bot.setInitialPosition(195,120,0); //Note that positions are in cm and angles are in radians
  Bot.setInitialPosition(120,275,0);
  numOfNeighbours = 2;
//  zref[0] = -0.5; zref[1] = -0.866; zref[2] = 0.5; zref[3] = -0.866;
  zref[0] = 0.5; zref[1] = 0.866; zref[2] = 1; zref[3] = 0;
  
  #elif AGENT == 3
  Bot.setInitialPosition(330,345,0);
  numOfNeighbours = 1;
//  zref[0] = -0.5; zref[1] = 0.866;
  zref[0] = -1; zref[1] = 0;
  #endif
}

void loop()
{
  t = (float)millis()/1000.0;
  /* Testing segment, to make sure the robots are up and running*/
//  Bot.controlBot();
//  Bot.controlBot(v,-3*PI/4,'h'); //for now, you have to multiply omega by 0.7 for correct performance
//  Bot.openLoop(200.0);
//  Bot.openLoopSlave(120);
//  Bot.updatePositions2();
//  Bot.estimateDistance();
//  Bot.estimatePosition();
//  if(t>10)v = 0;
//  Bot.goInCircle();
//  rendezvous();
  formation();
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
// checkAnomalies(a);
 
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
   
   if(vel < 0.2) vel = 0;
   
   Serial.print(theta);
   Serial.print("\t");
   Serial.println(vel);
   Bot.controlBot(vel,theta,'h');
 }
}

void formation()
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
// checkAnomalies(a);
 
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
    xdot += (a[2*i] - a[0] + zref[2*(i-1)])*factor;
    ydot += (a[2*i+1] - a[1]+ zref[2*(i-1)+1])*factor;
   }
   ydot += 0.3;
   vel = sqrt(xdot*xdot + ydot*ydot);
   theta = atan2(ydot,xdot);

//   if(vel < 0.2) vel = 0;
   
   Serial.print(theta);
   Serial.print("\t");
   Serial.println(vel);
   Bot.controlBot(vel,theta,'h');
 }
}

void checkAnomalies(float* a)
{
  /*
   * This function tries to detect abnormal changes in 
   * position readings obtained from other agents.
   */
   static float oldReadings[4];
   float change[4]; //describes the change from the old readings
   static boolean referenceEstablished = false;
   int i = 0;
   
   if(!referenceEstablished)
   {
      referenceEstablished = true;
      for(i = 0; i < 6; i++)
      {
        if(a[i] < 1)
        {
          referenceEstablished = false;
          break;
        }
      }
   }
   else
   {
      for(i = 0; i < 4; i++)
      {
        change[i] = abs(a[i+2] - oldReadings[i]); //inspecting the change from previous readings. Usually, there are some corrupt packages that get to the agents and lead them to take extreme and bizare disicions. if the change is too big for the current small sampling time, then most probably this readings is poor.
        if(change[i] > 1) //remember the posiitions are in meters. This condition means that the change is as high as 1 m in such small time period (50 ms)
        {
          a[i+2] = oldReadings[i];
        }
        else
        {
          oldReadings[i] = a[i+2];
        }
      }
   }
}

