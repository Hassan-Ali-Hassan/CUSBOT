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
  
  #if CASE == 1 /*normal operation without optical flow sensor, and with using wifi module*/
  Serial2.begin(115200);
  #elif CASE == 2 /*operation using xbee like wireless module*/
  Serial2.begin(9600);
  #endif

  #if OPTICALFLOW_ENABLED == 1
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  Bot.optical_init();
  #endif
  Wire.begin();
  randomSeed(analogRead(0));
  pinMode(13,OUTPUT);
  pinMode(44,OUTPUT);
//  digitalWrite(44,HIGH);
//  digitalWrite(44,LOW);
//  Bot.WIFI_init();
  Bot.IMU_init();

  #if AGENT == 1
//  Bot.setInitialPosition(0,0,0);
  Bot.setInitialPosition(195,120,0); //Note that positions are in cm and angles are in radians
  numOfNeighbours = 1;
//  zref[0] = 0.5; zref[1] = 0.866; //this vector contains the reference values of the interagent vectors, expressed in global axes
  zref[0] = -0.5; zref[1] = -0.866;
  
  #elif AGENT == 2
//  Bot.setInitialPosition(195,120,0); //Note that positions are in cm and angles are in radians
  Bot.setInitialPosition(120,275,0);
//  Bot.setInitialPosition(0,0,0);
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
  t = (float)millis()/1000.0 - Bot.getStartingTime();
  /* Testing segment, to make sure the robots are up and running*/
//  Bot.controlBot();
//  Bot.controlBot(v,-PI/4*0,'h'); //for now, you have to multiply omega by 0.7 for correct performance
//  Bot.openLoop(200.0);
//  Bot.openLoopSlave(80);
//  Bot.updatePositions2();
//  Bot.updatePositions3();
//  Bot.updatePositionsHTTP();
//  Bot.estimateDistance();
//  Bot.estimatePosition();
//  if(t>5)Bot.stopRover();
//  Bot.goInCircle();
//  rendezvous();
//  rendezvous2();
//  formation();
//  formation2();
//  GO();
//  Bot.xbeeLikeOperation();
//  opticalFlowTest();
//  Serial.println(t-oldTime,6);

  if(t > 5 && t < 10)
  {
    Bot.stopRover();
    Bot.keepIMUBusy();
  }
  else
  {
    Bot.controlBot(0.5,-PI/4,'h');
  }
  oldTime = t;
}

void opticalFlowTest()
{
  Bot.estimatePosition2();
  Bot.getPositions(pos);
  Serial2.print(pos[0]);
  Serial2.print("\t");
  Serial2.println(pos[1]);
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
 Bot.updatePositions3(); //tries to fgiure out where the neighbours are
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

//////////////////////////////////////////////////
void rendezvous2()
{
  /*
   * This function aims to let every robot in the network go towards its neighbours and
   * eventually hit each other.
   */
 float xdot = 0;
 float ydot = 0;
 int i = 0;
 float a[7];
 float vel = 0;
 float theta = 0;
 float yaw = 0;
 static boolean start = false;
 static boolean checkStart = true;
 float factor = 0.5;

 Bot.estimatePosition(); //tries to figure out where we are
 Bot.updatePositionsHTTP(); //tries to fgiure out where the neighbours are
 Bot.getPositions(a);  //just returns the positions.
 for(int j = 0; j < 6; j++){Serial.print(a[j]);Serial.print("\t");}
  Serial.println((int)a[6]);
// checkAnomalies(a);
 
 /* making sure that the neighbours' positions are not zero, as a signal to start*/
 if((int)a[6] == 1)
 {
  start = true;
 }
 else
 {
  start = false;
  Bot.controlBot(0,0,'h');
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
   
//   Serial.print(theta);
//   Serial.print("\t");
//   Serial.println(vel);
   Bot.controlBot(vel,theta,'h');
//    Bot.controlBot(v,0,'h');
//    if(millis() > 25000)v = 0;
 }
 else
 {
  Bot.keepIMUBusy();
 }
}

void formation2()
{
  /*
   * This function aims to let every robot in the network go towards its neighbours and
   * eventually hit each other.
   */
 float xdot = 0;
 float ydot = 0;
 int i = 0;
 float a[7];
 float vel = 0;
 float theta = 0;
 float yaw = 0;
 static boolean start = false;
 static boolean checkStart = true;
 float factor = 0.5;

 Bot.estimatePosition(); //tries to figure out where we are
 Bot.updatePositionsHTTP(); //tries to fgiure out where the neighbours are
 Bot.getPositions(a);  //just returns the positions.
// checkAnomalies(a);
 
 /* making sure that the neighbours' positions are not zero, as a signal to start*/
 if((int)a[6] == 1)
 {
  start = true;
 }
 else
 {
  start = false;
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

   if(vel < 0.2) vel = 0;
   
//   Serial.print(theta);
//   Serial.print("\t");
//   Serial.println(vel);
   Bot.controlBot(vel,theta,'h');
 }
 else
 {
  Bot.keepIMUBusy();
 }
}

/* In what follows are some testing functions to test different parts of the system*/
void GO()
{
  /*
   * This function aims to let every robot in the network go towards its neighbours and
   * eventually hit each other.
   */
 float xdot = 0;
 float ydot = 0;
 int i = 0;
 float a[7];
 float vel = 0;
 float theta = 0;
 float yaw = 0;
 static boolean start = false;
 static boolean checkStart = true;
 float factor = 0.5;

 Bot.estimatePosition(); //tries to figure out where we are
 Bot.updatePositionsHTTP(); //tries to fgiure out where the neighbours are
 Bot.getPositions(a);  //just returns the positions.
 for(int j = 0; j < 6; j++){Serial.print(a[j]);Serial.print("\t");}
  Serial.println((int)a[6]);
// checkAnomalies(a);
 
 /* making sure that the neighbours' positions are not zero, as a signal to start*/
 if((int)a[6] == 1)
 {
  start = true;
 }
 else
 {
  start = false;
 }
 if(start)
 {
//   Bot.controlBot(vel,theta,'h');
    v = 0.7;
    Bot.controlBot(v,0,'h');
//    if(millis() > 25000)v = 0;
 }
 else
 {
    v = 0;
    Bot.controlBot(v,0,'h');
 }
}
