#include<Wire.h> 

#define AGENT 3
//#define TEST  /*This variables is defined when there is a need to perform tests directly on the rover (to test a new feature or debug some problem)*/
#define SINGLE/* This is for testing rendezvous and formations with control laws based on single integrator point mass*/
//#define DOUBLE/* This is for testing rendezvous and formations with control laws based on double integrator point mass*/


float oldTime = 0;
float t = 0;

#ifdef TEST
#include "CUSBOT.h"
CUSBOT Bot(AGENT);
float pos[6];
float v = 0.5;
bool testParam = true;
#endif

#ifdef SINGLE
#include "singleIntegratorPointDynamics.h"
singleIntegratorPointDynamics sBot(AGENT);
#endif 

#ifdef DOUBLE
#include "doubleIntegratorPointDynamics.h"
doubleIntegratorPointDynamics dBot(AGENT);
#endif 

void setup()
{
//  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(9600);
  Wire.begin();
  
  #if OPTICALFLOW_ENABLED == 1 /*enabling the SPI for optical flow sensor, regardless of where it is in code*/
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  #endif
  randomSeed(analogRead(0));
  pinMode(13,OUTPUT);
  pinMode(44,OUTPUT);
  while( Serial2.read() != -1 ); //flusing the serial buffer after initializing the IMU, in order to neglect the initial confirmation string coming from the esp module

  #ifdef TEST
  #if OPTICALFLOW_ENABLED == 1
  Bot.optical_init();
  #endif
  Bot.IMU_init();
  #endif 

  #ifdef SINGLE
  sBot.init();
  #endif

  #ifdef DOUBLE
  dBot.init();
  #endif
}

void loop()
{
  

  #ifdef TEST
  t = (float)millis()/1000.0 - Bot.getStartingTime();
  /* Testing segment, to make sure the robots are up and running*/
//  Bot.controlBot();
  Bot.controlBot(v,-PI/4*0,'h'); //for now, you have to multiply omega by 0.7 for correct performance
//  Bot.openLoop(200.0);
//  Bot.openLoopSlave(80);
//  Bot.updatePositions2();
//  Bot.updatePositions3();
//  Bot.updatePositionsHTTP();
//  Bot.estimateDistance();
//  Bot.estimatePosition();
//  if(t>5)Bot.stopRover();
//  Bot.goInCircle();
//  Bot.xbeeLikeOperation();
//  opticalFlowTest();
//  moveAndLocalize2();
//  if(testParam)
//  {
//    Bot.parameterEstimationTest();
//    testParam = false;
//  }
//  if(t > 5)
//  {
//    Bot.stopRover();
//    Bot.keepIMUBusy();
//  }
//  else
//  {
////    Bot.controlBot(0.5,-PI/4,'h');
////    Bot.controlBot(v,-PI/4*0,'h');
//    Bot.openLoopSlave(1000);
//  }
  oldTime = t;
  #endif

  #ifdef SINGLE
//  sBot.rendezvousHTTP();
  sBot.formationHTTP();
//  sBot.formationMQTT();
//  sBot.rendezvousMQTT();
  #endif

  #ifdef DOUBLE
  dBot.rendezvousHTTP();
//  dBot.formationHTTP();
  #endif
}

#ifdef TEST
void moveAndLocalize()
{
  float a[7];
  Bot.xbeeLikeOperation();
  Bot.estimatePosition(); //tries to figure out where we are
  Bot.updatePositionsHTTP(); //tries to fgiure out where the neighbours are
  Bot.getPositions(a);  //just returns the positions.
  Serial3.print(a[0]);
  Serial3.print("\t");
  Serial3.println(a[1]);
}

void moveAndLocalize2()
{
  float a[7];
  Bot.xbeeLikeOperation();
//  Bot.estimatePosition(); //tries to figure out where we are
  Bot.updatePositions3(); //tries to fgiure out where the neighbours are
  Bot.getPositions2(a);  //just returns the positions.
  Serial3.print(a[0]);
  Serial3.print("\t");
  Serial3.print(a[1]);
  Serial3.print("\t");
  Serial3.println(a[6]);
}

void opticalFlowTest()
{
  Bot.estimatePosition2();
  Bot.getPositions(pos);
  Serial3.print(pos[0]);
  Serial3.print("\t");
  Serial3.println(pos[1]);
}
#endif
