#include "doubleIntegratorPointDynamics.h"

doubleIntegratorPointDynamics :: doubleIntegratorPointDynamics(int i): Bot(i)
{  
  index = i;
  if(index == 1)
  {
//    Bot.setInitialPosition(195,120,0); //Note that positions are in cm and angles are in radians
//    zref[0] = -0.5; zref[1] = -0.866;
    Bot.setInitialPosition(90,90,0); //Note that positions are in cm and angles are in radians
    zref[0] = -0.5; zref[1] = -0.866;
    numOfNeighbours = 1;
  }
  else if(index == 2)
  {
//    Bot.setInitialPosition(120,275,0);
//    zref[0] = 0.5; zref[1] = 0.866; zref[2] = 1; zref[3] = 0;
    Bot.setInitialPosition(210,420,0);
    zref[0] = 0.5; zref[1] = 0.866; zref[2] = -0.5; zref[3] = 0.866;
    numOfNeighbours = 2;
  }
  else if(index == 3)
  {
//    Bot.setInitialPosition(330,345,0);
//    zref[0] = -1; zref[1] = 0;
    Bot.setInitialPosition(360,120,0);
    zref[0] = 0.5; zref[1] = -0.866;
    numOfNeighbours = 1;
  }

  for(int j = 0; j < 7; j++)
  {
    oldPos[j] = 0;
  }
  oldTime = 0;
  xdot = 0;
  ydot = 0;
  xddot = 0;
  yddot = 0;
  vel = 0;
  theta = 0;
  factor = 0.3;
  start = false;
}

void doubleIntegratorPointDynamics ::init()
{
  #if OPTICALFLOW_ENABLED == 1
  Bot.optical_init();
  #endif
  Bot.IMU_init();
}

void doubleIntegratorPointDynamics::rendezvousMQTT()
{
  /*
   * This function aims to let every robot in the network go towards its neighbours and
   * eventually hit each other.
   */
 xdot = 0;
 ydot = 0;
 
 Bot.updatePositions3(); //tries to fgiure out where the neighbours are
 Bot.getPositions2(a);  //just returns the positions.
 
 /* making sure that the neighbours' positions are not zero, as a signal to start*/
 if(a[6] == 1)
 {
   for(int i = 1; i <=numOfNeighbours; i++)//we loop to 2 as the current case is of 3 robots, where every robot has at most two neighbours
   {
    xdot += (a[2*i] - a[0])*factor;
    ydot += (a[2*i+1] - a[1])*factor;
   }
   vel = sqrt(xdot*xdot + ydot*ydot);
   theta = atan2(ydot,xdot);
   
   if(vel < 0.2) vel = 0;
   else if(vel > 0.8) vel = 0.8;
   
   Serial3.print(theta);
   Serial3.print("\t");
   Serial3.println(vel);
   Bot.controlBot(vel,theta,'h');
 }
 else
 {
  Bot.keepIMUBusy();
  Bot.stopRover();
 }
}

void doubleIntegratorPointDynamics::rendezvousHTTP()
{
  /*
   * This function aims to let every robot in the network go towards its neighbours and
   * eventually hit each other.
   */
 float t = (float)millis()/1000.0 - Bot.getStartingTime();
 float dt = 0;
 xddot = 0;
 yddot = 0;
 Bot.estimatePosition(); //tries to figure out where we are
 Bot.updatePositionsHTTP(); //tries to fgiure out where the neighbours are
 Bot.getPositions(a);  //just returns the positions. 
 /* making sure that the neighbours' positions are not zero, as a signal to start*/
 if((int)a[6] == 1)
 {
  start = true;
 }
 else
 {
  start = false;
  Bot.stopRover();
 }
 if(start)
 {
   dt = t - oldTime;
   for(int j = 0; j < 7; j++)
   {
    V[j] = (a[j] - oldPos[j])/dt;
    oldPos[j] = a[j];
   }
   oldTime = t;
   for(int i = 1; i <=numOfNeighbours; i++)//we loop to 2 as the current case is of 3 robots, where every robot has at most two neighbours
   {
    xddot += (a[2*i] - a[0] + V[2*i] - V[0])*factor;
    yddot += (a[2*i+1] - a[1] + V[2*i+1] - V[1])*factor;
   }
   xdot += xddot * dt;
   ydot += yddot * dt;
   vel = sqrt(xdot*xdot + ydot*ydot);
   theta = atan2(ydot,xdot);
//   Serial3.print(theta*180/PI);
//   Serial3.print("\t");
//   Serial3.println(vel);
   if(vel < 0.2) vel = 0;
   else if(vel > 0.8) vel = 0.8;
   Bot.controlBot(vel,theta,'h');
 }
 else
 {
  Bot.keepIMUBusy();
  Bot.stopRover();
 }
}

void doubleIntegratorPointDynamics::formationMQTT()
{
 xdot = 0;
 ydot = 0;
 Bot.updatePositions3(); //tries to fgiure out where the neighbours are
 Bot.getPositions2(a);  //just returns the positions.
 
 /* making sure that the neighbours' positions are not zero, as a signal to start*/
 if(a[6] == 1)
 {
   for(int i = 1; i <=numOfNeighbours; i++)//we loop to 2 as the current case is of 3 robots, where every robot has at most two neighbours
   {
    xdot += (a[2*i] - a[0] + zref[2*(i-1)])*factor;
    ydot += (a[2*i+1] - a[1]+ zref[2*(i-1)+1])*factor;
   }
   vel = sqrt(xdot*xdot + ydot*ydot);
   theta = atan2(ydot,xdot);
   
   if(vel < 0.2) vel = 0;
   else if(vel > 0.8) vel = 0.8;
   
   Serial3.print(theta);
   Serial3.print("\t");
   Serial3.println(vel);
   Bot.controlBot(vel,theta,'h');
 }
 else
 {
  Bot.keepIMUBusy();
  Bot.stopRover();
 }
}

void doubleIntegratorPointDynamics::formationHTTP()
{
  /*
   * This function aims to let every robot in the network go towards its neighbours and
   * eventually hit each other.
   */
 xdot = 0;
 ydot = 0;
 
 Bot.estimatePosition(); //tries to figure out where we are
 Bot.updatePositionsHTTP(); //tries to fgiure out where the neighbours are
 Bot.getPositions(a);  //just returns the positions.
 
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
   for(int i = 1; i <=numOfNeighbours; i++)//we loop to 2 as the current case is of 3 robots, where every robot has at most two neighbours
   {
    xdot += (a[2*i] - a[0] + zref[2*(i-1)])*factor;
    ydot += (a[2*i+1] - a[1]+ zref[2*(i-1)+1])*factor;
   }
//   ydot += 0.3;
   vel = sqrt(xdot*xdot + ydot*ydot);
   theta = atan2(ydot,xdot);

   if(vel < 0.2) vel = 0;
   else if(vel > 0.8) vel = 0.8;
   Bot.controlBot(vel,theta,'h');
 }
 else
 {
  Bot.keepIMUBusy();
  Bot.stopRover();
 }
}



