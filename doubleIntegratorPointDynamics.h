#ifndef DOUBLE_H
#define DOUBLE_H

#include "CUSBOT.h"

class doubleIntegratorPointDynamics{

  public:

  doubleIntegratorPointDynamics(int);
  void init();
  void rendezvousMQTT();
  void rendezvousHTTP();
  void formationMQTT();
  void formationHTTP();
  
  protected:
  CUSBOT Bot;
  int index;
  int numOfNeighbours;
  float zref[4];
  float xdot;
  float ydot;
  float xddot;
  float yddot;
  float a[7];
  float oldPos[7];
  float oldTime;
  float V[7];
  float vel;
  float theta;
  float factor;
  bool start;

};
#endif
