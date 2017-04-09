#ifndef SINGLE_H
#define SINGLE_H

#include "CUSBOT.h"

class singleIntegratorPointDynamics{

  public:

  singleIntegratorPointDynamics(int);
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
  float a[7];
  float vel;
  float theta;
  float factor;
  bool start;

};
#endif
