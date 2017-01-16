#ifndef WIFI_H
#define WIFI_H
#define MESSAGESIZE  5

#include"Arduino.h"
class wifi{
  
  public:
  wifi();
  void init();
  boolean update();
  char messageC[MESSAGESIZE];
  float messageI[MESSAGESIZE]; //I for instructions
  
  protected:
  void parse();
  boolean parse2();
  boolean parsePosition();
  void setESP();
  void testCon();
  void echo();
  boolean isNum(char);
  boolean isLetter(char);
  
  int INDEX;
  String m ;
};
#endif
