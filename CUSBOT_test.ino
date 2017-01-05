#include<Wire.h> 
#include "CUSBOT.h"

CUSBOT Bot(3,4,2,5,6,7);
//CUSBOT Bot(4,3,2,5,6,7);
void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
//  Bot.WIFI_init();
  Bot.IMU_init();
}

void loop()
{
  Bot.controlBot();
//  Bot.controlBot(0.7,0.0); //for now, you have to multiply omega by 0.7 for correct performance
//  Bot.openLoop(200.0);
//Bot.openLoopSlave(230);
}
