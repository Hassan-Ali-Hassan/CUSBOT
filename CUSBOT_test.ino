#include<Wire.h> 
#include "CUSBOT.h"

CUSBOT Bot(3,4,2,5,6,7);
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  Bot.IMU_init();
}

void loop()
{
  Bot.controlBot(0.7,0);
  Serial.println();
}
