#include "wifi.h"

wifi::wifi()
{
  int i = 0;
  for(i = 0; i < MESSAGESIZE; i++)
  {
    messageI[i] = 0;
    messageC[i] = 0;
  }
}

void wifi::init()
{
  setESP();
}

void wifi::setESP()
{
  Serial.println("setting the esp");
  String networkConfiguration = "wifi.sta.config(\"SU27\",\"opera2525\")\r\n";
  String brokerSettings = "m:connect(\"192.168.1.101\", 1883, 0, function(client) print(\"connected\")end)\r\n";
//  String instructions = "m:on(\"message\", function(conn, topic, data)if data ~= nil then T = topic.gsub(topic,\"topic\",'$') print(T .. \":\".. data..\"#\") end end)\r\n";
//  String topics = "m:subscribe({[\"topic0\"]=0,[\"topic1\"]=0,[\"topic2\"]=0}, function(conn) print(\"subscribe success\") end)\r\n";
  String instructions = "m:on(\"message\", function(conn, topic, data)if data ~= nil then if string.find(topic,\"direction\") ~= nil then T = topic.gsub(topic,\"direction\",'$d') print(T .. \":\".. data..\"#\") elseif string.find(topic,\"speed\") ~= nil then T = topic.gsub(topic,\"speed\",'$s') print(T .. \":\".. data..\"#\") end end end) \r\n";
  String topics = "m:subscribe({[\"direction1\"]=0,[\"speed1\"]=0}, function(conn) print(\"subscribe success\") end)\r\n";
  String payload[] = {
                      "wifi.setmode(wifi.STATION)\r\n",                    
                      networkConfiguration,
                      "print(wifi.sta.status())\r\n",
                      "m = mqtt.Client(\"client_id\", 120, \"\",\"\")\r\n",
                      "m:on(\"connect\", function(con) print (\"connected\") end)\r\n",
                      "m:on(\"offline\", function(con) print (\"offline\") end)\r\n",
                      instructions,
                      topics,
                      brokerSettings                                            
                     };
  for (int i = 0; i < 9; i++)
  {
    Serial2.print(payload[i]);
//    Serial.println(payload[i]);
    delay(200);
    echo();
    delay(200);
  }
}

void wifi::testCon()
{
  char a = 0;
  Serial2.print("print(wifi.sta.status())\r\n");
  while (Serial2.available())
  {
    a = Serial2.read();
    Serial.write(a);
    if(a == '5')
    {
      Serial.println("ESP is connected to wifi");
    }
  }
}

void wifi::echo()
{
  while (Serial2.available())
  {
    Serial.write(Serial2.read());
  }
}

void wifi::parse()
{
  char a = 0;
  int i = 0;
  int j = 0;
  bool seekIndex = false;
  int topicIndex = 0;
  j = 0;
  while (Serial2.available())
  {
    a = Serial2.read();
    if ( (a >= 97 && a <= 122)/*if a is a letter*/ || (a >= 48 && a <= 57)/*...or a is a number*/ || a == '#' /*...or a is an end of phrase*/|| a == '$' /*... or a is the begining of a phrase*/|| a == ':'/*...or a is a separator between topic name and its value*/)
    {
      if (a == '$')
      {
        seekIndex = true;
        j = 0;
        m = "";
      }
      else if (a == ':')
      {
        seekIndex = false;
      }
      else if (seekIndex)
      {
        topicIndex = (int)a - 48;
      }
      else if (!seekIndex)
      {
        if(a != '#') 
        {
          m += a;
        }
        else
        {
          messageI[topicIndex] = m.toInt();
          analogWrite(13,messageI[topicIndex]);
        }
      }
      delay(2);
    }
  }
}

boolean wifi::parsePosition()
{
  boolean ongoingProcessFlag = false;
  boolean endOfProcessFlag = false;
  boolean seekIndexFlag = false;
  boolean seekXValueFlag = false;
  boolean seekYValueFlag = false;
  char a = 0;
  String xVal;
  String yVal;
  
  if(Serial2.available())
  {
    while(Serial2.available())
    {
      a = Serial2.read();
//      Serial.println(a);
      if(a == '$' && !ongoingProcessFlag) //if we are available and this is the mark of new message
      {       
        ongoingProcessFlag = true;
        seekIndexFlag = true;
        seekXValueFlag = false;
        seekYValueFlag = false;
      }
      else if(ongoingProcessFlag)
      {
        if(a == ':')
        {
          seekIndexFlag = false;
          seekXValueFlag = true;
          seekYValueFlag = false;
        }
        else if(a == ',')
        {
          seekXValueFlag = false;
          seekYValueFlag = true;
        }
        else if(a == '#')
        {
          ongoingProcessFlag = false;
          endOfProcessFlag = true;
          messageI[2*INDEX] = (float)xVal.toInt()/10.0;
          messageI[2*INDEX+1] = (float)yVal.toInt()/10.0;
        }
        else if(seekIndexFlag)
        {
          if(a == 'd')INDEX = 0;
          else if(a == 's')INDEX = 1;
        }
        else if(seekXValueFlag)
        {
          if(isNum(a))
          {
            xVal += a;
          }
        }
        else if(seekYValueFlag)
        {
          if(isNum(a))
          {
            yVal += a;
          }
        }
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

boolean wifi::parse2() //this is parsing instructions from the CUSBOT_CONTROL flow on node-red
{ 
  // The output of this function is to save the direction instruction in the first element of messageI and the speed 
  // instruction in the second element.
  char a = 0;
  int i = 0;
  int j = 0;
  bool seekIndex = false;

  j = 0;
  if(Serial2.available())
  {
    while(Serial2.available())
    {
      a = Serial2.read();
      if ( isLetter(a)/*if a is a letter*/ || isNum(a)/*...or a is a number*/ || a == '#' /*...or a is an end of phrase*/|| a == '$' /*... or a is the begining of a phrase*/|| a == ':'/*...or a is a separator between topic name and its value*/|| a == '-' /*...or a is a minus sign*/ || a == '.'/*a decimal point*/)
      {
        if (a == '$') //which marks the begining of a new data string. Here we set our variables to their default states
        {
          seekIndex = true;
          j = 0;
          m = "";
        }
        else if (a == ':') //here we've reached the separator between the topic name and its value
        {
          seekIndex = false;
        }
        else if (seekIndex) //if the characheter is not a special charachter, and we haven't reached the address/value separator, this is the topic's name 
        {
          if(isLetter(a)) //in the case of this parsing function specifically,the topic address is merely a number and a letter. IF the letter is a d, then it is a direction instruction, while s is for speed instruction. We save the direction instruction as the first element in the message array and we save the speed instruction in the second entry.
          {
            if(a == 'd') // hence it is a direction instruction
            {
              INDEX = 0;
            }
            else if(a == 's')
            {
              INDEX = 1;
            }
          }
          else if(isNum(a))
          {
          }
        }
        else if (!seekIndex)
        {
          if(a != '#') 
          {
            m += a;
          }
          else
          {
            messageI[INDEX] = (float)m.toInt()/10.0;
  //          Serial.println(messageI[INDEX]);
  //          analogWrite(13,messageI[INDEX]);
          }
        }
        delay(2);
      }
    }
    return true; //which means that we have just received a message and updated the incoming message array
  }
  else
  {
    return false; //we haven't received any message
  }
  
}

boolean wifi::isNum(char a)
{
  if(a >= 48 && a <= 57) {return true;}
  else {return false;}
}

boolean wifi::isLetter(char a)
{
  if(a >= 97 && a <= 122) {return true;}
  else {return false;}
}


boolean wifi::update()
{
  if(parsePosition())return true; //if we did the parsing and found a new message, then it's true that we've updated the buffer
  else return false; // we haven't updated anything yet
}
