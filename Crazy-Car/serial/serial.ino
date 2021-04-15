#include "motorControl.h"

void setup() {
  
  Serial.begin(9600);

}

void controlFunction()
{
  bool state = true;
  int time = 0;
  while(state)
  {
    if (Serial.available() > 0)
    {
      String cmd_control = Serial.readStringUntil('\n');
      if (cmd_control == "F")
      {
        forward_ ();
        delay(2000);
        stop_();
        Serial.println("Forward");  
      }
      else if (cmd_control == "B")
      {
        backward_ ();
        delay(2000);
        stop_();
        Serial.println("Backward");
      }
      else if (cmd_control == "R")
      {
        right_ ();
        delay(2000);
        stop_();
        Serial.println("Right");
      }
      else if (cmd_control == "L")
      {
        left_ ();
        delay(2000);
        stop_();
        Serial.println("Left");
      }
      else if (cmd_control == "LF")
      {
        leftDiagonalForward ();
        delay(2000);
        stop_();
        Serial.println("left Diagonal Forward");
      }
      else if (cmd_control == "RF")
      {
        rightDiagonalForward ();
        delay(2000);
        stop_();
        Serial.println("right Diagonal Forward");
      }
      else if (cmd_control == "RB")
      {
        rightDiagonalReverse ();
        delay(2000);
        stop_();
        Serial.println("right Diagonal Reverse");
      }
      else if (cmd_control == "LB")
      {
        leftDiagonalReverse ();
        delay(2000);
        stop_();
        Serial.println("left Diagonal Reverse");
      }
      else if (cmd_control == "X")
      {
        rotate_ ();
        delay(2000);
        stop_();
        Serial.println("Rotate");
      }
      else if (cmd_control == "S")
      {
        stop_();
        Serial.println("Mode control OFF");
        state = false;
      }
    }
  }
  return;
}

//-------------chua su dung -------------------
void readSensor(){
  bool state = true;
  while(state)
  {
    if (Serial.available() > 0)
    {
      String cmd_sensor = Serial.readStringUntil('\n');
      if (cmd_sensor == "read")
      {
        Serial.println("bum bum");
      }
      else if (cmd_sensor == "stop read")
      {
        Serial.println("Mode sensor OFF");
        state = false;
      }
    }
  }
}
//----------------------------------------------
void loop() {
  if(Serial.available() > 0)
  {
    String bum = Serial.readStringUntil('\n');
  // put your main code here, to run repeatedly:
   // Serial.println(bum);
//    if (bum == "sens or")
  //  {
    //  Serial.println("Mode sensor ON");
      //readSensor();
    //}
    if (bum == "control")
    {
      Serial.println("Mode control ON"); 
      controlFunction();
      
    }
    else if (bum == "die")
    {
      Serial.println("I'm die");
    }
    //delay(1000);
  }

}
