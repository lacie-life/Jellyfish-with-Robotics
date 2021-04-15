#include <MotorDriver.h>

void backward_();
void forward_();
void left_();
void right_();
void leftDiagonalForward();
void leftDiagonalReverse();
void rightDiagonalForward();
void rightDiagonalReverse();
void rotate_();

MotorDriver m;


void setup()
{
  Serial.begin(9600);
}


void loop()
{
  if(Serial.available() >0)
  {
    const String cmd = Serial.readString();
    if(cmd == "f")
    {
      forward_();
    }
    else if (cmd == "b")
    {
      backward_();
    }
    else if (cmd == "l")
    {
      left_();
    }
    else if (cmd == "r")
    {
      right_();
    }
    else if (cmd == "x")
    {
      rotate_();
    }
    else if (cmd == "lf")
    {
      leftDiagonalForward();
    }
    else if (cmd == "lr")
    {
      leftDiagonalReverse();
    }
    else if (cmd == "rr")
    {
      rightDiagonalReverse();
    }
    else if (cmd == "rf")
    {
      rightDiagonalForward();
    }
  }
  delay(3000);
}
void forward_ ()
{
  m.motor(1,FORWARD,100);
  m.motor(2,FORWARD,100);
  m.motor(4,FORWARD,100);
  m.motor(3,FORWARD,100);  
}
void backward_ ()
{
  m.motor(1,BACKWARD,100);
  m.motor(2,BACKWARD,100);
  m.motor(4,BACKWARD,100);
  m.motor(3,BACKWARD,100);   
}
void left_()
{
  m.motor(1,FORWARD,100);
  m.motor(2,BACKWARD,100);
  m.motor(4,BACKWARD,100);
  m.motor(3,FORWARD,100);  
}
void right_ ()
{
  m.motor(2,FORWARD,100);
  m.motor(1,BACKWARD,100);
  m.motor(3,BACKWARD,100);
  m.motor(4,FORWARD,100);    
}
void leftDiagonalForward ()
{
  m.motor(1,FORWARD,100);
  m.motor(3,FORWARD,100);
}
void rightDiagonalForward ()
{
  m.motor(2,FORWARD,100);
  m.motor(4,FORWARD,100);
}
void rotate_ ()
{
  m.motor(1,FORWARD,100);
  m.motor(3,BACKWARD,100);
}
void leftDiagonalReverse ()
{
  m.motor(1,BACKWARD,100);
  m.motor(3,BACKWARD,100);
}
void rightDiagonalReverse ()
{
  m.motor(2,BACKWARD,100);
  m.motor(4,BACKWARD,100);
}
