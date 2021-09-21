/////////Ball and Plate///////////////////////////////
/*
BALL AND PLATE PID CONTROL
*/
///Libraries///


#include <PID_v1.h>
#include "TouchScreen.h"
#include<Servo.h>


// Definitions TOUCH PINS
#define XM A3  
#define YM A2  
#define XP A1   
#define YP A0  

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 711);

// PID values
double Setpoint, Input, Output; //for X
double Setpoint1, Input1, Output1; //for Y
//

// servos variables
Servo servo1; //X axis
Servo servo2; //Y axis  

/////TIME SAMPLE

int Ts = 50; 

//PID const
float Kp = 0.08;                                                     
float Ki = 0.01;                                                      
float Kd = 0.026;

float Kp1 = Kp;                                                       
float Ki1 = Ki;                                                      
float Kd1 = Kd;
 
//INIT PID
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);
PID myPID1(&Input1, &Output1, &Setpoint1,Kp1,Ki1,Kd1, REVERSE);

void setup()
{
  servo1.attach(2);
  servo2.attach(3);
  Output=90;
  Output1=90;
  servo1.write(Output);
  servo2.write(Output1);

  Serial.begin(9600);
  
  //INIT OF TOUSCHSCREEN
   TSPoint p = ts.getPoint();

  //INIT SETPOINT
  Setpoint=510;
  Setpoint1=560;
  
  //// Make plate flat
  servo1.attach(2); 
  servo2.attach(3);
  Output=90;
  Output1=90;
  servo1.write(Output);
  servo2.write(Output1);
  
  // PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(90-25, 90+25);
  myPID1.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(90-25, 90+25);
  
  // TIME SAMPLE
  myPID1.SetSampleTime(Ts); 
  myPID.SetSampleTime(Ts);  
  /////
  delay(50);
 
  ///
 }
 
 
void loop()
{     
   TSPoint p = ts.getPoint();   //measure pressure on plate
     
   if (p.z > ts.pressureThreshhold) //ball is on plate
   {  
 
      TSPoint p = ts.getPoint(); // measure actual position 
      Input=(p.x);  // read X coordinate
      Input1=(p.y); // read Y coordinate
      myPID.Compute();  //action control X compute
      myPID1.Compute(); //   action control  Y compute   
      servo1.write(Output);//control
      servo2.write(Output1);//control 
  }
   else //if there is no ball on plate
  {
     Output=90; //make plate flat
     Output1=90;
     servo1.write(Output); 
     servo2.write(Output1);
    }
   
}
  



