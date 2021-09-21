#include <PID_v1.h>
#include "TouchScreen.h"
#include<Servo.h>


#define XN A3  
#define YN A2  
#define XP A1   
#define YP A0  

TouchScreen ts = TouchScreen(XP, YP, XN, YN, 711);


double Setpoint, Input, Output;
double Setpoint1, Input1, Output1;
//


Servo servoX;
Servo servoY;



int Ts = 40; 


float Kp = 0.03;                                                     
float Ki = 0.01;                                                      
float Kd = 0.03;

float Kp1 = Kp;                                                       
float Ki1 = Ki;                                                      
float Kd1 = Kd;
 

PID PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, REVERSE);
PID PID1(&Input1, &Output1, &Setpoint1,Kp1,Ki1,Kd1, REVERSE);

int mn=60;
int mx=120;

void setup()
{
  Serial.begin(9600);
  
  Setpoint=600;
  Setpoint1=510;
  

  servoX.attach(2); 
  servoY.attach(3);
  servo1.write(90);
  servo2.write(90);
  

  PID.SetMode(AUTOMATIC);
  PID.SetOutputLimits(mn, mx);
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(mn, mx);
  
  PID1.SetSampleTime(Ts); 
  PID.SetSampleTime(Ts);  
 }
 
 
void loop()
{     
   TSPoint p = ts.getPoint();
   Input=(p.x);  
   Input1=(p.y);  
   if (p.z > ts.pressureThreshhold)
   {    
      PID.Compute();
      PID1.Compute();  
      servo1.write(Output);
      servo2.write(Output1);
  }
   else
  {
     servo1.write(90); 
     servo2.write(90);
    }
   
}
  



