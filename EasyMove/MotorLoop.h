#ifndef MOTORLOOP
#define MOTORLOOP

#include <Wire.h>
#include <Zumo32U4.h>
#include "PID_v1pgt.h"

#define MOVE_TIME 2.0

double Setpoint;
double Inputr, Outputr;
double Inputl, Outputl;
// PID parameters
PID myPIDr(&Inputr, &Outputr, &Setpoint,2.0,7,0.01,P_ON_M, DIRECT);
PID myPIDl(&Inputl, &Outputl, &Setpoint,2.0,7,0.01,P_ON_M, DIRECT);

class MotorLoop{
  Zumo32U4Encoders encoders;
  Zumo32U4Motors motors;
  Zumo32U4LCD lcd;
  
  long desired_right;
  long desired_left;
  long start_right;
  long start_left;
  long start_time;
  int desired_speed;
  long lastDisplayTime;
 

  public:
  
  MotorLoop(){
    desired_right = encoders.getCountsRight(); 
    desired_left = encoders.getCountsLeft(); 
    Setpoint = 0;
    /*Inputr = (float)desired_right;
    Inputl = (float)desired_left; */
    myPIDr.SetOutputLimits(-400,400);
    myPIDr.SetMode(AUTOMATIC);
    myPIDl.SetOutputLimits(-400,400);
    myPIDl.SetMode(AUTOMATIC);
    start_time = millis();
  }

  void setSpeed(int d_speed){ // on a 0-100 scale
    myPIDr.SetOutputLimits(-400 * d_speed / 100, 400 * d_speed / 100);
    myPIDl.SetOutputLimits(-400 * d_speed / 100, 400 * d_speed / 100);
  }

  void gotoPos(long rpos, long lpos){
    desired_right = rpos;
    desired_left = lpos;
    start_time = millis();
    start_right = encoders.getCountsRight();
  }

  bool finished(){
    return abs(encoders.getCountsRight() - desired_right) < 5 &&
       abs(encoders.getCountsLeft() - desired_left) < 5;
  }

  update(){
      

      int16_t cur_right = encoders.getCountsRight();
      int16_t cur_left = encoders.getCountsLeft();

      Serial.print(cur_right);
      Serial.print(",");
      Serial.println(cur_left);
      
      //if(start_time + MOVE_TIME * 1000 > millis()){ // Moving!
        //float desired_curr = (float)(desired_right - start_right) * (float)(millis() - start_time)/MOVE_TIME/1000.0 + (float)start_right;
        Inputr = (float)(cur_right - desired_right);
        Inputl = (float)(cur_left - desired_left);
        myPIDr.Compute();
        myPIDl.Compute();
        motors.setRightSpeed((int)Outputr);
        motors.setLeftSpeed((int)Outputl);
      //}
      /*
      if((uint8_t)(millis() - lastDisplayTime) > 100){
        lcd.clear();
        lcd.print(cur_right);
        lcd.gotoXY(0,1);
        lcd.print((float)(millis() - start_time)/MOVE_TIME/1000.0);
        lastDisplayTime = millis();
      }*/

    }
    
  
};

#endif

