#ifndef MOTORLOOP
#define MOTORLOOP

#include <Wire.h>
#include <Zumo32U4.h>
#include "PID_v1pgt.h"

#define MOVE_TIME 2.0

#define ACCEL 100 
// Motor increase per second

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
  //long start_right;
  //long start_left;
  //long start_time;
  //int desired_speed;
  //long lastDisplayTime;
  unsigned long last_motor_measure_time;
  int last_left;
  int last_right;
 

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
    last_motor_measure_time = millis();
  }

  void setSpeed(int d_speed){ // on a 0-100 scale
    myPIDr.SetOutputLimits(-400 * d_speed / 100, 400 * d_speed / 100);
    myPIDl.SetOutputLimits(-400 * d_speed / 100, 400 * d_speed / 100);
  }

  void gotoPos(long rpos, long lpos){
    desired_right = rpos;
    desired_left = lpos;
    //start_time = millis();
    //start_right = encoders.getCountsRight();
    last_motor_measure_time = millis();
    last_right = 0;
    last_left = 0;
  }

  bool finished(){
    return abs(encoders.getCountsRight() - desired_right) < 5 &&
       abs(encoders.getCountsLeft() - desired_left) < 5;
  }

  update(){
      
      int16_t cur_right = encoders.getCountsRight();
      int16_t cur_left = encoders.getCountsLeft();

      /*Serial.print(cur_right);
      Serial.print(",");
      Serial.println(cur_left);
      */
      
      Inputr = (float)(cur_right - desired_right);
      Inputl = (float)(cur_left - desired_left);
      myPIDr.Compute();
      myPIDl.Compute();
      int dt = millis() - last_motor_measure_time;
      unsigned long max_delta = ACCEL * dt / 1000; // millis is in ms but ACCEL is per second
      
      
      int leftDelta = ((int)Outputl - last_left);
      int rightDelta = ((int)Outputr - last_right);
      if((int)Outputl - last_left > 1 && (int)Outputl > 0){
        //Serial.print("Left limit");
        motors.setLeftSpeed(last_left + 1);
        last_left = last_left + 1;
      }else if((int)Outputl - last_left < -1 && (int)Outputl < 0){
        //Serial.print("Left limit");
        motors.setLeftSpeed(last_left - 1);
        last_left = last_left - 1;
      }else{
        motors.setLeftSpeed((int)Outputl);
        last_left = (int)Outputl;
      }
      
      if((int)Outputr - last_right > 1 && (int)Outputr > 0){
        motors.setRightSpeed(last_right + 1);
        last_right = last_right + 1;
      }else if((int)Outputr - last_right < -1 && (int)Outputr < 0){
        motors.setRightSpeed(last_right - 1);
        last_right = last_right - 1;
      }else{
        motors.setRightSpeed((int)Outputr);
        last_right = (int)Outputr;
      }
      
      last_motor_measure_time = millis();
    
      delay(5);

    }
    
  
};

#endif
