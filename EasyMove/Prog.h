#ifndef PROG
#define PROG

#include <Wire.h>
#include <Zumo32U4.h>

#include "MotorLoop.h"

#define MAX_INS 100
#define IN_TO_TICKS 201
#define DEG_TO_TICKS 7.20
struct cmd_t{
  uint8_t cmd;
  float amt;
};

const char start_sound[] PROGMEM = "!c8g8";

class Program{
  cmd_t instructions[MAX_INS];
  uint8_t num_instructions;
  bool running;
  long lastDisplayTime;
  uint8_t cur_step;
  uint16_t lastA;
  MotorLoop motors;
  Zumo32U4Encoders encoders;

  Zumo32U4LCD lcd;
  Zumo32U4Buzzer buzzer;
  Zumo32U4ButtonA buttonA;
  Zumo32U4ButtonC buttonC;

  public:
  Program(){
    num_instructions = 0;
    running = false;
    cur_step = 0;
    lastA = 0;
    motors.setSpeed(75);
  }

  void update(){
    if(buttonA.isPressed() && millis() - lastA > 500){
      running = !running;
      lastA = millis();
    }
    if(buttonC.isPressed()){
      cur_step = 0;
      running = false;
    }
    if((uint8_t)(millis() - lastDisplayTime) > 100){
        lcd.clear();
        lcd.print("St:");
        lcd.print(cur_step);
        lcd.print("/");
        lcd.print(num_instructions);
        lcd.gotoXY(0,1);
        if(running){
          lcd.print("Running!");
        }else{
          lcd.print("-Paused-");
        }
        lastDisplayTime = millis();
    }
    if(running && motors.finished()){ // do next command!
      if(num_instructions == cur_step){ // Done!
        running = false;
        cur_step = 0;
      }else{
        if(cur_step == 0){
          buzzer.playFromProgramSpace(start_sound);
          delay(600);
        }
        delay(100);
        if(instructions[cur_step].cmd == 0){ // forward!
          motors.gotoPos(encoders.getCountsRight() + int(instructions[cur_step].amt * IN_TO_TICKS),
                         encoders.getCountsLeft() + int(instructions[cur_step].amt * IN_TO_TICKS));
        }else if(instructions[cur_step].cmd == 1){ // turn!
          motors.gotoPos(encoders.getCountsRight() + int(instructions[cur_step].amt * DEG_TO_TICKS),
                         encoders.getCountsLeft() - int(instructions[cur_step].amt * DEG_TO_TICKS));
          
        }

        cur_step++;
      }
      
    }
    motors.update();
    delay(1);
  }

  void forward(float distance){
    if(num_instructions >= MAX_INS)return;
    instructions[num_instructions].cmd = 0;
    instructions[num_instructions].amt = distance;
    num_instructions++;
  }

  void reverse(float distance){
    if(num_instructions >= MAX_INS)return;
    instructions[num_instructions].cmd = 0;
    instructions[num_instructions].amt = -distance;
    num_instructions++;
  }

  void right(float deg){
    if(num_instructions >= MAX_INS)return;
    instructions[num_instructions].cmd = 1;
    instructions[num_instructions].amt = -deg;
    num_instructions++;
  }

  void left(float deg){
    if(num_instructions >= MAX_INS)return;
    instructions[num_instructions].cmd = 1;
    instructions[num_instructions].amt = deg;
    num_instructions++;
  }
};
#endif 

