/* 
  A simple way to run the Zumo bot in a programmed fashion.  In setup() below, add commands
  for what you want the robot to do.  Once compiled and uploaded the commands will be run in 
  sequence.

  Paul G. Talaga
  Aug 2017
  */

#include "Prog.h"


Program p;

/* Possible commands:
 *    p.forward( distance in inches)
 *    p.reverse( distance in inches)
 *    p.right( angle in degrees)
 *    p.left(  angle in degrees)
 */

void setup()
{
 p.forward(10);
 p.left(90);
 p.left(90);
 p.forward(10);
 p.left(90);
 p.left(90);
 
}

void loop()
{
  // This is the function in the Program class which runs all instructions.
  p.update();
}
