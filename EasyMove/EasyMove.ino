/* This program shows how to read the encoders on the Zumo 32U4.
The encoders can tell you how far, and in which direction each
motor has turned.

You can press button A on the Zumo to drive both motors forward
at full speed.  You can press button C to drive both motors
in reverse at full speed.

Encoder counts are printed to the LCD and to the serial monitor.

On the LCD, the top line shows the counts from the left encoder,
and the bottom line shows the counts from the right encoder.
Encoder errors should not happen, but if one does happen then the
buzzer will beep and an exclamation mark will appear temporarily
on the LCD.

In the serial monitor, the first and second numbers represent
counts from the left and right encoders, respectively.  The third
and fourth numbers represent errors from the left and right
encoders, respectively. */

#include "Prog.h"


Program p;



void setup()
{
  //p.forward(5);
  p.forward(15);
  p.right(180);
  //p.right(90);
  p.forward(15);
  p.left(180);
}

void loop()
{
  p.update();
}
