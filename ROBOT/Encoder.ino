
/*
enum WHEEL
{
   LEFT,
   RIGHT,
   BOTH
};

*/

volatile long int lCounts, rCounts;
extern char lDir;
extern char rDir;
 
void Encoder_wheelTick(WHEEL wheel);
void Encoder_clearEnc(WHEEL wheel);
long Encoder_getTicks(WHEEL wheel);


void Encoder_begin(int lPin, int rPin)
{
  // RedBot only breaks out ten valid pins:
  //  A0-A5 a.k.a. D14-19 (PCINT 8-13)
  //  D3 (PCINT 19)
  //  D9-D11 (PCINT 1-3)
  // We'll need a whopping case statement to set up the pin change interrupts
  //  for this; in fact, we'll need two, but I'll abstract it to a function.
  //  A call to setPinChangeInterrupt() enables pin change interrupts for that
  //  pin, and pin change interrupts for the group that pin is a part of.
  pinMode(lPin, INPUT);
  pinMode(rPin, INPUT);
  attachInterrupt(lPin, countL, RISING);
  attachInterrupt(rPin, countR, RISING);
  lCounts = 0;
  rCounts = 0;
}

void countR()
{
  Encoder_wheelTick(RIGHT);
}

void countL()
{
  Encoder_wheelTick(LEFT);
}

// This private function changes the counter when a tick happens. The direction
//  is set by the functions that set the motor direction.
void Encoder_wheelTick( WHEEL wheel)
{
  switch(wheel)
  {
    case LEFT:
      lCounts += 1;//(long)lDir;
      break;
    case RIGHT:
      rCounts += 1;//(long)rDir;
      break;
    case BOTH:
      break;
  }
}

// Public function to clear the encoder counts.
void Encoder_clearEnc(WHEEL wheel)
{
  switch(wheel)
  {
    case LEFT:
      lCounts = 0;
      break;
    case RIGHT:
      rCounts = 0;
      break;
    case BOTH:
      lCounts = 0;
      rCounts = 0;
      break;
  }
}

// Public function to read the encoder counts for a given wheel.
long Encoder_getTicks(WHEEL wheel)
{
  switch(wheel)
  {
    case LEFT:
      return lCounts;
    case RIGHT:
      return rCounts;
    case BOTH:
      return 0;
  }
  return 0;
}
