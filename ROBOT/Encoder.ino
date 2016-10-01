
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

void Encoder_begin(int lPin, int rPin);
void Encoder_wheelTick(WHEEL wheel);
void Encoder_clearEnc(WHEEL wheel);
long Encoder_getTicks(WHEEL wheel);


void Encoder_begin(int rPin, int lPin)
{
  pinMode(11, OUTPUT); // For encoder power
  pinMode(12, OUTPUT); // For encoder power
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);

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
  switch (wheel)
  {
    case LEFT:
      lCounts += (long)lDir; // Changes base on motor direction
      break;
    case RIGHT:
      rCounts += (long)rDir; // Changes base on motor direction
      break;
    case BOTH:
      break;
  }
}

// Public function to clear the encoder counts.
void Encoder_clearEnc(WHEEL wheel)
{
  switch (wheel)
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
  switch (wheel)
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
