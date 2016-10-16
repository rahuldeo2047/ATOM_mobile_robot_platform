// Motor

// http://www.uctronics.com/smart-robot-car-chassis-kits-2wd-driver-motor-encoder-w-battery-box-for-arduino.html




#define MOTOR_LEFT_PIN_NEG (4)
#define MOTOR_LEFT_PIN_POS (5)

#define MOTOR_RIGHT_PIN_NEG (6)
#define MOTOR_RIGHT_PIN_POS (7)

// unit is mm and radian always

char lDir = 1;

char rDir = 1;

//void setVelocity(float speed)

void robot_begin()
{

  pinMode(MOTOR_RIGHT_PIN_POS, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN_NEG, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_POS, OUTPUT);
  pinMode(MOTOR_LEFT_PIN_NEG, OUTPUT);

  analogWrite(MOTOR_RIGHT_PIN_POS, 0);
  analogWrite(MOTOR_RIGHT_PIN_NEG, 0);
  analogWrite(MOTOR_LEFT_PIN_POS, 0);
  analogWrite(MOTOR_LEFT_PIN_NEG, 0);

}


void robot_wheel(WHEEL wheel, int wheel_speed, char * dir_l, char * dir_r)
{

  //mapSpeeds();

  switch (wheel)
  {
    case RIGHT:
      if (wheel_speed == 0)
      {
        analogWrite(MOTOR_RIGHT_PIN_POS, 0);
        analogWrite(MOTOR_RIGHT_PIN_NEG, 0);
        //rDir = -1;
        //lDir = 1;
        //(*dir_r) = rDir;
      }
      else if (wheel_speed < 0)
      {
        analogWrite(MOTOR_RIGHT_PIN_POS, 0);
        analogWrite(MOTOR_RIGHT_PIN_NEG, (-wheel_speed));
        rDir = -1;
        //lDir = 1;
        (*dir_r) = rDir;
      }
      else
      {
        analogWrite(MOTOR_RIGHT_PIN_POS, (wheel_speed));
        analogWrite(MOTOR_RIGHT_PIN_NEG, 0);
        rDir = 1;
        //lDir = -1;
        (*dir_r) = rDir;
      }
      break;

    case LEFT:
      if (wheel_speed == 0)
      {
        analogWrite(MOTOR_LEFT_PIN_POS, 0);
        analogWrite(MOTOR_LEFT_PIN_NEG, 0);
        //rDir = 1;
        //lDir = -1;
        //(*dir_l) = lDir;
      }
      else if (wheel_speed < 0)
      {
        analogWrite(MOTOR_LEFT_PIN_POS, 0);
        analogWrite(MOTOR_LEFT_PIN_NEG, (-wheel_speed ));
        //rDir = 1;
        lDir = -1;
        (*dir_l) = lDir;
      }
      else
      {
        analogWrite(MOTOR_LEFT_PIN_POS, (wheel_speed));
        analogWrite(MOTOR_LEFT_PIN_NEG, 0 );
        //rDir = -1;
        lDir = 1;
        (*dir_l) = lDir;
      }

      break;

    default : break;
  }





}

void robot_wheel_stop()
{
  char dummy;
  robot_wheel(RIGHT, 0, &dummy, &dummy);
  robot_wheel(LEFT, 0, &dummy, &dummy);
}

