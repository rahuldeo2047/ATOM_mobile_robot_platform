



void test_robot()
{
  test_robot_motors_begin();
  test_robot_encoder_begin();
}


void test_robot_motors_begin()
{
  robot_begin();
}

void test_robot_encoder_begin()
{
  Encoder_begin(digitalPinToInterrupt(3), digitalPinToInterrupt(2));
}

void test_encoder_reinit()
{
  Encoder_clearEnc(RIGHT);
  Encoder_clearEnc(LEFT);
}

boolean test_encoder_one_rev(WHEEL wheel, long int test_tick, int _speed)
{
  boolean is_matching = false;
  if(abs(Encoder_getTicks(wheel))>=abs(test_tick))
  {
    is_matching = true;
    robot_wheel(wheel, 0);
  }
  else
  {
    robot_wheel(wheel, _speed);
  }
  return is_matching;
}



void test_encoder_rev()
{
  static long int right_last_tick, left_last_tick;
  
  if( 
    ( Encoder_getTicks(RIGHT) != right_last_tick ) 
    ||
    ( Encoder_getTicks(LEFT) != left_last_tick ) 
    ) 
    {
      right_last_tick = Encoder_getTicks(RIGHT);
      left_last_tick = Encoder_getTicks(LEFT);
      
      Serial.println();
      Serial.print("Er=");
      Serial.print(right_last_tick);
      Serial.print(" El=");
      Serial.print(left_last_tick);
    }
}

void test_robot_motor(WHEEL wheel, int _speed)
{
  Serial.print(" S=");
  Serial.print(_speed);
  digitalToggle(13);
  //robot_setSpeed(255);
  robot_wheel(wheel, _speed);
  print_encoder(wheel);
  
}

#define TEST_SPEED (150)
void test_robot_motors()
{
  Serial.println();
  Serial.print("RIGHT ");
  test_robot_motor(RIGHT, TEST_SPEED);
  test_robot_motor(RIGHT, -TEST_SPEED);
  test_robot_motor(RIGHT, TEST_SPEED);
  test_robot_motor(RIGHT, -TEST_SPEED);
  test_robot_motor(RIGHT, -TEST_SPEED);
  test_robot_motor(RIGHT, TEST_SPEED);
   
  robot_wheel(RIGHT, 0);

  Serial.println();
  Serial.print("LEFT ");
  
  test_robot_motor(LEFT, TEST_SPEED);
  test_robot_motor(LEFT, -TEST_SPEED);
  test_robot_motor(LEFT, TEST_SPEED);
  test_robot_motor(LEFT, -TEST_SPEED);
  test_robot_motor(LEFT, -TEST_SPEED);
  test_robot_motor(LEFT, TEST_SPEED);
  
  digitalToggle(13);
  robot_wheel(LEFT, 0);
  robot_wheel(RIGHT, 0);

delay(1000);
  Serial.println();
  Serial.print("Motor and Encoder test complete.");
  

}

void print_encoder(WHEEL wheel)
{
  Serial.print("|>");
  Serial.print(Encoder_getTicks(wheel));
  Serial.print(" ... ");
  delay(1000);
  Serial.print(Encoder_getTicks(wheel));
  Serial.println("<|");
}
