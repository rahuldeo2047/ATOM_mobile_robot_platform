



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
  Encoder_begin(digitalPinToInterrupt(2), digitalPinToInterrupt(3));
}


void test_robot_motors()
{
  digitalToggle(13);
  //robot_setSpeed(255);
  robot_wheel(RIGHT, 255);
  print_encoder(RIGHT);

  digitalToggle(13);
  //  robot_setSpeed(-255);
  robot_wheel(RIGHT, -255);
  print_encoder(RIGHT);

  digitalToggle(13);
  //robot_setSpeed(255);
  robot_wheel(LEFT, 255);
  print_encoder(LEFT);

  digitalToggle(13);
  //robot_setSpeed(-255);
  robot_wheel(LEFT, -255);
  print_encoder(LEFT);

  digitalToggle(13);
  robot_wheel(LEFT, 0);
  robot_wheel(RIGHT, 0);

}

void print_encoder(WHEEL wheel)
{
  Serial.println();
  Serial.print(Encoder_getTicks(wheel));
  Serial.print(" ... ");
  delay(1000);
  Serial.print(Encoder_getTicks(wheel));

}
