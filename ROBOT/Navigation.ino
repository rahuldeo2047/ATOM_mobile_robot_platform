
#define __PI__ (3.1416f)


float f_wheel_radius = 32.0f; // mm

float f_speed = 0;
float f_angle = 0;

float b_speed_R = 0;
float b_speed_L = 0;

float f_wheel_saparation = 150.0f; // mm




float deg2rad(float _degrees)
{
  float _radians = _degrees * (__PI__ / 180.0f);
  return _radians;
}

float rad2deg(float _radians)
{
  float _degrees = _radians * (180.0f / __PI__);
  return _degrees;
}


void calculate_RL(float _speed, float _angle_deg)
{
  float f_angle = deg2rad(_angle_deg);
  
  Serial.println();
  Serial.print("_speed=");
  Serial.print(_speed, 5);
  Serial.print(" f_angle=");
  Serial.print(f_angle, 5);
  
  b_speed_R = ( 2.0f * _speed + f_angle * f_wheel_saparation ) / 2.0f * f_wheel_radius ;
  b_speed_L = ( 2.0f * _speed - f_angle * f_wheel_saparation ) / 2.0f * f_wheel_radius ;
}

void mapSpeeds()
{
  printSpeeds();
  //map(value, fromLow, fromHigh, toLow, toHigh)
  b_speed_R = (byte)map(b_speed_R, -100.0f, 100.0f, -255.0f, 255.0f);
  b_speed_L = (byte)map(b_speed_L, -100.0f, 100.0f, -255.0f, 255.0f);
  printSpeeds();

}

void robot_setSpeed(float _speed)
{
  calculate_RL(_speed, 0.0);
}

void robot_setVelocity(float _speed, float _angle_deg)
{
  calculate_RL(_speed, _angle_deg);
}

void robot_setTurn(float _angle_deg)
{
  calculate_RL(0.0f, _angle_deg);
}

void robot_run()
{

  mapSpeeds();

  if (b_speed_R < 0)
  {
    robot_wheel(RIGHT, -b_speed_R);
  }
  else
  {
    robot_wheel(RIGHT, b_speed_R);
  }


  if (b_speed_L < 0)
  {
    robot_wheel(LEFT, -b_speed_R);
  }
  else
  {
    robot_wheel(LEFT, b_speed_R);
  }

}

void printSpeeds()
{
  Serial.println();
  Serial.print("R=");
  Serial.print(b_speed_R);
  Serial.print(", L=");
  Serial.print(b_speed_L);
}

