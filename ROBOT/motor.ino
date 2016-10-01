// Motor

// http://www.uctronics.com/smart-robot-car-chassis-kits-2wd-driver-motor-encoder-w-battery-box-for-arduino.html

#define __PI__ (3.1416f)

#define MOTOR_RIGHT_PIN_POS (4)
#define MOTOR_RIGHT_PIN_NEG (5)

#define MOTOR_LEFT_PIN_POS (6)
#define MOTOR_LEFT_PIN_NEG (7)

// unit is mm and radian always

float f_wheel_radius = 32.0f; // mm

float f_speed = 0;
float f_angle = 0;

byte b_speed_R = 0;
byte b_speed_L = 0;

float f_wheel_saparation = 150.0f; // mm

char lDir = 0;

char rDir = 0;



float deg2rad(float _degrees)
{
  float _radians = _degrees * (__PI__/180.0f);
  return _radians;
}

float rad2deg(float _radians)
{
  float _degrees = _radians * (180.0f/__PI__);
  return _degrees;
}

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

void calculate_RL(float _speed, float _angle_deg)
{
    float f_angle = deg2rad(_angle_deg);
    b_speed_R = ( 2.0f * _speed + f_angle * f_wheel_saparation ) / 2.0f * f_wheel_radius ;
    b_speed_L = ( 2.0f * _speed - f_angle * f_wheel_saparation ) / 2.0f * f_wheel_radius ;    
}

void mapSpeeds()
{
  //map(value, fromLow, fromHigh, toLow, toHigh)
  b_speed_R = map(b_speed_R, -100.0f, 100.0f, -255, 255);
  b_speed_L = map(b_speed_L, -100.0f, 100.0f, -255, 255);
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
    analogWrite(MOTOR_RIGHT_PIN_POS, 0);
    analogWrite(MOTOR_RIGHT_PIN_NEG, - b_speed_R);
    rDir = 1;
    lDir = -1;


  }
 else
 {
    analogWrite(MOTOR_RIGHT_PIN_POS, b_speed_R);
    analogWrite(MOTOR_RIGHT_PIN_NEG, 0);
    rDir = -1;
    lDir = 1;

 }

  
  if (b_speed_L < 0) 
  {
    analogWrite(MOTOR_LEFT_PIN_POS, 0);
    analogWrite(MOTOR_LEFT_PIN_NEG, -b_speed_L ); 
    rDir = -1;
    lDir = 1;

  }
  else
  {
    analogWrite(MOTOR_LEFT_PIN_POS, b_speed_L);
    analogWrite(MOTOR_LEFT_PIN_NEG, 0 ); 
    rDir = 1;
    lDir = -1;

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

