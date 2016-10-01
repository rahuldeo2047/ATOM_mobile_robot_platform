// Robot for slam


//Ext Int input
// Pins 2, 3, 18, 19, 20, 21

//PWM output
// Pins PWM: 2 to 13 and 44 to 46.

// Encoder 2 and 3 ;; Allocated
// Motor 4 5 6 7 ;; Allocated
// Servo 8 9 10 11
// Accel Gyro Ext Int xxxx
// LED indicator 13


enum WHEEL
{
  LEFT,
  RIGHT,
  BOTH
};;



extern void Encoder_begin(int lPin, int rPin);



#define digitalToggleFast(P) *portInputRegister(digitalPinToPort(P)) = digitalPinToBitMask(P)
void digitalToggle(uint8_t P);


void digitalToggle(uint8_t P)
{
  *portInputRegister(digitalPinToPort(P)) = digitalPinToBitMask(P);
}

void _setup() {
  // put your setup code here, to run once:



}

void _loop() {
  // put your main code here, to run repeatedly:
}


#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define two Tasks for DigitalRead & AnalogRead
void TaskDigitalRead( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskRobotTest( void *pvParameters );

//
//void testRobot(float a, float b)
//{
//      digitalToggle(13);
//
//  robot_setVelocity(a, b);
//      printSpeeds();
//      robot_run();
//      printSpeeds();
//          randomSeed(analogRead(0));
//  Serial.print(" | S=");
//  Serial.print((byte)a);
//  Serial.print(" , A=");
//  Serial.print((byte)b);
//
//      Serial.println();
//
//}
// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }

  Serial.println();
  Serial.println("<<HELLO WORLD>>");
  //randomSeed(analogRead(0));
  pinMode(13, OUTPUT);

  //robot_begin();
  //Encoder_begin(digitalPinToInterrupt(20),digitalPinToInterrupt(21));

  test_robot();
  test_robot_motors();

  /*

    while(1);
    {

    }
    while(1)
    {
        testRobot(99,0);
        delay(4000);
    float randomSpeed = random(-100, 100);
         testRobot(randomSpeed,0);
        delay(4000);
      float randomAngle = (float)random(0, 80);
        testRobot(100,randomAngle);
        delay(4000);
    }
  */
  /*
    // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
    // because it is sharing a resource, such as the Serial port.
    // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
    if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
    {
      xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
      if ( ( xSerialSemaphore ) != NULL )
        xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
    }
  */

  /*
    // Now set up two Tasks to run independently.
    xTaskCreate(
      TaskDigitalRead
      ,  (const portCHAR *)"DigitalRead"  // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

    xTaskCreate(
      TaskAnalogRead
      ,  (const portCHAR *) "AnalogRead"
      ,  128  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL );

  */
  /*
    xTaskCreate(
      TaskRobotTest
      ,  (const portCHAR *) "AnalogRead"
      ,  2048  // Stack size
      ,  NULL
      ,  1  // Priority
      ,  NULL );
  */
  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}


extern long Encoder_getTicks(WHEEL wheel);

int last_Tick = 0;

void loop()
{
  //  // Empty. Things are done in Tasks.
  //   int this_tick = Encoder_getTicks(RIGHT);
  //  if(last_Tick!= this_tick)
  //
  //  {
  //     last_Tick = this_tick;
  //    Serial.println(this_tick);
  //  }

}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskDigitalRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
    DigitalReadSerial
    Reads a digital input on pin 2, prints the result to the serial monitor

    This example code is in the public domain.
  */

  // digital pin 2 has a pushbutton attached to it. Give it a name:
  uint8_t pushButton = 2;

  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);

  for (;;) // A Task shall never return or exit.
  {
    // read the input pin:
    int buttonState = digitalRead(pushButton);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      //Serial.println(buttonState);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskAnalogRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      //Serial.println(sensorValue);

      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}


void TaskRobotTest( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  float randomSpeed = 0.0f, randomAngle = 0.0f;
  for (;;)
  {
    // read the input on analog pin 0:
    // int sensorValue = analogRead(A0);

    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    //if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      //Serial.println(sensorValue);
      randomSpeed = random(-100, 100);
      randomAngle = random(-80, 80);

      digitalToggle(13);

      robot_setVelocity(randomSpeed, randomAngle);
      printSpeeds();
      robot_run();
      printSpeeds();
      randomSeed(analogRead(0));
      Serial.println();

      //xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(200);  // one tick delay (15ms) in between reads for stability
  }
}

