//#include <mpu_wrappers.h>
//#include <croutine.h>
//#include <FreeRTOSConfig.h>
//#include <portable.h>
//#include <StackMacros.h>
//#include <event_groups.h>
//#include <queue.h>
//#include <FreeRTOSVariant.h>
//#include <semphr.h>
//#include <projdefs.h>
//#include <list.h>
//#include <portmacro.h>
//#include <Arduino_FreeRTOS.h>
//#include <timers.h>
//#include <task.h>

#include <Bounce2.h>

#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only only one Task is accessing this resource at any time.
//SemaphoreHandle_t xSerialSemaphore;
enum WHEEL
{
  LEFT,
  RIGHT,
  BOTH
};

#define USE_PRINTER_TASK (1)
// define two Tasks for DigitalRead & AnalogRead
void TaskEncoderTicksReadWithDebouncing( void *pvParameters );
void TaskPrintData( void *pvParameters );

#if (1 == USE_PRINTER_TASK)
void TaskRobotTest( void *pvParameters );
#endif

void robot_wheel(WHEEL wheel, int wheel_speed);

struct AMessage
{
  unsigned long ulTickCount_l;
  unsigned long ulTickDeltaTime_l;
  unsigned long ulTickTimeStamp_l;

  unsigned long ulTickCount_r;
  unsigned long ulTickDeltaTime_r;
  unsigned long ulTickTimeStamp_r;

} xMessage;

QueueHandle_t xPrintQueue, xTickQueue;

/*
  DESCRIPTION
  ====================
  Reports through serial (57600 baud) the time since
  a button press (transition from HIGH to LOW).

*/

// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce-Arduino-Wiring



#define BUTTON_PIN 2
#define LED_PIN 13


void setup() {

  Serial.begin(250000);
  Serial.println( "HELLO WORLD !" );

  // Setup the button with an internal pull-up :
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(11, OUTPUT); // For encoder power
  pinMode(12, OUTPUT); // For encoder power
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);

  robot_begin();

  BaseType_t xReturned;

  // Setup the LED :
  pinMode(LED_PIN, OUTPUT);

#if (1 == USE_PRINTER_TASK)
  xPrintQueue = xQueueCreate( 10, sizeof( AMessage) );
#endif

  xTickQueue  = xQueueCreate( 10, sizeof( AMessage) );


  if ( (xPrintQueue != NULL)
#if (1 == USE_PRINTER_TASK)
       && (xTickQueue != NULL)
#endif
     )
  {
    // Now set up two Tasks to run independently.
    xReturned = xTaskCreate(
                  TaskEncoderTicksReadWithDebouncing
                  ,  (const portCHAR *)"EncoderTicksWithDebouncing"  // A name just for humans
                  ,  1024  // This stack size can be checked & adjusted by reading the Stack Highwater
                  ,  NULL
                  ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                  ,  NULL );

    xReturned &= xTaskCreate(
                   TaskRobotTest
                   ,  (const portCHAR *) "RobotRun"
                   ,  128  // Stack size
                   ,  NULL
                   ,  2  // Priority
                   ,  NULL );

#if (1 == USE_PRINTER_TASK)
    xReturned &= xTaskCreate(
                   TaskPrintData
                   ,  (const portCHAR *) "Printer"
                   ,  128  // Stack size
                   ,  NULL
                   ,  3  // Priority
                   ,  NULL );
#endif

    Serial.println( xReturned );


  }



}

void loop() 
{

}

#if (1 == USE_PRINTER_TASK)
void TaskPrintData( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  Serial.println( "TaskPrintData" );
  AMessage xMessage3;

  for (;;) // A Task shall never return or exit.
  {
    xQueueReceive( xPrintQueue, &xMessage3, portMAX_DELAY );

    Serial.print("t");
    Serial.print( xMessage3.ulTickDeltaTime_l );
    Serial.print(",c");
    Serial.println( xMessage3.ulTickCount_l );
  }

}
#endif

void TaskRobotTest( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

#define WHEEL_CYCLE_TEST_COUNT ( 20)
#define WHEEL_CYCLE_TEST_SPEED (200)

  Serial.println( "TaskRobotTest" );
  robot_wheel(LEFT, WHEEL_CYCLE_TEST_SPEED);

  byte bRndSpeed = 0;

  AMessage xMessage2;
  unsigned long ulTicks_l = 0, ulTicks_r = 0;
  unsigned long ulLastTicks_l = 0, ulLastTicks_r = 0;

  unsigned long ulTickDeltaTime_l = 0, ulTickDeltaTime_r = 0;
  unsigned long ulTickLastDeltaTime_l = 0, ulTickLastDeltaTime_r = 0;

  for (;;) // A Task shall never return or exit.
  {
    xQueueReceive( xTickQueue, &xMessage2, portMAX_DELAY );

    ulTicks_l = xMessage2.ulTickCount_l;
    // ulTickDeltaTime_l = xMessage2.ulTickDeltaTime_l;
    
    xMessage2.ulTickCount_l = xMessage2.ulTickCount_l - ulLastTicks_l;
    // xMessage2.ulTickDeltaTime_l = xMessage2.ulTickDeltaTime_l - ulTickDeltaTime_l;

    xQueueSendToFront( xPrintQueue, &( xMessage2 ), 0 );

    if ( (ulTicks_l % WHEEL_CYCLE_TEST_COUNT ) == 0)
    {

      //#if (0 == USE_PRINTER_TASK)
      //#endif
      //robot_wheel(LEFT, 0);
      //vTaskDelay(100);
      bRndSpeed = random(bRndSpeed-50, bRndSpeed+50);
      bRndSpeed = bRndSpeed<150 ? 150 : bRndSpeed;
      bRndSpeed = bRndSpeed>255 ? 255 : bRndSpeed;
      
      Serial.print("<<>>");
      Serial.println(bRndSpeed);
      robot_wheel(LEFT, bRndSpeed );

      ulLastTicks_l = ulTicks_l;
      ulLastTicks_r = ulTicks_r;
      ulTicks_l = 0;

//      ulTickLastDeltaTime_l = ulTickDeltaTime_l;
//      ulTickLastDeltaTime_r = ulTickDeltaTime_r;
//      ulTickDeltaTime_l = 0;
//      

    }
  }

}

void TaskEncoderTicksReadWithDebouncing( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  Serial.println( "TaskEncoderTicksReadWithDebouncing" );

  struct ASample
  {

    Bounce debouncer_l = Bounce();
    Bounce debouncer_r = Bounce();

    unsigned long ulEncoderTickTimeStamp_l = 0;
    unsigned long ulEncoderTickTimeStamp_r = 0;
    unsigned long ulEncoderLastTickTimeStamp_l = 0;
    unsigned long ulEncoderLastTickTimeStamp_r = 0;

    unsigned long ulTickCount_l = 0;
    unsigned long ulTickCount_r = 0;
    unsigned long ulTickDeltaTime_l = 0;
    unsigned long ulTickDeltaTime_r = 0;

  } xSample;

  ASample xSample1;

  xSample1.ulEncoderTickTimeStamp_l = millis();
  xSample1.ulEncoderTickTimeStamp_r = millis();

  xSample1.ulEncoderLastTickTimeStamp_l = millis();
  xSample1.ulEncoderLastTickTimeStamp_r = millis();

  xSample1.debouncer_l.attach(2);
  xSample1.debouncer_r.attach(3);

  xSample1.debouncer_l.interval(5);
  xSample1.debouncer_r.interval(5);

  unsigned long ulTimeStamp = millis();

  AMessage xMessage1;
  randomSeed(analogRead(0));

  boolean hasUpdated = false;

  xMessage1.ulTickCount_l = 0;
  xMessage1.ulTickCount_r = 0;

  xMessage1.ulTickDeltaTime_l = 0;
  xMessage1.ulTickDeltaTime_r = 0;

  for (;;) // A Task shall never return or exit.
  {

    xSample1.debouncer_l.update();
    xSample1.debouncer_r.update();
    ulTimeStamp = millis();
    xMessage1.ulTickTimeStamp_l = ulTimeStamp;
    xMessage1.ulTickTimeStamp_r = ulTimeStamp;

    // Call code if Bounce fell (transition from HIGH to LOW) :
    if ( xSample1.debouncer_l.fell() )
    {
      xSample1.ulEncoderTickTimeStamp_l = ulTimeStamp;
      xSample1.ulTickCount_l++;
      xSample1.ulTickDeltaTime_l = abs(ulTimeStamp - xSample1.ulEncoderLastTickTimeStamp_l);
      xSample1.ulEncoderLastTickTimeStamp_l = ulTimeStamp;

      xMessage1.ulTickCount_l = xSample1.ulTickCount_l;
      xMessage1.ulTickDeltaTime_l = xSample1.ulTickDeltaTime_l ;

      hasUpdated = true;
    }

    if ( xSample1.debouncer_r.fell() )
    {
      xSample1.ulEncoderTickTimeStamp_r = ulTimeStamp;
      xSample1.ulTickCount_r++;
      xSample1.ulTickDeltaTime_r = abs(ulTimeStamp - xSample1.ulEncoderLastTickTimeStamp_r);
      xSample1.ulEncoderLastTickTimeStamp_r = ulTimeStamp;

      xMessage1.ulTickCount_r = xSample1.ulTickCount_r;
      xMessage1.ulTickDeltaTime_r = xSample1.ulTickDeltaTime_r ;

      hasUpdated = true;
    }

    if (true == hasUpdated)
    {
      xQueueSendToFront( xTickQueue, &( xMessage1 ), 0 );

      xMessage1.ulTickCount_l = 0;
      xMessage1.ulTickDeltaTime_l = 0;

      xMessage1.ulTickCount_r = 0;
      xMessage1.ulTickDeltaTime_r = 0;

      hasUpdated = false;
    }

  }

}

