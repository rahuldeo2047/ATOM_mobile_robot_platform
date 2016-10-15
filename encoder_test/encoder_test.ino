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

#if (1 == USE_PRINTER_TASK)
void TaskPrintData( void *pvParameters );
#endif

void TaskRobotTest( void *pvParameters );

void TaskRobotWheelCtrlTest( void *pvParameters );


TaskHandle_t xHandleTaskRobotWheelCtrlTest;



void robot_wheel(WHEEL wheel, int wheel_speed, char*, char *);

struct AMessage
{
  char cTickDir_l;
  long lTickCount_l;
  unsigned long ulTickDeltaTime_l;
  unsigned long ulTickTimeStamp_l;

  char cTickDir_r;
  long lTickCount_r;
  unsigned long ulTickDeltaTime_r;
  unsigned long ulTickTimeStamp_r;

  boolean bResetAll;

} xMessage;

QueueHandle_t xPrintQueue, xTickQueue, xTickDirQueue;

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
  xTickDirQueue  = xQueueCreate( 10, sizeof( AMessage) );


  if (
#if (1 == USE_PRINTER_TASK)
    (xPrintQueue != NULL)
#endif
    && (xTickQueue != NULL)
    && (xTickDirQueue != NULL)

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
                   ,  1024  // Stack size
                   ,  NULL
                   ,  2  // Priority
                   ,  NULL );

    xReturned &= xTaskCreate(
                   TaskRobotWheelCtrlTest
                   ,  (const portCHAR *) "RobotWheelCtrl"
                   ,  1024  // Stack size
                   ,  NULL
                   ,  2  // Priority
                   ,  &xHandleTaskRobotWheelCtrlTest );

#if (1 == USE_PRINTER_TASK)
    xReturned &= xTaskCreate(
                   TaskPrintData
                   ,  (const portCHAR *) "Printer"
                   ,  1024  // Stack size
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

  //Serial.println( "TaskPrintData" );
  AMessage xMessage3;

  for (;;) // A Task shall never return or exit.
  {
    xQueueReceive( xPrintQueue, &xMessage3, portMAX_DELAY );

    //Serial.print("t");
    Serial.print( xMessage3.ulTickTimeStamp_l );
    Serial.print("\t");
    Serial.print( xMessage3.ulTickDeltaTime_l );
    Serial.print("\t");
    Serial.println( xMessage3.lTickCount_l );
  }

}
#endif

#define WHEEL_CYCLE_TEST_SPEED (200)
#define WHEEL_CYCLE_TEST_MAX_SPEED (255)
#define WHEEL_CYCLE_TEST_MIN_SPEED (150)

void TaskRobotWheelCtrlTest( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  //Serial.println( "TaskRobotWheelCtrlTest" );
  AMessage xMessageTickDir;
  char dummy;
  int iRndSpeed = 0;
  char cPosNeg = 0;
  boolean bToggleZeroSpeed = true;


  for (;;) // A Task shall never return or exit.
  {
    iRndSpeed = random(iRndSpeed - 50, iRndSpeed + 50);
    iRndSpeed = iRndSpeed < WHEEL_CYCLE_TEST_MIN_SPEED ? WHEEL_CYCLE_TEST_MIN_SPEED : iRndSpeed;
    iRndSpeed = iRndSpeed > WHEEL_CYCLE_TEST_MAX_SPEED ? WHEEL_CYCLE_TEST_MAX_SPEED : iRndSpeed;

    cPosNeg = (char)random(1, 2) == 1 ? 1 : -1;
    iRndSpeed *= (int)cPosNeg;//random(0,1)==0 ? 1 : -1
    //xMessage2.cTickDir_l = cPosNeg;

    robot_wheel(LEFT, iRndSpeed , &xMessageTickDir.cTickDir_l, &xMessageTickDir.cTickDir_r );
    //Serial.print("<<>>");

    bToggleZeroSpeed = !bToggleZeroSpeed;

    vTaskSuspend( xHandleTaskRobotWheelCtrlTest);
    vTaskDelay(100);
    Serial.print("<<1>> ");
    Serial.println(iRndSpeed);
    xMessageTickDir.bResetAll = true;
    xQueueSendToFront( xTickDirQueue, &( xMessageTickDir ), 0 );
    // Doing xQueueSendToFrontFromISR(...) && no xMessageTickDir.bResetAll = true;
    // is forcing it to be used in the next thread.

    //
  }

}


void TaskRobotTest( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

#define WHEEL_CYCLE_TEST_COUNT ( 20)


  //Serial.println( "TaskRobotTest" );

  char dummy;
  robot_wheel(LEFT, WHEEL_CYCLE_TEST_SPEED, &dummy, &dummy);

  int iRndSpeed = 0;
  char cPosNeg = 0;
  boolean bCheckFirstTick = true;
  //AMessage xMessageTick;
  AMessage xMessageTickDir;

  long lTicks_l = 0, lTicks_r = 0;
  long lLastTicks_l = 0, lLastTicks_r = 0;
  long lFirstTicks_l = 0, lFirstTicks_r = 0;

  unsigned long ulTickDeltaTime_l = 0, ulTickDeltaTime_r = 0;
  unsigned long ulTickLastDeltaTime_l = 0, ulTickLastDeltaTime_r = 0;
  unsigned long lFirstDeltaTime_l = 0, lFirstDeltaTime_r = 0;

  for (;;) // A Task shall never return or exit.
  {
    if (xQueueReceive( xTickQueue, &xMessageTickDir, portMAX_DELAY ))
    {

      lTicks_l = xMessageTickDir.lTickCount_l;
      // ulTickDeltaTime_l = xMessage2.ulTickDeltaTime_l;

      //      if ( true == bCheckFirstTick)
      //      {
      //        lFirstTicks_l = xMessageTickDir.lTickCount_l;
      //        lFirstDeltaTime_l = xMessageTickDir.ulTickDeltaTime_l;
      //        bCheckFirstTick = false;
      //      }
      //
      //      xMessageTickDir.lTickCount_l = xMessageTickDir.lTickCount_l - lLastTicks_l - lFirstTicks_l;
      //      xMessageTickDir.ulTickDeltaTime_l = xMessageTickDir.ulTickDeltaTime_l - ulTickDeltaTime_l;

      //xMessageTickDir = xMessageTick;

      if ( (lTicks_l % WHEEL_CYCLE_TEST_COUNT ) == 0)
      {

        //#if (0 == USE_PRINTER_TASK)
        //#endif
        //robot_wheel(LEFT, 0);
        //vTaskDelay(100);
        iRndSpeed = random(iRndSpeed - 50, iRndSpeed + 50);
        iRndSpeed = iRndSpeed < 150 ? 150 : iRndSpeed;
        iRndSpeed = iRndSpeed > 255 ? 255 : iRndSpeed;

        cPosNeg = (char)random(0, 1) == 0 ? 1 : -1;
        // iRndSpeed *= (int)cPosNeg;//random(0,1)==0 ? 1 : -1
        //xMessage2.cTickDir_l = cPosNeg;

        xMessageTickDir.bResetAll = true;
        robot_wheel(LEFT, 0 , &xMessageTickDir.cTickDir_l, &xMessageTickDir.cTickDir_r );
        Serial.println("[S]");
        //Serial.println(iRndSpeed);
        vTaskResume( xHandleTaskRobotWheelCtrlTest);
        //xMessageTickDir = xMessageTick;

        lLastTicks_l = 0;
        lLastTicks_r = 0;
        lTicks_l = 0;
        lTicks_r = 0;
        bCheckFirstTick = true;

        ulTickLastDeltaTime_l = ulTickDeltaTime_l;
        ulTickLastDeltaTime_r = ulTickDeltaTime_r;
        ulTickDeltaTime_l = 0;


      }

      //xQueueSendToFront( xTickDirQueue, &( xMessageTickDir ), 0 );

      xQueueSendToFront( xPrintQueue, &( xMessageTickDir ), 0 );

    }
  }

}


//Tested for ticks
//Testing for diff speed

void TaskEncoderTicksReadWithDebouncing( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  //Serial.println( "TaskEncoderTicksReadWithDebouncing" );

  struct ASample
  {

    Bounce debouncer_l = Bounce();
    Bounce debouncer_r = Bounce();

    unsigned long ulEncoderTickTimeStamp_l = 0;
    unsigned long ulEncoderTickTimeStamp_r = 0;
    unsigned long ulEncoderLastTickTimeStamp_l = 0;
    unsigned long ulEncoderLastTickTimeStamp_r = 0;

    long lTickCount_l = 0;
    long lTickCount_r = 0;
    unsigned long ulTickDeltaTime_l = 0;
    unsigned long ulTickDeltaTime_r = 0;

    char cTickDir_l = 1;
    char cTickDir_r = 1;

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

  AMessage xMessageTick, xMessageTickDir;
  randomSeed(analogRead(0));

  boolean hasUpdated = false;

  xMessageTick.cTickDir_l = 0;
  xMessageTick.cTickDir_r = 0;

  xMessageTick.lTickCount_l = 0;
  xMessageTick.lTickCount_r = 0;

  xMessageTick.ulTickDeltaTime_l = 0;
  xMessageTick.ulTickDeltaTime_r = 0;

  for (;;) // A Task shall never return or exit.
  {

    xSample1.debouncer_l.update();
    xSample1.debouncer_r.update();
    ulTimeStamp = millis();
    xMessageTick.ulTickTimeStamp_l = ulTimeStamp;
    xMessageTick.ulTickTimeStamp_r = ulTimeStamp;

    //check if any change in motor direction registered
    if (xQueueReceive( xTickDirQueue, &xMessageTickDir, 0 ))
    {
      xSample1.cTickDir_l = xMessageTickDir.cTickDir_l;
      xSample1.cTickDir_r = xMessageTickDir.cTickDir_r;
      if (true == xMessageTickDir.bResetAll)
      {
        xSample1.lTickCount_l = 0;
        //Serial.println("<<2>>");
      }
    }
    else // Valid for test : Will create problem when real use of direction
    {
      xSample1.cTickDir_l = 1;
      xSample1.cTickDir_r = 1;
    }

    //

    // Call code if Bounce fell (transition from HIGH to LOW) :
    if ( xSample1.debouncer_l.fell() )
    {
      //ulTimeStamp = millis();
      xSample1.ulEncoderTickTimeStamp_l = ulTimeStamp;
      xSample1.lTickCount_l += xSample1.cTickDir_l;
      xSample1.ulTickDeltaTime_l = abs(ulTimeStamp - xSample1.ulEncoderLastTickTimeStamp_l);
      xSample1.ulEncoderLastTickTimeStamp_l = ulTimeStamp;

      xMessageTick.lTickCount_l = xSample1.lTickCount_l;
      xMessageTick.ulTickDeltaTime_l = xSample1.ulTickDeltaTime_l ;

      hasUpdated = true;
    }

    if ( xSample1.debouncer_r.fell() )
    {
      //ulTimeStamp = millis();
      xSample1.ulEncoderTickTimeStamp_r = ulTimeStamp;
      xSample1.lTickCount_r += xSample1.cTickDir_r;
      xSample1.ulTickDeltaTime_r = abs(ulTimeStamp - xSample1.ulEncoderLastTickTimeStamp_r);
      xSample1.ulEncoderLastTickTimeStamp_r = ulTimeStamp;

      xMessageTick.lTickCount_r = xSample1.lTickCount_r;
      xMessageTick.ulTickDeltaTime_r = xSample1.ulTickDeltaTime_r ;

      hasUpdated = true;
    }

    if (true == hasUpdated)
    {
      //ulTimeStamp = millis();
      xQueueSendToFront( xTickQueue, &( xMessageTick ), 0 );

      xMessageTick.lTickCount_l = 0;
      xMessageTick.ulTickDeltaTime_l = 0;

      xMessageTick.lTickCount_r = 0;
      xMessageTick.ulTickDeltaTime_r = 0;

      hasUpdated = false;
    }

  }

}

