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
#include <timers.h>
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

void TaskRobotNAV( void *pvParameters );

void TaskRobotWheelCtrlTest( void *pvParameters );


TaskHandle_t xHandleTaskRobotWheelCtrlTest;
SemaphoreHandle_t xSemaphoreSample = NULL;


void robot_wheel(WHEEL wheel, int wheel_speed, char*, char *);

struct AMessage
{
  char cTickDir_l;
  long lTickCount_l;
  long ulTickDeltaTime_l;
  long ulTickTimeStamp_l;

  char cTickDir_r;
  long lTickCount_r;
  long ulTickDeltaTime_r;
  long ulTickTimeStamp_r;

  boolean bResetAll;

  boolean bHasMotorSpeedUpdated_l;
  boolean bHasMotorSpeedUpdated_r;

  int iMotorSpeed_l;
  int iMotorSpeed_r;

} xMessage;

#define DEFAULT_AMESSAGE { 0,0,0,0,0,0,0,0,false, false, false, 0, 0 };

QueueHandle_t xPrintQueue, xTickQueue, xTickDirQueue, xMotorQueue, xSampleQueue;

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

  Serial.println(sizeof(AMessage));

#if (1 == USE_PRINTER_TASK)
  xPrintQueue = xQueueCreate( 5, sizeof( AMessage) );
#endif

  xTickQueue  = xQueueCreate( 5, sizeof( AMessage) );
  xTickDirQueue  = xQueueCreate( 5, sizeof( AMessage) );
  xMotorQueue = xQueueCreate( 5, sizeof( AMessage) );
  xSampleQueue = xQueueCreate( 1, sizeof( boolean) );

  if (
#if (1 == USE_PRINTER_TASK)
    (xPrintQueue != NULL)
#endif
    && (xTickQueue != NULL)
    && (xTickDirQueue != NULL)
    && (xMotorQueue != NULL)
    && (xSampleQueue != NULL)
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
    Serial.println( xReturned );

    xReturned &= xTaskCreate(
                   TaskRobotNAV
                   ,  (const portCHAR *) "RobotRun"
                   ,  1024  // Stack size
                   ,  NULL
                   ,  2  // Priority
                   ,  NULL );
    Serial.println( xReturned );

    xReturned &= xTaskCreate(
                   TaskRobotWheelCtrlTest
                   ,  (const portCHAR *) "RobotWheelCtrl"
                   ,  1024  // Stack size
                   ,  NULL
                   ,  2  // Priority
                   ,  &xHandleTaskRobotWheelCtrlTest );
    Serial.println( xReturned );

    TimerHandle_t xBlockTime = xTimerCreate
                               ( /* Just a text name, not used by the RTOS
                     kernel. */
                                 "Timer",
                                 /* The timer period in ticks, must be
                                   greater than 0. */
                                 5,
                                 /* The timers will auto-reload themselves
                                   when they expire. */
                                 pdTRUE,
                                 /* The ID is used to store a count of the
                                   number of times the timer has expired, which
                                   is initialised to 0. */
                                 ( void * ) 0,
                                 /* Each timer calls the same callback when
                                   it expires. */
                                 vTimerCallback
                               );

    xReturned &= xTimerStart(xBlockTime, TickType_t(10));
    Serial.println( xReturned );

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

void vTimerCallback( TimerHandle_t xTimer )
{
  uint32_t ulCount;

  /* Optionally do something if the pxTimer parameter is NULL. */
  configASSERT( pxTimer );

  /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
  ulCount = ( uint32_t ) pvTimerGetTimerID( xTimer );

  /* Increment the count, then test to see if the timer has expired
    ulMaxExpiryCountBeforeStopping yet. */
  ulCount++;

  //xTimerStop( pxTimer, 0 );
  vTimerSetTimerID( xTimer, ( void * ) ulCount );

  boolean bTriggerSample = true;
  xQueueSendToFront( xSampleQueue, &( bTriggerSample ), 0 );
  //Serial.print("T"), Serial.println(ulCount);

}

#if (1 == USE_PRINTER_TASK)
void TaskPrintData( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  //Serial.println( "TaskPrintData" );
  AMessage xMessage3;

  for (;;) // A Task shall never return or exit.
  {
    xMessage3 = DEFAULT_AMESSAGE;
    xQueueReceive( xPrintQueue, &xMessage3, portMAX_DELAY );

    //Serial.print("t");
    Serial.print( xMessage3.ulTickTimeStamp_l );
    Serial.print("\t");
    Serial.print( xMessage3.ulTickDeltaTime_l );
    Serial.print("\t");
    Serial.print( xMessage3.lTickCount_l );
    Serial.print("\t");
    Serial.print( (int)xMessage3.cTickDir_l );
    Serial.print("\t|\t");
    Serial.print( xMessage3.ulTickTimeStamp_r );
    Serial.print("\t");
    Serial.print( xMessage3.ulTickDeltaTime_r );
    Serial.print("\t");
    Serial.print( xMessage3.lTickCount_r );
    Serial.print("\t");
    Serial.println( (int)xMessage3.cTickDir_r );
  }

}
#endif

#define WHEEL_CYCLE_TEST_SPEED (160)
#define WHEEL_CYCLE_TEST_MAX_SPEED (255)
#define WHEEL_CYCLE_TEST_MIN_SPEED (150)

void TaskRobotWheelCtrlTest( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  Serial.println( "TaskRobotWheelCtrlTest" );
  AMessage xMessageTickDir;
  char dummy;
  int iRndSpeed = 0;
  char cPosNeg = 0;
  boolean bToggleZeroSpeed = true;


  for (;;) // A Task shall never return or exit.
  {

    if (xQueueReceive( xMotorQueue, &xMessageTickDir, portMAX_DELAY ))
    {

      iRndSpeed = random(iRndSpeed - 50, iRndSpeed + 50);
      iRndSpeed = iRndSpeed < WHEEL_CYCLE_TEST_MIN_SPEED ? WHEEL_CYCLE_TEST_MIN_SPEED : iRndSpeed;
      iRndSpeed = iRndSpeed > WHEEL_CYCLE_TEST_MAX_SPEED ? WHEEL_CYCLE_TEST_MAX_SPEED : iRndSpeed;

      cPosNeg = (char)random(1, 2) == 1 ? 1 : -1;
      iRndSpeed *= (int)cPosNeg;//random(0,1)==0 ? 1 : -1
      //xMessage2.cTickDir_l = cPosNeg;

      //xMessageTickDir = DEFAULT_AMESSAGE;
      if ( true == xMessageTickDir.bHasMotorSpeedUpdated_l )
      {
        xMessageTickDir.bHasMotorSpeedUpdated_l = false;
        robot_wheel(LEFT, xMessageTickDir.iMotorSpeed_l , &xMessageTickDir.cTickDir_l, &xMessageTickDir.cTickDir_r );
        //Serial.println("[SL^]");
        //xMessageTickDir.bResetAll = true;
        //xQueueSendToFront( xTickDirQueue, &( xMessageTickDir ), 0 );
      }

      if ( true == xMessageTickDir.bHasMotorSpeedUpdated_r )
      {
        xMessageTickDir.bHasMotorSpeedUpdated_r = false;
        robot_wheel(RIGHT, xMessageTickDir.iMotorSpeed_r , &xMessageTickDir.cTickDir_l, &xMessageTickDir.cTickDir_r );
        //Serial.println("[SR^]");
        //xMessageTickDir.bResetAll = true;
        //xQueueSendToFront( xTickDirQueue, &( xMessageTickDir ), 0 );
      }

      xQueueSendToFront( xTickDirQueue, &( xMessageTickDir ), 0 );
      // !? Doing xQueueSendToFrontFromISR(...) && no xMessageTickDir.bResetAll = true;
      // is forcing it to be used in the next thread.

      //
    }
  }

}


TaskHandle_t xHandleTaskStartMotors;
void TaskStartMotors( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  char dummy;
  //for (;;) // A Task shall never return or exit.
  {

    robot_wheel(LEFT, WHEEL_CYCLE_TEST_SPEED, &dummy, &dummy);
    robot_wheel(RIGHT, WHEEL_CYCLE_TEST_SPEED, &dummy, &dummy);
    vTaskDelete(xHandleTaskStartMotors);

  }
}

void TaskRobotNAV( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

#define WHEEL_CYCLE_TEST_COUNT ( 20)


  Serial.println( "TaskRobotNAV" );

  AMessage xMessageTickDir;
  xMessageTickDir = DEFAULT_AMESSAGE;

  xMessageTickDir.bResetAll = false;

  xMessageTickDir.iMotorSpeed_l = -WHEEL_CYCLE_TEST_SPEED;
  xMessageTickDir.bHasMotorSpeedUpdated_l = true;

  xMessageTickDir.iMotorSpeed_r = -WHEEL_CYCLE_TEST_SPEED;
  xMessageTickDir.bHasMotorSpeedUpdated_r = true;

  xQueueSendToFront( xMotorQueue , &xMessageTickDir, portMAX_DELAY );


  // SET POINT
  //
#define SET_POINT__DISTANCE (100.f)

  //  char dummy;
  //  robot_wheel(LEFT, WHEEL_CYCLE_TEST_SPEED, &dummy, &dummy);
  //  robot_wheel(RIGHT, WHEEL_CYCLE_TEST_SPEED, &dummy, &dummy);

  boolean bHasUpdated_l = false;
  boolean bHasUpdated_r = false;
  boolean bHasUpdated = false;

  int iRndSpeed = 0;
  char cPosNeg = 0;
  boolean bCheckFirstTick = true;
  //AMessage xMessageTick;


  long lTicks_l = 0, lTicks_r = 0;
  long lLastTicks_l = 0, lLastTicks_r = 0;
  long lFirstTicks_l = 0, lFirstTicks_r = 0;

  long ulTickDeltaTime_l = 0, ulTickDeltaTime_r = 0;
  long ulTickLastDeltaTime_l = 0, ulTickLastDeltaTime_r = 0;
  long lFirstDeltaTime_l = 0, lFirstDeltaTime_r = 0;

  long ulTimeStamp = millis();

  // Odometry
  //float PI = 3.14159f;
  //float TWO_PI = PI * 2.0f;

  //long lTicks_l = 0, lTicks_r = 0; // mLeftEncoder; mRightEncoder;
  //long lLastTicks_l = 0, lLastTicks_r = 0; //int mPreviousLeftCounts; int mPreviousRightCounts;


  float mX = 0.0f;
  float mY = 0.0f;
  float mHeading = 0.0f;
  //int mPeriod;

  int iDeltaTick_l = 0, iDeltaTick_r = 0;
  long lLastTime = 0;
  long lDeltaTime = 0;

  float fDeltaSpeed = 0;
  float fLastDeltaSpeed = 0;

  float fDeltaAccel = 0;

  float fDeltaSpeed_l = 0, fDeltaSpeed_r = 0;
  float fLastDeltaSpeed_l = 0, fLastDeltaSpeed_r = 0;

  float fDeltaAccel_l = 0, fDeltaAccel_r = 0;


  float fTrackWidth = 150.0f;
  float fDiameterWheel = 64.0f;
  float fCountPerRev = 20.0f;

  // Odometry var
  float fDistancePerCount = PI * fDiameterWheel / fCountPerRev;
  float fTotalDistance = 0;
  float fDeltaDistance = 0; //(leftCounts + rightCounts) / 2.0 * fDistancePerCount;
  float fCountsPerRotation = (fTrackWidth / fDiameterWheel) * fCountPerRev;
  float fRadiansPerCount = PI * (fDiameterWheel / fTrackWidth) / fCountPerRev;
  float fHeading = 0.0f;
  float fDeltaHeading = 0;//(rightCounts leftCounts) * fRadiansPerCount;
  float fDeltaX = fDeltaDistance * cos(fHeading);
  float fDeltaY = fDeltaDistance * sin(fHeading);

  for (;;) // A Task shall never return or exit.
  {

    if ( true == bHasUpdated)
    {
      //#warning loop broken
      xQueueSendToFront( xMotorQueue , &xMessageTickDir, portMAX_DELAY );
      //      xMessageTickDir.iMotorSpeed_r = WHEEL_CYCLE_TEST_SPEED;
      //      xMessageTickDir.bHasMotorSpeedUpdated_r = true;
      //      xQueueSendToFront( xMotorQueue , &xMessageTickDir, portMAX_DELAY );
      //
      //      xMessageTickDir.iMotorSpeed_l = WHEEL_CYCLE_TEST_SPEED;
      //      xMessageTickDir.bHasMotorSpeedUpdated_l = true;
      //      xQueueSendToFront( xMotorQueue , &xMessageTickDir, portMAX_DELAY );
      //

      //robot_wheel(LEFT, WHEEL_CYCLE_TEST_SPEED, &dummy, &dummy);
      //robot_wheel(RIGHT, WHEEL_CYCLE_TEST_SPEED, &dummy, &dummy);

      bHasUpdated = false;

    }

    //xMessageTickDir = DEFAULT_AMESSAGE;
    if (xQueueReceive( xTickQueue, &xMessageTickDir, portMAX_DELAY ))
    {

      lTicks_l = xMessageTickDir.lTickCount_l;
      lTicks_r = xMessageTickDir.lTickCount_r;

      lTicks_l = lTicks_l == 0 ? 1 : lTicks_l;
      lTicks_r = lTicks_r == 0 ? 1 : lTicks_r;

      ////////////////////////////////////////////////////////////////////////////////////

      iDeltaTick_l = lTicks_l - lLastTicks_l;
      iDeltaTick_r = lTicks_l - lLastTicks_l;
      lDeltaTime = ulTimeStamp - lLastTime;

      fDeltaDistance = (iDeltaTick_l + iDeltaTick_r) / 2.0 * fDistancePerCount;
      fTotalDistance += fDeltaDistance;

      fDeltaSpeed = fDeltaDistance / lDeltaTime;
      fDeltaAccel = (fDeltaSpeed - fLastDeltaSpeed) / lDeltaTime;

      if ( 1 < ( abs(iDeltaTick_l) + abs(iDeltaTick_r) ))
      {
        Serial.print(iDeltaTick_l);
        Serial.print( ", ");

        Serial.print(iDeltaTick_r);
        Serial.print( ", ");

        Serial.print(fTotalDistance);
        Serial.print( " | ");


        Serial.print(1000 * fDeltaSpeed);
        Serial.print( " mm/s ");
        Serial.print(fDeltaDistance);
        Serial.println( " mm");
      }

      // Perform Odometry calc
      //fDeltaDistance = (iDeltaTick_l + iDeltaTick_r) / 2.0 * fDistancePerCount;
      //fHeading = 0.0f;
      fDeltaHeading = (-iDeltaTick_l + iDeltaTick_r) * fRadiansPerCount;
      fDeltaX = fDeltaDistance * cos(fHeading);
      fDeltaY = fDeltaDistance * sin(fHeading);

      lLastTicks_l = lTicks_l;
      lLastTicks_r = lTicks_r;

      lLastTime = ulTimeStamp;
      fLastDeltaSpeed = fDeltaSpeed;


      ////////////////////////////////////////////////////////////////////////////////////

      //PID
      if (SET_POINT__DISTANCE < fTotalDistance)
      {

        xMessageTickDir.bResetAll = false;

        xMessageTickDir.iMotorSpeed_l = WHEEL_CYCLE_TEST_MAX_SPEED;
        xMessageTickDir.bHasMotorSpeedUpdated_l = true;



        xMessageTickDir.bResetAll = false;

        xMessageTickDir.iMotorSpeed_r = WHEEL_CYCLE_TEST_MAX_SPEED;
        xMessageTickDir.bHasMotorSpeedUpdated_r = true;

        bHasUpdated = true;

      }
      else
      {
        xMessageTickDir.bResetAll = false;

        xMessageTickDir.iMotorSpeed_l = 0;
        xMessageTickDir.bHasMotorSpeedUpdated_l = true;



        xMessageTickDir.bResetAll = false;

        xMessageTickDir.iMotorSpeed_r = 0;
        xMessageTickDir.bHasMotorSpeedUpdated_r = true;

        bHasUpdated = true;
      }
#warning printer is off
      //xQueueSendToFront( xPrintQueue, &( xMessageTickDir ), 0 );

    }

  }

}


//Tested for ticks
//Testing for diff speed

void TaskEncoderTicksReadWithDebouncing( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  Serial.println( "TaskEncoderTicksReadWithDebouncing" );

  struct ASample
  {

    Bounce debouncer_l = Bounce();
    Bounce debouncer_r = Bounce();

    long ulEncoderTickTimeStamp_l = 0;
    long ulEncoderTickTimeStamp_r = 0;
    long ulEncoderLastTickTimeStamp_l = 0;
    long ulEncoderLastTickTimeStamp_r = 0;

    long lTickCount_l = 0;
    long lTickCount_r = 0;
    long ulTickDeltaTime_l = 0;
    long ulTickDeltaTime_r = 0;

    long lLastTickCount_l = 0;
    long lLastTickCount_r = 0;

    //long lTickVel_l = 0;
    //long lTickVel_r = 0;

    //long lTickAcc_l = 0;
    //long lTickAcc_r = 0;

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

  long ulTimeStamp = millis();

  AMessage xMessageTick, xMessageTickDir;
  randomSeed(analogRead(0));

  boolean hasUpdated = false, bTriggerSample = false;

  xMessageTick.cTickDir_l = 0;
  xMessageTick.cTickDir_r = 0;

  xMessageTick.lTickCount_l = 0;
  xMessageTick.lTickCount_r = 0;

  xMessageTick.ulTickDeltaTime_l = 0;
  xMessageTick.ulTickDeltaTime_r = 0;

  xSample1.cTickDir_l = 1;
  xSample1.cTickDir_r = 1;

  for (;;) // A Task shall never return or exit.
  {

    //xMessageTickDir = DEFAULT_AMESSAGE;
    xSample1.debouncer_l.update();
    xSample1.debouncer_r.update();
    ulTimeStamp = millis();
    xMessageTick.ulTickTimeStamp_l = ulTimeStamp;
    xMessageTick.ulTickTimeStamp_r = ulTimeStamp;

    //    //check if any change in motor direction registered
    if (xQueueReceive( xTickDirQueue, &xMessageTickDir, 0 ))
    {
      xMessageTick = xMessageTickDir;
      xMessageTick.ulTickTimeStamp_l = ulTimeStamp;
      xMessageTick.ulTickTimeStamp_r = ulTimeStamp;

      xSample1.cTickDir_l = xMessageTickDir.cTickDir_l;
      xSample1.cTickDir_r = xMessageTickDir.cTickDir_r;

      //#warning xSample1.cTickDir_l/r = 1;
      //      // Temporary
      //      xSample1.cTickDir_l = 1;
      //      xSample1.cTickDir_r = 1;

      if (true == xMessageTickDir.bResetAll)
      {
        xSample1.lTickCount_l = 0;
        xSample1.lTickCount_r = 0;
        Serial.println("<<2>>");
      }
    }
    //    else // Valid for test : Will create problem when real use of direction
    //    {
    //      xSample1.cTickDir_l = 1;
    //      xSample1.cTickDir_r = 1;
    //    }

    //
    if (xQueueReceive( xSampleQueue, &bTriggerSample, 0 ))
    {
      if (true == bTriggerSample)
      {


        //Serial.println("T^");
        bTriggerSample = false;

        if (true == hasUpdated) // might introduce timing gap : A BUG
        {
          xQueueSendToFront( xTickQueue, &( xMessageTick ), 0 );
          hasUpdated = false;
        }


      }


    }

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



  }

}


