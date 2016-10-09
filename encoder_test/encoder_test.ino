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
SemaphoreHandle_t xSerialSemaphore;
enum WHEEL
{
  LEFT,
  RIGHT,
  BOTH
};


// define two Tasks for DigitalRead & AnalogRead
void TaskEncoderTicksReadWithDebouncing( void *pvParameters );
void TaskPrintData( void *pvParameters );
//void TaskRobotTest( void *pvParameters );

void robot_wheel(WHEEL wheel, int wheel_speed);

struct AMessage
{
    unsigned long ulTickCount;
    unsigned long ulTickTime;
} xMessage;

QueueHandle_t xPrintQueue;

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


  // Setup the button with an internal pull-up :
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  robot_begin();
  robot_wheel(LEFT, 255);

  pinMode(11, OUTPUT); // For encoder power
  pinMode(12, OUTPUT); // For encoder power
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);


  // Setup the LED :
  pinMode(LED_PIN, OUTPUT);

  xPrintQueue = xQueueCreate( 10, sizeof( AMessage) );

  if ( xPrintQueue != NULL )
  {
    // Now set up two Tasks to run independently.
    xTaskCreate(
      TaskEncoderTicksReadWithDebouncing
      ,  (const portCHAR *)"EncoderTicksWithDebouncing"  // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL );

    xTaskCreate(
      TaskPrintData
      ,  (const portCHAR *) "Printer"
      ,  128  // Stack size
      ,  NULL
      ,  2  // Priority
      ,  NULL );
  }



}

void loop() {

  //   // Update the Bounce instance :
  //  debouncer.update();
  //
  //  // Call code if Bounce fell (transition from HIGH to LOW) :
  //  if ( debouncer.fell()  ) {;
  //
  //
  //  //robot_wheel(LEFT, 0);
  //  }


}


void TaskPrintData( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  AMessage xMessage;
 
  for (;;) // A Task shall never return or exit.
  {
    xQueueReceive( xPrintQueue, &xMessage, portMAX_DELAY );

   Serial.println( xMessage.ulTickTime );

 //   ulEncoderTickDebounceTime = millis();
  }

}

void TaskEncoderTicksReadWithDebouncing( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  // Instantiate a Bounce object :
  Bounce debouncer = Bounce();
  unsigned long ulEncoderTickDebounceTime=0;
  unsigned long ulDebounceTime=0, ulEncoderTicks=0;
  AMessage xMessage;
  
  // After setting up the button, setup the Bounce instance :
  debouncer.attach(2);
  debouncer.interval(5);

  for (;;) // A Task shall never return or exit.
  {
    // Update the Bounce instance :
    debouncer.update();

    // Call code if Bounce fell (transition from HIGH to LOW) :
    if ( debouncer.fell()  )
    {
      xMessage.ulTickCount = ulEncoderTicks++;
      xMessage.ulTickTime = abs(millis() - ulEncoderTickDebounceTime);
      ulEncoderTickDebounceTime = millis();
      xQueueSendToFront( xPrintQueue, &( xMessage ), 0 );
      
      //robot_wheel(LEFT, 0);
    }
  }

}

