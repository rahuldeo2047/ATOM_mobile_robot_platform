////#include <mpu_wrappers.h>
////#include <croutine.h>
////#include <FreeRTOSConfig.h>
////#include <portable.h>
////#include <StackMacros.h>
////#include <event_groups.h>
////#include <queue.h>
////#include <FreeRTOSVariant.h>
////#include <semphr.h>
////#include <projdefs.h>
////#include <list.h>
////#include <portmacro.h>
//#include <Arduino_FreeRTOS.h>
////#include <timers.h>
////#include <task.h>
//
//
//#include <Arduino_FreeRTOS.h>
//#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
//
//// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
//// It will be used to ensure only only one Task is accessing this resource at any time.
//SemaphoreHandle_t xSerialSemaphore;


/* 
DESCRIPTION
====================
Reports through serial (57600 baud) the time since 
a button press (transition from HIGH to LOW).

*/

// Include the Bounce2 library found here :
// https://github.com/thomasfredericks/Bounce-Arduino-Wiring
#include <Bounce2.h>

enum WHEEL
{
  LEFT,
  RIGHT,
  BOTH
};

void robot_wheel(WHEEL wheel, int wheel_speed);

#define BUTTON_PIN 2
#define LED_PIN 13

// Instantiate a Bounce object :
Bounce debouncer = Bounce(); 

unsigned long buttonPressTimeStamp;

void setup() {
  
  Serial.begin(250000);
  
  // Setup the button with an internal pull-up :
  pinMode(BUTTON_PIN,INPUT_PULLUP);

  robot_begin();
  robot_wheel(LEFT, 255);

  pinMode(11, OUTPUT); // For encoder power
  pinMode(12, OUTPUT); // For encoder power
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);

  // After setting up the button, setup the Bounce instance :
  debouncer.attach(BUTTON_PIN);
  debouncer.interval(5);
  
  // Setup the LED :
  pinMode(LED_PIN,OUTPUT);
  
}

void loop() {
  
   // Update the Bounce instance :
  debouncer.update();

  // Call code if Bounce fell (transition from HIGH to LOW) :
  if ( debouncer.fell()  ) {;
  
    Serial.println( abs(millis()-buttonPressTimeStamp) );
     buttonPressTimeStamp = millis();
  //robot_wheel(LEFT, 0);
  }
  

}

