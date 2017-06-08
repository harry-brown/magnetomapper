/**
 * \file
 *         mpu-input.c
 *         Measures the accelerometer and Gyro X, Y Z axis and sends in json format over
 *         serial. Left button pauses data read and send. 
 * \author
 *         G Siggins
 */

#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "board-peripherals.h"
#include "mpu-9250-sensor.h"
#include "button-sensor.h"

#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>

/*---------------------------------------------------------------------------*/
PROCESS(mpu_input_process, "Accelerometer process");
AUTOSTART_PROCESSES(&mpu_input_process);
/*---------------------------------------------------------------------------*/

static struct ctimer mpu_timer; 
static void init_mpu_reading(void *not_used);
uint8_t button;

/*----------------------------------------------------------------------------*/
//Prints readinga in JSON format with tags and random id
static void
print_mpu_reading(int Xreading, int Zreading, int Yreading, int Greading, int gy, int gz)
{
  printf("{\"id\": 4, \"X\":");
  if(Xreading < 0) {
    printf("-");
    Xreading = -Xreading;
  }
 printf("%d.%02d, ", Xreading / 100, Xreading % 100);

 printf("\"Z\":");
  if(Zreading < 0) {
    printf("-");
    Zreading = -Zreading;
  }
 printf("%d.%02d, ", Zreading / 100, Zreading % 100);

  printf("\"Y\":");
  if(Yreading < 0) {
    printf("-");
    Yreading = -Yreading;
  }
 printf("%d.%02d, ", Yreading / 100, Yreading % 100);

 printf("\"gY\":");
   if(gy < 0) {
     printf("-");
     gy = -gy;
   }
  printf("%d.%02d, ", gy / 100, gy % 100);

   printf("\"gZ\":");
   if(gz < 0) {
     printf("-");
     gz = -gz;
   }
  printf("%d.%02d, ", gz / 100, gz % 100);

 printf("\"gX\":");
  if(Greading < 0) {
    printf("-");
    Greading = -Greading;
  }
 printf("%d.%02d}\n", Greading / 100, Greading % 100);

}

/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *not_used)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

/*---------------------------------------------------------------------------*/
//If button has been pressed, read and print values
static void
get_mpu_reading()
{
  int Xvalue, Zvalue, Gvalue, Yvalue, GY, GZ;
  clock_time_t next = CLOCK_SECOND/100;

  if(button == 1) {
    Xvalue = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    Zvalue = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
    Yvalue = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    Gvalue = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
    GY = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
    GZ = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
    print_mpu_reading(Xvalue, Zvalue, Yvalue, Gvalue, GY, GZ);
    SENSORS_DEACTIVATE(mpu_9250_sensor);
  }

  ctimer_set(&mpu_timer, next, init_mpu_reading, NULL);
}
/*----------------------------------------------------------------------------*/

PROCESS_THREAD(mpu_input_process, ev, data) {

  PROCESS_BEGIN();	//Start of thread

	leds_off(LEDS_GREEN);		//Turn LED off.
  init_mpu_reading(NULL); //initialise mpu

	//Processing loop of thread
	while (1) {

        //Wait for event to occur
		PROCESS_YIELD();
  		
        //Check if sensor event has occured
		if(ev == sensors_event) {
            
    	if(data == &mpu_9250_sensor) {
        get_mpu_reading();
			}

      if(data == &button_left_sensor) {
        leds_toggle(LEDS_GREEN);    //Toggle LED
        //toggle button value
        if(button == 0){
          button = 1;
        } else {
          button = 0;
        }
      }
		}
  }
  PROCESS_END();		//End of thread
}

