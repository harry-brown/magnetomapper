/**
 * \file
 *         mpu-input.c
 *         Measures the accelerometer's X axis and sends in json format over
 *         serial. Left button pauses data send. 
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
//Prints X axis reading in JSON format with X as tag and random id
static void
print_mpu_reading(int reading)
{
  printf("{\"id\": 4, \"X\":");
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }
 printf("%d.%02d}\n", reading / 100, reading % 100);
}

/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *not_used)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

/*---------------------------------------------------------------------------*/
//If button has been pressed read and print X axis value
static void
get_mpu_reading()
{
  int value;
  clock_time_t next = CLOCK_SECOND/20;

  if(button == 1) {
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    print_mpu_reading(value);
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

