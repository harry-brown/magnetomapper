/**
 * \file
 *         mapper.c
 * \brief
 *         Takes serial input as a prompt, returns magnetometer data
 *         Data will be used to map the magnetic field of a given space
 * \author
 *         Harry Brown
 */

/* Includes ------------------------------------------------------------------*/
#include "mapper.h"

/* Defines -------------------------------------------------------------------*/
// #define DEBUG
#ifdef DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Threads -------------------------------------------------------------------*/
PROCESS(main_process, "Main process");
PROCESS(mpu_sensor_process, "MPU Sensor process");
AUTOSTART_PROCESSES(&main_process, &mpu_sensor_process);

/* Variables -----------------------------------------------------------------*/

/* Timers --------------------------------------------------------------------*/
static struct ctimer alive_timer;
static struct ctimer mpu_timer;

/* Main Process --------------------------------------------------------------*/
PROCESS_THREAD(main_process, ev, data)
{

    PROCESS_BEGIN(); //Start of thread

    PRINTF("Initialising Main Process...\n\r");

    init(); //Initialise

    //Processing loop of thread
    while (1)
    {

        PROCESS_YIELD(); //Let other threads run

        //Wait for event triggered by serial input
        if (ev == serial_line_event_message)
        {
            if (strcmp(data, "poll") == 0)
            {
                printf("<measurement text here>");
                PRINTF("\n\r");
            }
        }
    }
    PROCESS_END(); //End of thread
}

/* MPU Sensor Process --------------------------------------------------------*/
PROCESS_THREAD(mpu_sensor_process, ev, data)
{

    PROCESS_BEGIN(); //Start of thread

    PRINTF("Initialising MPU...");

    init_mpu_reading(NULL);

    //Processing loop of thread
    while (1)
    {

        PROCESS_YIELD(); //Let other threads run

        //Check for Thermopile reading
        if (ev == sensors_event && data == &mpu_9250_sensor)
        {
            get_mpu_reading();
        }
    }
    PROCESS_END(); //End of thread
}

/* Initialisation ------------------------------------------------------------*/
static void init(void)
{
    cc26xx_uart_set_input(serial_line_input_byte); //Initalise UART in serial driver
    leds_off(LEDS_ALL);                            //Turn LEDs off.
    ctimer_set(&alive_timer, CLOCK_SECOND / 4, alive_timeout, NULL);
}

/* Alive Timeout -------------------------------------------------------------*/
// Blink an LED to show alive status
static void alive_timeout(void *ptr)
{
    leds_toggle(LEDS_GREEN);

    ctimer_reset(&alive_timer);
}

/*----------------------------------------------------------------------------*/
static void init_mpu_reading(void *not_used)
{
    mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ACC_ALL | MPU_9250_SENSOR_TYPE_MAG);
}
/*----------------------------------------------------------------------------*/
static void get_mpu_reading()
{
  int value;
  clock_time_t next = CLOCK_SECOND / 2;

  printf("MPU Acc: X=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Acc: Y=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Acc: Z=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Mag: X=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_X);
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Mag: Y=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Y);
  print_mpu_reading(value);
  printf(" G\n");

  printf("MPU Mag: Z=");
  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Z);
  print_mpu_reading(value);
  printf(" G\n");

  SENSORS_DEACTIVATE(mpu_9250_sensor);

  ctimer_set(&mpu_timer, next, init_mpu_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void print_mpu_reading(int reading)
{
  if(reading < 0) {
    printf("-");
    reading = -reading;
  }

  printf("%d.%02d", reading / 100, reading % 100);
}
