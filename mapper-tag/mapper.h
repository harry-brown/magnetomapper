/**
 * \file
 *         mapper.h
 * \brief
 *         Takes serial input as a prompt, returns magnetometer data
 *         Data will be used to map the magnetic field of a given space
 * \author
 *         Harry Brown
 */
/*---------------------------------------------------------------------------*/
#ifndef MAPPER_H_
#define MAPPER_H_
/* Includes ------------------------------------------------------------------*/
<<<<<<< HEAD
#include "modified-mpu-9250-sensor.h"
=======
#include "mpu-9250-sensor.h"
#include "magnetometer.h"
>>>>>>> 9eaad081c114e2701e2c952930ef439193655af7
#include "contiki.h"
#include "dev/serial-line.h"
#include "dev/cc26xx-uart.h"
#include "dev/leds.h"
#include "sys/ctimer.h"
#include "sensor-common.h"
#include "lib/sensors.h"

#include <stdio.h>
#include <string.h>

/* Function Prototypes -------------------------------------------------------*/
static void init(void);
static void alive_timeout(void *ptr);
static void init_mpu_reading(void *not_used);
static void get_mpu_reading();
static void print_mpu_reading(int reading);

/*---------------------------------------------------------------------------*/
#endif /* MAPPER_H_ */
