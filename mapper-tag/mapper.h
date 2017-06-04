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
#include "modified-mpu-9250-sensor.h"
#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/resolv.h"
#include "dev/leds.h"
#include "sensortag/board-peripherals.h"
#include "sensortag/cc2650/board.h"
#include "lib/cc26xxware/driverlib/gpio.h"
#include "ti-lib.h"
#include "dev/serial-line.h"
#include "dev/cc26xx-uart.h"
#include "dev/leds.h"
#include "sys/ctimer.h"
#include "sensor-common.h"
#include "lib/sensors.h"
#include "button-sensor.h"
#include "net/ip/uip-debug.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* Function Prototypes -------------------------------------------------------*/
static void init(void);
static void alive_timeout(void *ptr);
static void init_mpu_reading(void *not_used);
static void get_mpu_reading();
static void sprint_mpu_reading(int reading, char* buffer);

/*---------------------------------------------------------------------------*/
#endif /* MAPPER_H_ */
