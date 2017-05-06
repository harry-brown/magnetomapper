/**
 * \file
 *         mapper.h
 * \brief
 *         Takes serial input as a prompt, returns magnetometer data
 *         Data will be used to map the magnetic field of a given space
 * \author
 *         Harry Brown
 */


/* Includes ------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/serial-line.h"
#include "dev/cc26xx-uart.h"
#include "dev/leds.h"
#include "sys/ctimer.h"

#include <stdio.h>
#include <string.h>
/*----------------------------------------------------------------------------*/


/* Function Prototypes -------------------------------------------------------*/
static void init(void);
static void alive_timeout(void *ptr);
/*----------------------------------------------------------------------------*/