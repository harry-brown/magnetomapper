/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup sensortag-cc26xx-mpu
 * @{
 *
 * \file
 *  Driver for the Sensortag Invensense MPU9250 motion processing unit
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "mpu-9250-sensor.c"
#include "magnetometer.h"
#include "sys/rtimer.h"
#include "sensor-common.h"
#include "board-i2c.h"

#include "ti-lib.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Sensor Mag I2C address */
#define SENSOR_MAG_I2C_ADDRESS        0x0C
/* User control register */
#define BIT_ACTL                      0x80
#define BIT_LATCH_EN                  0x20
/*---------------------------------------------------------------------------*/
/* INT Pin / Bypass Enable Configuration */
#define BIT_AUX_IF_EN                 0x20 /* I2C_MST_EN */
#define BIT_BYPASS_EN                 0x02
/* Magnetometer Registers ----------------------------------------------------*/
#define AK8963_ADDRESS   0x0C
#define AK8963_WHO_AM_I  0x00 // should return 0x48
#define AK8963_INFO      0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
#define SENSOR_MAG_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_1, SENSOR_MAG_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Delay */
#define delay_ms(i) (ti_lib_cpu_delay(8000 * (i)))
/*---------------------------------------------------------------------------*/
static uint8_t mpu_config;
static uint8_t acc_range;
static uint8_t acc_range_reg;
static uint8_t val;
static uint8_t interrupt_status;

static float magCalibration[3];
/*---------------------------------------------------------------------------*/
#define SENSOR_STATE_DISABLED     0
#define SENSOR_STATE_BOOTING      1
#define SENSOR_STATE_ENABLED      2

static int state = SENSOR_STATE_DISABLED;
static int elements = MPU_9250_SENSOR_TYPE_NONE;
/*---------------------------------------------------------------------------*/
/* 3 16-byte words for all sensor readings */
#define SENSOR_DATA_BUF_SIZE   3

static uint16_t sensor_value[SENSOR_DATA_BUF_SIZE];
/*---------------------------------------------------------------------------*/
/*
 * Wait SENSOR_BOOT_DELAY ticks for the sensor to boot and
 * SENSOR_STARTUP_DELAY for readings to be ready
 * Gyro is a little slower than Acc
 */
#define SENSOR_BOOT_DELAY     8
#define SENSOR_STARTUP_DELAY  5

static struct ctimer startup_timer;
/*---------------------------------------------------------------------------*/
/* Wait for the MPU to have data ready */
rtimer_clock_t t0;

/*
 * Wait timeout in rtimer ticks. This is just a random low number, since the
 * first time we read the sensor status, it should be ready to return data
 */
#define READING_WAIT_TIMEOUT 10
/*---------------------------------------------------------------------------*/
/**
 * \brief Read data from the magnetometer - X, Y, Z - 3 words
 * \return True if a valid reading could be taken, false otherwise
 */
static bool
mag_read(uint16_t *data)
{
  bool success;
  int timeout = 0;

  if(interrupt_status & BIT_RAW_RDY_EN) {
    /* Select this sensor */
    SENSOR_MAG_SELECT();

    while(!(sensor_common_read_reg(AK8963_ST1, (uint8_t *)data, 1)) & 0x01){
      clock_delay(1);
      if (timeout++ > 1000) {
        printf("1\n\r");
        success = false;
        return success;
      }
    }

    /* Burst read of all magnetometer values */
    success = sensor_common_read_reg(AK8963_XOUT_L, (uint8_t *)data, DATA_SIZE);

    if(success) {
      convert_to_le((uint8_t *)data, DATA_SIZE);
    } else {
      sensor_common_set_error_data((uint8_t *)data, DATA_SIZE);
    }

    SENSOR_DESELECT();
  } else {
    printf("2\n\r");
    success = false;
  }

  return success;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Convert magnetometer raw reading to ________________
 * \param raw_data The raw magnetometer reading
 * \return The converted value
 */
static float
mag_convert(int16_t raw_data, int type)
{
  float value = 0.0;
  switch (type){
    case MPU_9250_SENSOR_TYPE_MAG_X:
      value = raw_data * 10.*4912./3276000.0 * magCalibration[0]; //magic number
      break;
    case MPU_9250_SENSOR_TYPE_MAG_Y:
      value = raw_data * 10.*4912./3276000.0 * magCalibration[1]; //magic number
      break;
    case MPU_9250_SENSOR_TYPE_MAG_Z:
      value = raw_data * 10.*4912./3276000.0 * magCalibration[2]; //magic number
      break;
  }

  return value;
}
/*---------------------------------------------------------------------------*/
static void
magnetometer_initialise(float * calibration)
{
  bool success;
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here

  /* Configure the accelerometer range */
  if((elements & MPU_9250_SENSOR_TYPE_ACC) != 0) {
    acc_set_range(MPU_9250_SENSOR_ACC_RANGE);
  }

  enable_sensor(elements & MPU_9250_SENSOR_TYPE_ALL);

  PRINTF("Initialising Magnetometer...");

  SENSOR_SELECT();

  val = 0x22;
  success = sensor_common_write_reg(INT_PIN_CFG, &val, 1); // Enable Bypass
  clock_delay(10);
  
  SENSOR_MAG_SELECT();

  val = 0x00;
  success = success & sensor_common_write_reg(AK8963_CNTL, &val, 1); // Power down magnetometer  
  clock_delay(10);

  val = 0x0F;
  success = success & sensor_common_write_reg(AK8963_CNTL, &val, 1); // Enter Fuse ROM access mode
  clock_delay(10);
  
  success = success & sensor_common_read_reg(AK8963_ASAX, &rawData[0], 3);  // Read the x-, y-, and z-axis calibration values
  calibration[0] =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
  calibration[1] =  (float)(rawData[1] - 128)/256. + 1.;  
  calibration[2] =  (float)(rawData[2] - 128)/256. + 1.;

  val = 0x00;
  success = success & sensor_common_write_reg(AK8963_CNTL, &val, 1); // Enter Fuse ROM access mode
  clock_delay(10);

  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates  
  val = 0x12;
  success = success & sensor_common_write_reg(AK8963_CNTL, &val, 1); // Set magnetometer data resolution and sample ODR
  clock_delay(10);

  if (success) {
    PRINTF("SUCCESS\n\r");
  }else{
    PRINTF("FAILED\n\r");
  }

  ctimer_set(&startup_timer, SENSOR_STARTUP_DELAY, notify_ready, NULL);
}
/*---------------------------------------------------------------------------*/
static void
magnetometer_power_up(void)
{
  ti_lib_gpio_set_dio(BOARD_IOID_MPU_POWER);
  state = SENSOR_STATE_BOOTING;

  ctimer_set(&startup_timer, SENSOR_BOOT_DELAY, magnetometer_initialise, NULL);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns a reading from the sensor
 * \param type MPU_9250_SENSOR_TYPE_ACC_[XYZ] or MPU_9250_SENSOR_TYPE_GYRO_[XYZ]
 * \return centi-G (ACC) or centi-Deg/Sec (Gyro)
 */
static int
magnetometer_value(int type)
{
  int rv;
  float converted_val = 0;

  if(state == SENSOR_STATE_DISABLED) {
    PRINTF("MPU: Sensor Disabled\n");
    return CC26XX_SENSOR_READING_ERROR;
  }

  memset(sensor_value, 0, sizeof(sensor_value));

  if((type & MPU_9250_SENSOR_TYPE_MAG) != 0) {
    t0 = RTIMER_NOW();

    while(!int_status() &&
          (RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + READING_WAIT_TIMEOUT)));

    rv = mag_read(sensor_value);

    if(rv == 0) {
      PRINTF("MPU: MAG read error\n");
      return CC26XX_SENSOR_READING_ERROR;
    }

    PRINTF("MPU: MAG = 0x%04x 0x%04x 0x%04x = ",
           sensor_value[0], sensor_value[1], sensor_value[2]);

    if(type == MPU_9250_SENSOR_TYPE_MAG_X) {
      converted_val = mag_convert(sensor_value[0], type);
    } else if(type == MPU_9250_SENSOR_TYPE_MAG_Y) {
      converted_val = mag_convert(sensor_value[1], type);
    } else if(type == MPU_9250_SENSOR_TYPE_MAG_Z) {
      converted_val = mag_convert(sensor_value[2], type);
    }
    rv = (int)(converted_val * 100);
  } else {
    PRINTF("MPU: Invalid type\n");
    rv = CC26XX_SENSOR_READING_ERROR;
  }

  PRINTF("%ld\n", (long int)(converted_val * 100));

  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the MPU9250 sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
static int
magnetometer_configure(int type, int enable)
{
  switch(type) {
  case SENSORS_HW_INIT:
    ti_lib_rom_ioc_pin_type_gpio_input(BOARD_IOID_MPU_INT);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_MPU_INT, IOC_IOPULL_DOWN);
    ti_lib_ioc_io_hyst_set(BOARD_IOID_MPU_INT, IOC_HYST_ENABLE);

    ti_lib_rom_ioc_pin_type_gpio_output(BOARD_IOID_MPU_POWER);
    ti_lib_ioc_io_drv_strength_set(BOARD_IOID_MPU_POWER, IOC_CURRENT_4MA,
                                   IOC_STRENGTH_MAX);
    ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
    elements = MPU_9250_SENSOR_TYPE_NONE;
    break;
  case SENSORS_ACTIVE:
    if((enable & MPU_9250_SENSOR_TYPE_ALL) != 0) {
      PRINTF("MPU: Enabling\n");
      elements = enable & MPU_9250_SENSOR_TYPE_ALL;

      magnetometer_power_up();

      state = SENSOR_STATE_BOOTING;
    } else {
      PRINTF("MPU: Disabling\n");
      if(HWREG(GPIO_BASE + GPIO_O_DOUT31_0) & BOARD_MPU_POWER) {
        /* Then check our state */
        elements = MPU_9250_SENSOR_TYPE_NONE;
        ctimer_stop(&startup_timer);
        sensor_sleep();
        while(ti_lib_i2c_master_busy(I2C0_BASE));
        state = SENSOR_STATE_DISABLED;
        ti_lib_gpio_clear_dio(BOARD_IOID_MPU_POWER);
      }
    }
    break;
  default:
    break;
  }
  return state;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the sensor is enabled
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return state;
    break;
  default:
    break;
  }
  return SENSOR_STATE_DISABLED;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(mpu_9250_sensor, "MPU9250", value, magnetometer_configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
