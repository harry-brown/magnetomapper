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
PROCESS(button_input_process, "Button Input process");
AUTOSTART_PROCESSES(&main_process, &mpu_sensor_process, &button_input_process);

/* Variables -----------------------------------------------------------------*/
static uint8_t calibrating = 0;
static uint16_t calibration_samples = 0;
static uint16_t number_samples = 100;

float magBias[3], magScale[6], averageScale;

/* Timers --------------------------------------------------------------------*/
static struct ctimer alive_timer;

static struct MagnetometerCalibration{
    int maxx;
    int minx;
    int maxy;
    int miny;
    int maxz;
    int minz;
} magCalibration = {-10000,10000,-10000,10000,-10000,10000};

/* Initialisation ------------------------------------------------------------*/
static void init(void)
{
    cc26xx_uart_set_input(serial_line_input_byte); //Initalise UART in serial driver
    leds_off(LEDS_ALL);                            //Turn LEDs off.
    ctimer_set(&alive_timer, CLOCK_SECOND / 4, alive_timeout, NULL);
}

/* Alive Timeout -------------------------------------------------------------*/
static void alive_timeout(void *ptr)
{
    leds_toggle(LEDS_GREEN);    // Blink an LED to show alive status
    ctimer_reset(&alive_timer); // Reset 500ms timer
    
}

/* Initiate a reading from the MPU9250 Sensor --------------------------------*/
static void init_mpu_reading(void *not_used)
{
    mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ACC_ALL |
                                                  MPU_9250_SENSOR_TYPE_MAG);
}

/* Called when a reading is received from the MPU9250 ------------------------*/
static int calibrate_mag_reading(int raw, int type)
{
    int calibrated = raw;
    
    switch (type){
        case MPU_9250_SENSOR_TYPE_MAG_X:
            if ((raw > magCalibration.maxx) || (raw < magCalibration.minx)){
                if (raw > magCalibration.maxx) magCalibration.maxx = raw;
                if (raw < magCalibration.minx) magCalibration.minx = raw;
                magBias[0] = ((magCalibration.maxx + magCalibration.minx) / 2);
                magScale[3] = ((magCalibration.maxx - magCalibration.minx) / 2);
                averageScale = (magScale[3] + magScale[4] + magScale[5])/15;
                magScale[0] = averageScale / magScale[3];
                magScale[1] = averageScale / magScale[4];
                magScale[2] = averageScale / magScale[5];
            }
            calibrated = (calibrated - magBias[0]) * magScale[0];
            break;
        case MPU_9250_SENSOR_TYPE_MAG_Y:
            if ((raw > magCalibration.maxy) || (raw < magCalibration.miny)){
                if (raw > magCalibration.maxy) magCalibration.maxy = raw;
                if (raw < magCalibration.miny) magCalibration.miny = raw;
                magBias[1] = ((magCalibration.maxy + magCalibration.miny) / 2);
                magScale[4] = ((magCalibration.maxy - magCalibration.miny) / 2);
                averageScale = (magScale[3] + magScale[4] + magScale[5])/15;
                magScale[0] = averageScale / magScale[3];
                magScale[1] = averageScale / magScale[4];
                magScale[2] = averageScale / magScale[5];
            }
            calibrated = (calibrated - magBias[1]) * magScale[1];
            break;
        case MPU_9250_SENSOR_TYPE_MAG_Z:
            if ((raw > magCalibration.maxz) || (raw < magCalibration.minz)){
                if (raw > magCalibration.maxz) magCalibration.maxz = raw;
                if (raw < magCalibration.minz) magCalibration.minz = raw;
                magBias[2] = ((magCalibration.maxz + magCalibration.minz) / 2);
                magScale[5] = ((magCalibration.maxz - magCalibration.minz) / 2);
                averageScale = (magScale[3] + magScale[4] + magScale[5])/15;
                magScale[0] = averageScale / magScale[3];
                magScale[1] = averageScale / magScale[4];
                magScale[2] = averageScale / magScale[5];
            }
            calibrated = (calibrated - magBias[2]) * magScale[2];
            break;
        default:
            return 0;
            break;
    }

    return calibrated;
}

/* Called when a reading is received from the MPU9250 ------------------------*/
static void get_mpu_reading()
{
    int value;

    printf("{");

    printf("\"accx\": ");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    print_mpu_reading(value);

    printf(", \"accy\": ");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    print_mpu_reading(value);

    printf(", \"accz\": ");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
    print_mpu_reading(value);

    printf(", \"magx\": ");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_X);
    value = calibrate_mag_reading(value, MPU_9250_SENSOR_TYPE_MAG_X);
    print_mpu_reading(value);

    printf(", \"magy\": ");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Y);
    value = calibrate_mag_reading(value, MPU_9250_SENSOR_TYPE_MAG_Y);
    print_mpu_reading(value);

    printf(", \"magz\": ");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Z);
    value = calibrate_mag_reading(value, MPU_9250_SENSOR_TYPE_MAG_Z);
    print_mpu_reading(value);

    printf("}\n");

    SENSORS_DEACTIVATE(mpu_9250_sensor);
}

/* Called when a reading is received from the MPU9250 ------------------------*/
static void calibrate_mag()
{
    int value;
    
    clock_delay_usec(450);
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_X);
    if (value > magCalibration.maxx) magCalibration.maxx = value;
    if (value < magCalibration.minx) magCalibration.minx = value;
    clock_delay_usec(450);
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Y);
    if (value > magCalibration.maxy) magCalibration.maxy = value;
    if (value < magCalibration.miny) magCalibration.miny = value;
    clock_delay_usec(450);
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Z);
    if (value > magCalibration.maxz) magCalibration.maxz = value;
    if (value < magCalibration.minz) magCalibration.minz = value;

    SENSORS_DEACTIVATE(mpu_9250_sensor);
    clock_delay_usec(500);
}

/* Format the data to be displayed -------------------------------------------*/
static void print_mpu_reading(int reading)
{
    if (reading < 0)
    {
        printf("-");
        reading = -reading;
    }

    printf("%d.%02d", reading / 100, reading % 100);
}

/* Main Process --------------------------------------------------------------*/
PROCESS_THREAD(main_process, ev, data)
{

    PROCESS_BEGIN(); //Start of thread

    PRINTF("Initialising Main Process...\n");

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
                init_mpu_reading(NULL);
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

    //Processing loop of thread
    while (1)
    {

        PROCESS_YIELD(); //Let other threads run

        //Check for Thermopile reading
        if (ev == sensors_event && data == &mpu_9250_sensor)
        {
            if (calibrating)
            {
                calibrate_mag();
                if (calibration_samples++ > number_samples){
                    calibrating = 0;
                    magBias[0] = ((magCalibration.maxx + magCalibration.minx) / 2);
                    magBias[1] = ((magCalibration.maxy + magCalibration.miny) / 2);
                    magBias[2] = ((magCalibration.maxz + magCalibration.minz) / 2);

                    magScale[3] = ((magCalibration.maxx - magCalibration.minx) / 2);
                    magScale[4] = ((magCalibration.maxy - magCalibration.miny) / 2);
                    magScale[5] = ((magCalibration.maxz - magCalibration.minz) / 2);

                    averageScale = (magScale[3] + magScale[4] + magScale[5])/15;

                    magScale[0] = averageScale / magScale[3];
                    magScale[1] = averageScale / magScale[4];
                    magScale[2] = averageScale / magScale[5];

                    printf(" Complete!\n");
                }else{
                    init_mpu_reading(NULL);
                }
            }
            else
            {
                get_mpu_reading();
            }
        }
    }
    PROCESS_END(); //End of thread
}

/* Button Input Process ------------------------------------------------------*/
PROCESS_THREAD(button_input_process, ev, data)
{

    PROCESS_BEGIN(); //Start of thread

    //Processing loop of thread
    while (1)
    {

        PROCESS_YIELD(); //Let other threads run

        //Check if sensor event has occured from left button press
        if (ev == sensors_event && data == &button_left_sensor)
        {
            if (calibration_samples > number_samples){
                printf("ping\n");
            }else{
                printf("Calibrating magnetometer...");
                calibrating = 1;
                init_mpu_reading(NULL);
            }
        }
    }

    PROCESS_END(); //End of thread
}
/*---------------------------------------------------------------------------*/
