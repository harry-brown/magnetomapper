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
// #define DEBUG DEBUG_PRINT
#ifdef DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* Threads -------------------------------------------------------------------*/
PROCESS(main_process, "Main process");
PROCESS(mpu_sensor_process, "MPU Sensor process");
PROCESS(udp_client_process, "UDP client process");
PROCESS(button_input_process, "Button Input process");
AUTOSTART_PROCESSES(&main_process, &mpu_sensor_process, &udp_client_process, &button_input_process);

/* Variables -----------------------------------------------------------------*/
static uint16_t calibration_samples = 0;
static uint16_t number_samples = 100;

float magBias[3], magScale[6], averageScale;

enum communicationModes
{
    SERIAL = 1,
    UDP = 2,
    CALIBRATION = 4
} mode;

static struct uip_udp_conn *client_conn;

/* Timers --------------------------------------------------------------------*/
static struct ctimer alive_timer;

static struct MagnetometerCalibration
{
    int maxx;
    int minx;
    int maxy;
    int miny;
    int maxz;
    int minz;
} magCalibration = {40073, 8207, 19496, -11261, -700, -38309};

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
    mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ACC_ALL | MPU_9250_SENSOR_TYPE_MAG);
}

/* Called when a reading is received from the MPU9250 ------------------------*/
static int calibrate_mag_reading(int raw, int type)
{
    int calibrated = raw;

    switch (type)
    {
    case MPU_9250_SENSOR_TYPE_MAG_X:
        calibrated = (calibrated - magBias[0]) * magScale[0];
        break;
    case MPU_9250_SENSOR_TYPE_MAG_Y:
        calibrated = (calibrated - magBias[1]) * magScale[1];
        break;
    case MPU_9250_SENSOR_TYPE_MAG_Z:
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
    char str[160];
    int delay_between_readings = 500;

    sprintf(str, "{");

    sprintf(str + strlen(str), "\"ax\":");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    sprint_mpu_reading(value, str);
    clock_delay_usec(delay_between_readings);

    sprintf(str + strlen(str), ",\"ay\":");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    sprint_mpu_reading(value, str);
    clock_delay_usec(delay_between_readings);

    sprintf(str + strlen(str), ",\"az\":");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
    sprint_mpu_reading(value, str);
    clock_delay_usec(delay_between_readings);

    sprintf(str + strlen(str), ",\"mx\":");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_X);
    value = calibrate_mag_reading(value, MPU_9250_SENSOR_TYPE_MAG_X);
    sprint_mpu_reading(value, str);
    clock_delay_usec(delay_between_readings);

    sprintf(str + strlen(str), ",\"my\":");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Y);
    value = calibrate_mag_reading(value, MPU_9250_SENSOR_TYPE_MAG_Y);
    sprint_mpu_reading(value, str);
    clock_delay_usec(delay_between_readings);

    sprintf(str + strlen(str), ",\"mz\":");
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Z);
    value = calibrate_mag_reading(value, MPU_9250_SENSOR_TYPE_MAG_Z);
    sprint_mpu_reading(value, str);
    clock_delay_usec(delay_between_readings);

    sprintf(str + strlen(str), "}\n");

    printf("printing now\n");

    switch (mode)
    {
    case SERIAL:
        printf("%s", str);
        break;
    case UDP:
        printf("Sending data over UDP: ");
        printf("%s", str);
        printf("\n");
        uip_udp_packet_send(client_conn, str, strlen(str));
        break;
    default:
        printf("error\n");
        break;
    };

    SENSORS_DEACTIVATE(mpu_9250_sensor);
}

/* Called when a reading is received from the MPU9250 ------------------------*/
static void calibrate_mag()
{
    int value;
    clock_delay_usec(500);
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_X);
    if (value > magCalibration.maxx)
        magCalibration.maxx = value;
    if (value < magCalibration.minx)
        magCalibration.minx = value;
    clock_delay_usec(500);
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Y);
    if (value > magCalibration.maxy)
        magCalibration.maxy = value;
    if (value < magCalibration.miny)
        magCalibration.miny = value;
    clock_delay_usec(500);
    value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_MAG_Z);
    if (value > magCalibration.maxz)
        magCalibration.maxz = value;
    if (value < magCalibration.minz)
        magCalibration.minz = value;

    SENSORS_DEACTIVATE(mpu_9250_sensor);

    buzzer_start(1000);
    clock_delay_usec(1000);
    buzzer_stop();
}

/* Format the data to be displayed -------------------------------------------*/
static void sprint_mpu_reading(int reading, char *buffer)
{
    if (reading < 0)
    {
        sprintf(buffer + strlen(buffer), "-");
        reading = -reading;
    }

    sprintf(buffer + strlen(buffer), "%d.%02d", reading / 100, reading % 100);
}

/* Print local address -------------------------------------------------------*/
static void
print_local_addresses(void)
{
    int i;
    uint8_t state;

    PRINTF("Client IPv6 addresses: ");
    for (i = 0; i < UIP_DS6_ADDR_NB; i++)
    {
        state = uip_ds6_if.addr_list[i].state;
        if (uip_ds6_if.addr_list[i].isused &&
            (state == ADDR_TENTATIVE || state == ADDR_PREFERRED))
        {
            PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
            PRINTF("\n\r");
        }
    }
}

/* Set global address --------------------------------------------------------*/
#if UIP_CONF_ROUTER
static void
set_global_address(void)
{
    uip_ipaddr_t ipaddr;

    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}
#endif /* UIP_CONF_ROUTER */

/* Set the connection address ------------------------------------------------*/
static resolv_status_t
set_connection_address(uip_ipaddr_t *ipaddr)
{
#ifndef UDP_CONNECTION_ADDR
#if RESOLV_CONF_SUPPORTS_MDNS
#define UDP_CONNECTION_ADDR       aaaa:0:0:0:0:0:0:1
// #define UDP_CONNECTION_ADDR       aaaa:0:0:0:0212:4b00:07b5:1d03
// #define UDP_CONNECTION_ADDR       contiki-udp-server.local
#elif UIP_CONF_ROUTER
#define UDP_CONNECTION_ADDR       aaaa:0:0:0:0212:7404:0004:0404
#else
#define UDP_CONNECTION_ADDR       fe80:0:0:0:6466:6666:6666:6666
#endif
#endif /* !UDP_CONNECTION_ADDR */

#define _QUOTEME(x) #x
#define QUOTEME(x) _QUOTEME(x)

    resolv_status_t status = RESOLV_STATUS_ERROR;

    if (uiplib_ipaddrconv(QUOTEME(UDP_CONNECTION_ADDR), ipaddr) == 0)
    {
        uip_ipaddr_t *resolved_addr = NULL;
        status = resolv_lookup(QUOTEME(UDP_CONNECTION_ADDR), &resolved_addr);
        if (status == RESOLV_STATUS_UNCACHED || status == RESOLV_STATUS_EXPIRED)
        {
            PRINTF("Attempting to look up %s\n\r", QUOTEME(UDP_CONNECTION_ADDR));
            resolv_query(QUOTEME(UDP_CONNECTION_ADDR));
            status = RESOLV_STATUS_RESOLVING;
        }
        else if (status == RESOLV_STATUS_CACHED && resolved_addr != NULL)
        {
            PRINTF("Lookup of \"%s\" succeded!\n\r", QUOTEME(UDP_CONNECTION_ADDR));
        }
        else if (status == RESOLV_STATUS_RESOLVING)
        {
            PRINTF("Still looking up \"%s\"...\n\r", QUOTEME(UDP_CONNECTION_ADDR));
        }
        else
        {
            PRINTF("Lookup of \"%s\" failed. status = %d\n\r", QUOTEME(UDP_CONNECTION_ADDR), status);
        }
        if (resolved_addr)
            uip_ipaddr_copy(ipaddr, resolved_addr);
    }
    else
    {
        status = RESOLV_STATUS_CACHED;
    }

    return status;
}

/* TCP/IP Message Handler --------------------------------------------------------------*/
static void tcpip_handler(void)
{
    char *str;

    if (uip_newdata())
    {
        str = uip_appdata;
        str[uip_datalen()] = '\0';

        if ((strcmp(str, "poll") == 0) && (mode != CALIBRATION))
        {
            PRINTF("Poll packet received\n");
            init_mpu_reading(NULL);
            mode = UDP;
        }
    }
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
            if ((strcmp(data, "poll") == 0) && (mode != CALIBRATION))
            {
                init_mpu_reading(NULL);
                mode = SERIAL;
            }
            else if (strcmp(data, "cal") == 0)
            {
                printf("minx: %d, maxx: %d, miny: %d, maxy: %d, minz: %d, maxz: %d\n", magCalibration.minx, magCalibration.maxx, magCalibration.miny, magCalibration.maxy, magCalibration.minz, magCalibration.maxz);
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

    magBias[0] = ((magCalibration.maxx + magCalibration.minx) / 2);
    magBias[1] = ((magCalibration.maxy + magCalibration.miny) / 2);
    magBias[2] = ((magCalibration.maxz + magCalibration.minz) / 2);

    magScale[3] = ((magCalibration.maxx - magCalibration.minx) / 2);
    magScale[4] = ((magCalibration.maxy - magCalibration.miny) / 2);
    magScale[5] = ((magCalibration.maxz - magCalibration.minz) / 2);

    averageScale = (magScale[3] + magScale[4] + magScale[5]) / 15;

    magScale[0] = averageScale / magScale[3];
    magScale[1] = averageScale / magScale[4];
    magScale[2] = averageScale / magScale[5];

    //Processing loop of thread
    while (1)
    {

        PROCESS_YIELD(); //Let other threads run

        //Check for Thermopile reading
        if (ev == sensors_event && data == &mpu_9250_sensor)
        {
            if (mode == CALIBRATION)
            {
                calibrate_mag();
                if (calibration_samples++ > number_samples)
                {
                    mode = SERIAL;

                    magBias[0] = ((magCalibration.maxx + magCalibration.minx) / 2);
                    magBias[1] = ((magCalibration.maxy + magCalibration.miny) / 2);
                    magBias[2] = ((magCalibration.maxz + magCalibration.minz) / 2);

                    magScale[3] = ((magCalibration.maxx - magCalibration.minx) / 2);
                    magScale[4] = ((magCalibration.maxy - magCalibration.miny) / 2);
                    magScale[5] = ((magCalibration.maxz - magCalibration.minz) / 2);

                    averageScale = (magScale[3] + magScale[4] + magScale[5]) / 15;

                    magScale[0] = averageScale / magScale[3];
                    magScale[1] = averageScale / magScale[4];
                    magScale[2] = averageScale / magScale[5];

                    printf(" Complete!\n");
                    buzzer_start(10000);
                    clock_delay_usec(20000);
                    buzzer_stop();
                }
                else
                {
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

/* UDP Client Process --------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
    uip_ipaddr_t ipaddr;

    PROCESS_BEGIN();
    PRINTF("UDP client process started\n\r");

#if UIP_CONF_ROUTER
    set_global_address();
#endif

    print_local_addresses(); //Show current address

    static resolv_status_t status = RESOLV_STATUS_UNCACHED;
    while (status != RESOLV_STATUS_CACHED)
    {
        status = set_connection_address(&ipaddr);

        if (status == RESOLV_STATUS_RESOLVING)
        {
            PROCESS_WAIT_EVENT_UNTIL(ev == resolv_event_found);
        }
        else if (status != RESOLV_STATUS_CACHED)
        {
            PRINTF("Can't get connection address.\n\r");
            PROCESS_YIELD();
        }
    }

    /* new connection with remote host */
    client_conn = udp_new(&ipaddr, UIP_HTONS(7005), NULL);
    udp_bind(client_conn, UIP_HTONS(4003));

    PRINTF("Created a great connection with the server ");
    PRINT6ADDR(&client_conn->ripaddr);
    PRINTF(" local/remote port %u/%u\n\r",
           UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

    while (1)
    {
        PROCESS_YIELD();

        if (ev == tcpip_event)
        {
            tcpip_handler();
        }
    }

    PROCESS_END();
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
            //if (calibration_samples > number_samples)
            //{
            printf("ping\n");
            //}
            // else
            // {
            //     //printf("Calibrating magnetometer...");
            //     init_mpu_reading(NULL);
            //     mode = CALIBRATION;
            // }
        }
    }

    PROCESS_END(); //End of thread
}
/*---------------------------------------------------------------------------*/
