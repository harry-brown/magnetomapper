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
/*----------------------------------------------------------------------------*/

/* Defines -------------------------------------------------------------------*/
// #define DEBUG
/*----------------------------------------------------------------------------*/

/* Threads -------------------------------------------------------------------*/
PROCESS(main_process, "Main process");
AUTOSTART_PROCESSES(&main_process);
/*----------------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
//
/*----------------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
static struct ctimer alive_timer;
/*----------------------------------------------------------------------------*/

/* Main Process --------------------------------------------------------------*/
PROCESS_THREAD(main_process, ev, data)
{

    PROCESS_BEGIN(); //Start of thread

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
                printf("Taking reading");
#ifdef DEBUG
                printf("\n\r");
#endif
            }
        }
    }
    PROCESS_END(); //End of thread
}
/*----------------------------------------------------------------------------*/

/* Initialisation ------------------------------------------------------------*/
static void init(void)
{
    cc26xx_uart_set_input(serial_line_input_byte); //Initalise UART in serial driver
    leds_off(LEDS_ALL);                            //Turn LEDs off.
    ctimer_set(&alive_timer, CLOCK_SECOND / 2, alive_timeout, NULL);
}
/*----------------------------------------------------------------------------*/

/* Alive Timeout -------------------------------------------------------------*/
// Blink an LED to show alive status
static void alive_timeout(void *ptr)
{
    leds_toggle(LEDS_GREEN);
    ctimer_reset(&alive_timer);
}
/*----------------------------------------------------------------------------*/
