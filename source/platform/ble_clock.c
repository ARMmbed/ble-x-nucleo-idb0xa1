
#include "ble_clock.h"
#ifdef YOTTA_CFG_MBED_OS
	#include "mbed-drivers/wait_api.h"
	#include "mbed-drivers/rtc_time.h"
#else
    #include "wait_api.h"
	#include "rtc_time.h"
#endif

const uint32_t CLOCK_SECOND = 1000;

/*---------------------------------------------------------------------------*/

void Clock_Init(void)
{
  //Not Used
}

/*---------------------------------------------------------------------------*/

tClockTime Clock_Time(void)
{
	return clock();
}

/*---------------------------------------------------------------------------*/
/**
 * Wait for a multiple of 1 ms.
 *
 */
void Clock_Wait(uint32_t i)
{
	wait_ms(i);
}
/*---------------------------------------------------------------------------*/
