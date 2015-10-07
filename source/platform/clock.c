
#include "clock.h"
#include "wait_api.h"
#include "rtc_time.h"

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



