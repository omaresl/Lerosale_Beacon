/*
 * config.h
 *
 *  Created on: 03/11/2018
 *      Author: Omar Sevilla
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/*============================================================================*
 *  Timer Parameters
 *============================================================================*/

/* Number of timers used in this application */
#define MAX_TIMERS 2

/* First timeout at which the timer has to fire a callback */
#define TIMER_TIMEOUT1 (1 * SECOND)
/* Second timeout at which the timer has to fire a callback */
#define TIMER_TIMEOUT2 (1 * SECOND)

/*============================================================================*
 *  TLV493D Parameters
 *============================================================================*/
#define TLV493D_ADRESS0					(0x3Eu)

#define TLV493D_OFFSET_READREGISTER0	(0u) // BX
#define TLV493D_OFFSET_READREGISTER1	(1u) // BY
#define TLV493D_OFFSET_READREGISTER2	(2u) // BZ
#define TLV493D_OFFSET_READREGISTER3	(3u) // Temp
#define TLV493D_OFFSET_READREGISTER4	(4u) // BX2
#define TLV493D_OFFSET_READREGISTER5	(5u) // BZ2
#define TLV493D_OFFSET_READREGISTER6	(6u) // Temp2

#define TLV493D_READREGISTERS_SIZE		(7u)

/*============================================================================*
 *  Measure Parameters
 *============================================================================*/
#define MEASURE_HISTORY_SIZE	(30u)

#define MEASURE_GAP_THRESHOLD	(10u)

#define MEASURE_LOW_LEVEL_THRESHOLD	(20u)

/*============================================================================*
 *  Battery Parameters
 *============================================================================*/
#define BATTERY_LOW_THRESHOLD	(2700u)

#endif /* CONFIG_H_ */
