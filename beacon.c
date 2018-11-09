/******************************************************************************
 *  Copyright (C) Cambridge Silicon Radio Limited, 2014
 *
 *  FILE
 *      beacon.c
 *
 *  DESCRIPTION
 *      This file defines an advertising node implementation
 *
 *****************************************************************************/

/*============================================================================*
 *  SDK Header Files
 *============================================================================*/

#include <main.h>
#include <gap_app_if.h>
#include <config_store.h>
#include <pio.h>
#include <random.h>
#include <timer.h>          /* Chip timer functions */
#include <panic.h>          /* Support for applications to panic */
#include <battery.h>
#include "i2c_comms.h"
#include "config.h"
/*============================================================================*
 *  Local Header File
 *============================================================================*/

#include "beacon.h"

/*============================================================================*
 *  Private Definitions
 *============================================================================*/

enum msg_types {
	MSG_TYPE_PERIODIC,
	MSG_TYPE_GAIN,
	MSG_TYPE_LOW_LEVEL,
	MSG_TYPE_LOSE,
	MSG_TYPE_LOW_BATT,
	N_MSG_TYPES
};

/*============================================================================*
 *  Private Data
 *============================================================================*/

/* Declare timer buffer to be managed by firmware library */
static uint16 app_timers[SIZEOF_APP_TIMER * MAX_TIMERS];

static uint8 advData[MAX_ADVERT_PACKET_SIZE];
static uint8 raub_ChipData[TLV493D_READREGISTERS_SIZE];
static uint8 rub_Measure[MEASURE_N_AXIS];
static uint8 raub_MeasureHistory[MEASURE_HISTORY_SIZE];
static uint8 rub_MeasureHistoryIndex = 0u;
static uint8 rub_MsgType = MSG_TYPE_PERIODIC;

static uint8 lub_MaxMeasuredValue = 0u;
static uint8 lub_MinMeasuredValue = 0u;

static uint8 rub_Counter = 0u;

uint8 advPayloadSize;
ls_addr_type addressType = ls_addr_type_public;     /* use public address */

/*============================================================================*
 *  Private Function Prototypes
 *============================================================================*/

static void startAdvertising(void);

static void appSetRandomAddress(void);

/* Start timer */
static void startTimer(uint32 timeout, timer_callback_arg handler);

/* Callback after first timeout */
static void timerCallback1(timer_id const id);
static void timerCallback2(timer_id const id);

/*============================================================================*
 *  Private Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      startTimer
 *
 *  DESCRIPTION
 *      Start a timer
 *
 * PARAMETERS
 *      timeout [in]    Timeout period in seconds
 *      handler [in]    Callback handler for when timer expires
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void startTimer(uint32 timeout, timer_callback_arg handler)
{
	/* Now starting a timer */
	const timer_id tId = TimerCreate(timeout, TRUE, handler);

	/* If a timer could not be created, panic to restart the app */
	if (tId == TIMER_INVALID)
	{
		/* Panic with panic code 0xfe */
		Panic(0xfe);
	}
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      timerCallback1
 *
 *  DESCRIPTION
 *      This function is called when the timer created by TimerCreate expires.
 *      It creates a new timer that will expire after the second timer interval.
 *
 * PARAMETERS
 *      id [in]     ID of timer that has expired
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void timerCallback1(timer_id const id)
{
	uint16 luw_BatteryLevel;

	/* Start broadcasting */
	LsStartStopAdvertise(FALSE, whitelist_disabled, addressType);

	/* clear the existing advertisement data, if any */
	LsStoreAdvScanData(0, NULL, ad_src_advertise);

//	rub_Counter++;
	advData[13] = rub_Counter; // Counter Increase

	luw_BatteryLevel = BatteryReadVoltage(); // Battery Level Update

	//GAS IDE
	advData[3] = 0xAA;

	//MAC
	advData[4] = 0x01;
	advData[5] = 0x03;
	advData[6] = 0x20;
	advData[7] = 0x16;

	//Battery Level
	advData[11] = (uint8)(luw_BatteryLevel >> 8u);
	advData[12] = (uint8)(luw_BatteryLevel);

	if(FALSE)//MSG_TYPE_PERIODIC != rub_MsgType)
	{
		if(MSG_TYPE_LOSE == rub_MsgType)
		{
			// D0 - Level
			advData[9] = lub_MaxMeasuredValue;

			//D1 - void
			advData[10] = lub_MinMeasuredValue;
		}
		else if(MSG_TYPE_GAIN == rub_MsgType)
		{
			// D0 - Level
			advData[9] = lub_MinMeasuredValue;

			//D1 - void
			advData[10] = lub_MaxMeasuredValue;
		}
		else
		{
			//This should not be reached
		}
	}
	else
	{
		if(rub_Measure[MEASURE_X_AXIS] < MEASURE_LOW_LEVEL_THRESHOLD)
		{
			rub_MsgType = MSG_TYPE_LOW_LEVEL;
		}
		else
		{
			if(luw_BatteryLevel <= BATTERY_LOW_THRESHOLD)
			{
				rub_MsgType = MSG_TYPE_LOW_BATT;
			}
			else
			{
				//Keep Periodic Message
			}
		}



		//Msg Type
		advData[8] = rub_MsgType;

		// D0 - Level
		advData[9] = rub_Measure[MEASURE_X_AXIS];

		//D1 - void
		advData[10] = rub_Measure[MEASURE_Y_AXIS];

	}

	/* store the advertisement data */
	LsStoreAdvScanData(advPayloadSize + 3, advData, ad_src_advertise);

	/* Start broadcasting */
	LsStartStopAdvertise(TRUE, whitelist_disabled, addressType);

	/* Now start a new timer for second callback */
	startTimer((TIMER_TIMEOUT1), timerCallback1);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      timerCallback2
 *
 *  DESCRIPTION
 *      This function is called when the timer created by TimerCreate expires.
 *      It creates a new timer that will expire after the second timer interval.
 *
 * PARAMETERS
 *      id [in]     ID of timer that has expired
 *
 * RETURNS
 *      Nothing
 *----------------------------------------------------------------------------*/
static void timerCallback2(timer_id const id)
{
	uint8 lub_i;
	uint8 lub_GainCounter;
	uint8 lub_LoseCounter;
	static uint8 lub_CommRes;

	lub_GainCounter = 0u;
	lub_LoseCounter = 0u;
	lub_MaxMeasuredValue = 0u;
	lub_MinMeasuredValue = 0u;

	/* Acquire I2C bus */
	if(I2CAcquire())
	{
		/* Initialise I2C communication. */
		I2CcommsInit();

		TimeDelayUSec(250);

		/* Start Conversion */
		lub_CommRes = I2CWriteRegister(0x3E,0x00,0x82u);

		/* The sensor takes around 250us for transition from OFF state to stand
		 * by state.
		 */
		TimeDelayUSec(10000);

		/* Stop Conversion */
		lub_CommRes = I2CWriteRegister(0x3E,0x00,0x00u);

		lub_CommRes = I2CReadRegisters(0x3Eu,0x00u,3u,raub_ChipData);

		/* Release the I2C bus */
		I2CRelease();
	}

	if(TRUE == lub_CommRes)
	{
		rub_Counter++;
	}
	else
	{

	}

	rub_Measure[MEASURE_X_AXIS] = raub_ChipData[TLV493D_OFFSET_READREGISTER0];
	rub_Measure[MEASURE_Y_AXIS] = raub_ChipData[TLV493D_OFFSET_READREGISTER1];

	/* Update History */
	raub_MeasureHistory[rub_MeasureHistoryIndex] = rub_Measure[MEASURE_X_AXIS];

	//Check if history buffer has been filled
	if(rub_MeasureHistoryIndex < MEASURE_HISTORY_SIZE)
	{
		rub_MeasureHistoryIndex++;
	}
	else
	{//History Buffer filled, clear index
		rub_MeasureHistoryIndex = 0u;
	}

	/* Message Type Selection */
	/* Search for Max and Min value in history */
	for(lub_i = 0u; lub_i < MEASURE_HISTORY_SIZE; lub_i++)
	{
		//Max
		if(raub_MeasureHistory[lub_i] > lub_MaxMeasuredValue)
		{
			lub_MaxMeasuredValue = raub_MeasureHistory[lub_i];
		}
		else
		{
			//Keep max value unchanged
		}

		//Min
		if(raub_MeasureHistory[lub_i] < lub_MinMeasuredValue)
		{
			lub_MaxMeasuredValue = raub_MeasureHistory[lub_i];
		}
		else
		{
			//Keep min value unchanged
		}

		/* Check gain direction */
		if(lub_i > 1u)
		{
			if(raub_MeasureHistory[lub_i] > raub_MeasureHistory[lub_i - 1u])
			{
				lub_GainCounter++;
			}
			else if(raub_MeasureHistory[lub_i] < raub_MeasureHistory[lub_i - 1u])
			{
				lub_LoseCounter++;
			}
			else
			{
				//equal values
			}
		}
		else
		{
			//Do not check
		}
	}

	if((lub_MaxMeasuredValue - lub_MinMeasuredValue) > MEASURE_GAP_THRESHOLD)
	{
		if(lub_GainCounter > lub_LoseCounter)
		{
			//Gaining
			rub_MsgType = MSG_TYPE_GAIN;
		}
		else if(lub_LoseCounter > lub_GainCounter)
		{
			//Loosing
			rub_MsgType = MSG_TYPE_LOSE;
		}
		else
		{
			//Stable - This condition should not be executed
			rub_MsgType = MSG_TYPE_PERIODIC;
		}

	}
	else
	{
		// Keep Periodic Message running
		rub_MsgType = MSG_TYPE_PERIODIC;
	}

	/* Now start a new timer for second callback */
	startTimer((TIMER_TIMEOUT2), timerCallback2);
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      appSetRandomAddress
 *
 *  DESCRIPTION
 *      This function generates a non-resolvable private address and sets it
 *      to the firmware.
 *
 *  RETURNS
 *      Nothing.
 *
 *----------------------------------------------------------------------------*/
static void appSetRandomAddress(void)
{
	BD_ADDR_T addr;

	/* "completely" random MAC addresses by default: */
	for(;;)
	{
		uint32 now = TimeGet32();
		/* Random32() is just two of them, no use */
		uint32 rnd = Random16();
		addr.uap = 0xff & (rnd ^ now);
		/* No sub-part may be zero or all-1s */
		if ( 0 == addr.uap || 0xff == addr.uap ) continue;
		addr.lap = 0xffffff & ((now >> 8) ^ (73 * rnd));
		if ( 0 == addr.lap || 0xffffff == addr.lap ) continue;
		addr.nap = 0x3fff & rnd;
		if ( 0 == addr.nap || 0x3fff == addr.nap ) continue;
		break;
	}

	/* Set it to actually be an acceptable random address */
	addr.nap &= ~BD_ADDR_NAP_RANDOM_TYPE_MASK;
	addr.nap |=  BD_ADDR_NAP_RANDOM_TYPE_NONRESOLV;
	GapSetRandomAddress(&addr);
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      startAdvertising
 *
 *  DESCRIPTION
 *      This function is called to start advertisements.
 *
 *      Advertisement packet will contain Flags AD and Manufacturer-specific
 *      AD with Manufacturer id set to CSR and payload set to the value of
 *      the User Key 0. The payload size is set by the User Key 1.
 *
 *      +--------+-------------------------------------------------+
 *      |FLAGS AD|MANUFACTURER AD                                  |
 *      +--------+-------------------------------------------------+
 *       0      2 3
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/
void startAdvertising(void)
{
	uint16 offset = 0;
	uint8 filler;
	uint16 advInterval;

	/* initialise values from User CsKeys */

	/* read User key 0 for the payload filler */
	filler = (uint8)(CSReadUserKey(0) & 0x00FF);

	/* read User key 1 for the payload size */
	advPayloadSize = (uint8)(CSReadUserKey(1) & 0x00FF);

	/* range check */
	if((advPayloadSize < 1) || (advPayloadSize > MAX_ADVERT_PAYLOAD_SIZE))
	{
		/* revert to default payload size */
		advPayloadSize = DEFAULT_ADVERT_PAYLOAD_SIZE;
	}

	/* read User key 2 for the advertising interval */
	advInterval = CSReadUserKey(2);

	/* range check */
	if((advInterval < MIN_ADVERTISING_INTERVAL) ||
			(advInterval > MAX_ADVERTISING_INTERVAL))
	{
		/* revert to default advertising interval */
		advInterval = DEFAULT_ADVERTISING_INTERVAL;
	}

	/* read address type from User key 3 */
	if(CSReadUserKey(3))
	{
		/* use random address type */
		addressType = ls_addr_type_random;

		/* generate and set the random address */
		appSetRandomAddress();
	}

	/* set the GAP Broadcaster role */
	GapSetMode(gap_role_broadcaster,
			gap_mode_discover_no,
			gap_mode_connect_no,
			gap_mode_bond_no,
			gap_mode_security_none);

	/* clear the existing advertisement data, if any */
	LsStoreAdvScanData(0, NULL, ad_src_advertise);

	/* set the advertisement interval, API accepts the value in microseconds */
	GapSetAdvInterval(advInterval * MILLISECOND, advInterval * MILLISECOND);

	/* manufacturer-specific data */
	advData[0] = AD_TYPE_MANUF;

	/* CSR company code, little endian */
	advData[1] = 0x0A;
	advData[2] = 0x00;

	//	/* Counter */
	//	advData[3] = 0x00u;
	//
	//	luw_BatteryLevel = BatteryReadVoltage();
	//
	//	advData[4] = (uint8)(luw_BatteryLevel >> 8u);
	//	advData[5] = (uint8)(luw_BatteryLevel);



	/* fill in the rest of the advertisement */
	for(offset = 0; offset < (advPayloadSize - 3); offset++)
	{
		advData[3 + offset] = filler;
	}

	//	/* store the advertisement data */
	//	LsStoreAdvScanData(advPayloadSize + 3, advData, ad_src_advertise);
	//
	//	/* Start broadcasting */
	//	LsStartStopAdvertise(TRUE, whitelist_disabled, addressType);
}


/*============================================================================*
 *  Public Function Implementations
 *============================================================================*/

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppPowerOnReset
 *
 *  DESCRIPTION
 *      This function is called just after a power-on reset (including after
 *      a firmware panic).
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppPowerOnReset(void)
{
	/* empty */
}

/*----------------------------------------------------------------------------*
 *  NAME
 *      AppInit
 *
 *  DESCRIPTION
 *      This function is called after a power-on reset (including after a
 *      firmware panic) or after an HCI Reset has been requested.
 *
 *      NOTE: In the case of a power-on reset, this function is called
 *      after AppPowerOnReset().
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppInit(sleep_state last_sleep_state)
{
	/* Timer Init */
	TimerInit(MAX_TIMERS, (void *)app_timers);

	rub_Counter = 0u;

	/* set all PIOs to inputs and pull them down */
	PioSetModes(0xFFFFFFFFUL, pio_mode_user);
	PioSetDirs(0xFFFFFFFFUL, FALSE);
	PioSetPullModes(0xFFFFFFFFUL, pio_mode_strong_pull_down);

	/* disable wake up on UART RX */
	SleepWakeOnUartRX(FALSE);

	/* Acquire I2C bus */
	if(I2CAcquire())
	{
		/* Initialise I2C communication. */
		I2CcommsInit();

		/* The sensor takes around 250us for transition from OFF state to stand
		 * by state.
		 */
		TimeDelayUSec(250);
		I2CWriteRegister(0x00,0x00,0x00); //Chip REset

		/* Release the I2C bus */
		I2CRelease();
	}

	/* Start advertising */
	startAdvertising();

	/* Start the first timer */
	startTimer(TIMER_TIMEOUT1, timerCallback1);
	/* Start the second timer */
	startTimer(TIMER_TIMEOUT2, timerCallback2);
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessSystemEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a system event, such
 *      as a battery low notification, is received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

void AppProcessSystemEvent(sys_event_id id, void *data)
{
	/* empty */
}


/*----------------------------------------------------------------------------*
 *  NAME
 *      AppProcessLmEvent
 *
 *  DESCRIPTION
 *      This user application function is called whenever a LM-specific event is
 *      received by the system.
 *
 *  RETURNS
 *      Nothing.
 *
 *---------------------------------------------------------------------------*/

bool AppProcessLmEvent(lm_event_code event_code, 
		LM_EVENT_T *p_event_data)
{
	return TRUE;
}
