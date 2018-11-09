/*******************************************************************************
 *  Copyright Cambridge Silicon Radio Limited 2014-2015
 *  Part of CSR uEnergy SDK 2.4.5
 *  Application version 2.4.5.0
 *
 * FILE
 *    i2c_comms.h
 *
 *  DESCRIPTION
 *    Header file for the I2C procedures
 *
 ******************************************************************************/
#ifndef _I2C_COMMS_H
#define _I2C_COMMS_H

/*=============================================================================
 *  SDK Header Files
 *============================================================================*/
#include <pio.h>
#include <types.h>
#include <i2c.h>

/*=============================================================================
 *  Local Header Files
 *============================================================================*/

/* I2C Communication lines */
#define I2C_SCL_PIO                               (0xFF)
#define I2C_SDA_PIO                               (0xFF)

/*=============================================================================
 *  Public function prototypes
 *============================================================================*/
/* This function acquires the I2C bus. */
extern bool I2CAcquire(void);

/* This function releases the I2C bus. */
extern void I2CRelease(void);

/* Initialise the I2C */
extern void I2CcommsInit(void);

/* Read the specified register (1-byte) from the specified device.
 * The baseAddress parameter here is the WRITE address for the device.
 */
extern bool I2CReadRegister(uint8 base_address, uint8 reg, 
                             uint8 *p_register_value);

/* Read a contiguous sequence of registers from the specified device. */
extern bool I2CReadRegisters(uint8 base_address, uint8 start_reg, 
                              uint8 num_bytes, uint8 *buffer);

/* Write one byte of data to the specified register on the specified device. */
extern bool I2CWriteRegister(uint8 base_address, uint8 reg, 
                              uint8 register_value);

extern bool I2CWriteRegisters(uint8 base_address, uint8 length,
                              uint8* register_value_array);
#endif /* _I2C_COMMS_H */
