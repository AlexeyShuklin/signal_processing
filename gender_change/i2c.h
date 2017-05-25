/*
 *  I2C header file
 *
 */

#ifndef I2C_
#define I2C_

#include "C5515.h"

/* ------------------------------------------------------------------------ *
 *  I2C Module                                                              *
 * ------------------------------------------------------------------------ */

#define I2C_IER    	       *(volatile ioport Uint16*)(0x1A04)
#define I2C_STR    	       *(volatile ioport Uint16*)(0x1A08)
#define I2C_CLKL           *(volatile ioport Uint16*)(0x1A0C)
#define I2C_CLKH           *(volatile ioport Uint16*)(0x1A10)
#define I2C_CNT    		   *(volatile ioport Uint16*)(0x1A14)
#define I2C_DRR    		   *(volatile ioport Uint16*)(0x1A18)
#define I2C_SAR    	       *(volatile ioport Uint16*)(0x1A1C)
#define I2C_DXR    	       *(volatile ioport Uint16*)(0x1A20)
#define I2C_MDR            *(volatile ioport Uint16*)(0x1A24)
#define I2C_EDR    	       *(volatile ioport Uint16*)(0x1A2C)
#define I2C_PSC    	       *(volatile ioport Uint16*)(0x1A30)

#define MDR_STT			0x2000
#define MDR_TRX			0x0200
#define MDR_MST			0x0400
#define MDR_IRS			0x0020
#define MDR_FREE		0x4000
#define STR_XRDY		0x0010
#define STR_RRDY		0x0008
#define MDR_STP 		0x0800
/* ------------------------------------------------------------------------ *
 *  Prototypes                                                              *
 * ------------------------------------------------------------------------ */
Int16 I2C_init ( );
Int16 I2C_close( );
Int16 I2C_read( Uint16 i2c_addr, Uint8* data, Uint16 len );
Int16 I2C_write( Uint16 i2c_addr, Uint8* data, Uint16 len );

#endif
