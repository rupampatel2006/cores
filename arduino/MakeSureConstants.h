/*
    Copyright (C) 2012 Libelium Comunicaciones Distribuidas S.L.
    http://www.libelium.com
 
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
  
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
    Version:		1.0
    Design:			David Gascón
    Implementation:	David Cuartielles, Alberto Bielsa
*/
 
  


#ifndef __MakeSureCONSTANTS_H__
#define __MakeSureCONSTANTS_H__

/******************************************************************************
 * Includes
 ******************************************************************************/
 
#include	"pins_arduino.h"

/******************************************************************************
 * Definitions & Declarations
 ******************************************************************************/

// internal peripherals flag (IPF) register
// it just re-arranges the PRR0 and PRR1 registers from the MCU:
// MSB	-	7	6	5	4	3	2	1	  0
// ----------------------------------------------------------------------------- 
// PRR0	-	PRTWI	PRTIM2 	PRTIM0	– 	PRTIM1	PRSPI	PRUSART0  PRADC
// PRR1	-	– 	– 	– 	– 	PRTIM3 	– 	–  	  PRUSART1

// IPRA register
// has the three MSB positions empty
#define IPADC 1
#define IPTWI 2
#define IPSPI 4
#define IPUSART0 8
#define IPUSART1 16

// IPRB register
// has the four MSB positions empty
#define IPTIM0 1
#define IPTIM1 2
#define IPTIM2 4
#define IPTIM3 8

// external peripherals flag (EPF) register and constants
// '1' marks should be present, '0' marks not installed
// the EPF should be hardcoded in EEPROM on manufacturing
// a function will then check through the flags if the peripheral is
// installed on a certain MakeSure device, there is room for others
// that could come on the possible add-on boards

// EPRA register
#define PGPS 1
#define PSD 2
#define PACC 4
#define PXBEE 8
#define PRTC 16
#define PUSB 32
#define PSID 64
#define PSLOW_CLK 128

// EPRB register - for GPRS and add-on boards
#define PGPRS 1


// Analog Pins
#define	ANALOG0	0
#define	ANALOG1	1
#define	ANALOG2	2
#define	ANALOG3	3
#define	ANALOG4	4
#define	ANALOG5	5
#define	ANALOG6	6
#define	ANALOG7	7

// MakeSure interrupt vector
// Definitions for 'intFlag' and 'intConf' bit-map vectors
// these are 32-bit interrupt vectors to contain
// the flags for the different HW/SW interrupts
// 0 will denote not-active, 1 will denote active
// the HW interrupts and default callback functions are
// stored inside WInterrupts.c

#define	HAI_INT		1 		// bit 0
#define	LAI_INT		2		// bit 1
#define ACC_INT   	4		// bit 2
#define BAT_INT   	8		// bit 3
#define RTC_INT   	16		// bit 4
#define WTD_INT   	32		// bit 5
#define TIM0_INT  	64		// bit 6
#define TIM1_INT  	128		// bit 7
#define TIM2_INT  	256		// bit 8
#define PIN_INT   	512		// bit 9
#define UART0_INT 	1024	// bit 10
#define UART1_INT 	2048	// bit 11
#define	SENS_INT	4096	// bit 12
#define	PLV_INT		8192	// bit 13
#define	HIB_INT		16384	// bit 14
#define	RAD_INT		32768	// bit 15
#define XBEE_INT 	65536	// bit 16
#define PIR_3G_INT 	131072	// bit 17

/* 
 * Interrupt Counter Vector: 'intArray' vector indexes
 *  __________________________________________________________________________
 * |        |      |     |     |      |       |       |     |     |     |     |    
 * | PIR_3G | XBEE | RAD | PLV | SENS | UART1 | UART0 | PIN | ... | LAI | HAI |
 * |______ _|______|_____|_____|______|_______|_______|_____|_____|_____|_____|
 *     16      15     14   13     12     11       10     11          1     0
 *  
 */
#define	HAI_POS		0		
#define	LAI_POS		1
#define	ACC_POS		2
#define	BAT_POS		3
#define	RTC_POS		4
#define	WTD_POS		5
#define	TIM0_POS	6
#define	TIM1_POS	7
#define	TIM2_POS	8
#define	PIN_POS		9
#define	UART0_POS	10
#define	UART1_POS	11
#define	SENS_POS	12
#define	PLV_POS		13
#define RAD_POS		14
#define XBEE_POS	15
#define PIR_3G_POS	16


// sensor's interrupt pin
#define ACC_INT_ACT 		2 			// MUX_RX
#define	ACC_INT_PIN_MON		RDY_ACC 	// PE6

#define	HAI_INT_ACT			2 			// MUX_RX
#define	HAI_INT_PIN_MON		I2C_SDA 	// PD1

#define	LAI_INT_ACT			3 			// MUX_TX
#define	LAI_INT_PIN_MON		I2C_SDA 	// PD1

#define	BAT_INT_ACT			3 			// MUX_TX
#define	BAT_INT_PIN_MON		LOW_BAT_MON	// PG0

#define	RTC_INT_ACT			2 			// MUX_RX
#define	RTC_INT_PIN_MON		RST_RTC		// PE7

#define	WTD_INT_ACT			4			// 
#define	WTD_INT_PIN_MON		DIGITAL0	// PE4

#define	UART1_INT_ACT		2			// MUX_RX
#define	UART1_INT_PIN_MON	GPRS_PIN 	// PC2

// Event Sensor Board
#define	SENS_INT_ACT		2			// MUX_RX
#define	SENS_INT_PIN_MON 	DIGITAL2  	// PA2
#define	SENS_INT_CLK_REG	DIGITAL7	// PC4
#define	SENS_INT_DO			DIGITAL1	// PE3
#define	SENS_INT_ENABLE		DIGITAL8	// PC5
#define	SENS_INT_CLK_INH	DIGITAL3	// PA4

// Smart Metering Sensor Board
#define	SENS_INT_SMART_ACT			2			// MUX_RX
#define	SENS_INT_SMART_PIN_MON 		DIGITAL2 	// PA2
#define	SENS_INT_SMART_CLK_REG		19
#define	SENS_INT_SMART_DO			18
#define	SENS_INT_SMART_ENABLE		20
#define	SENS_INT_SMART_CLK_INH		17

// Smart Cities Sensor Board
#define	SENS_INT_CITIES_ACT			2			// MUX_RX
#define	SENS_INT_CITIES_PIN_MON 	DIGITAL2	// PA2
#define	SENS_INT_CITIES_CLK_REG		DIGITAL7	// PC4
#define	SENS_INT_CITIES_DO			DIGITAL3	// PA4
#define	SENS_INT_CITIES_ENABLE		DIGITAL5	// PC7
#define	SENS_INT_CITIES_CLK_INH		DIGITAL8	// PC5

// Meteo Sensor Board
#define	PLV_INT_ACT			3 			// MUX_TX
#define SENS2_INT_PIN_MON	DIGITAL5 	// PC7
#define SENS2_INT_PIN2_MON	DIGITAL3	// PA4

// Radiation Sensor Board
#define RAD_INT_ACT			3			// MUX_TX
#define RAD_INT_PIN_MON    DIGITAL7		// PC4

// XBEE Wake up interrupt
#define XBEE_INT_ACT		2			// MUX_RX
#define XBEE_INT_PIN_MON	XBEE_MON	// PA7

#define MAX_ARGS 20    			// max amount of arguments in MakeSure
#define MAX_ARG_LENGTH 16    	// max length for MakeSure arguments

// 3G Camera Board
#define PIR_3G_INT_ACT		2			// MUX_RX	
#define PIR_3G_PIN_MON    A4			// PF5


/*! \def SOCKET0
    \brief socket where UART0 is used
*/
/*! \def SOCKET1
    \brief socket where UART1 is used   
 */
#define SOCKET0	0  
#define SOCKET1	1


/*! \def XBEE_ON
    \brief XBee Power Mode. OFF in this case    
 */
/*! \def XBEE_HIBERNATE
    \brief XBee Power Mode. HIBERNATE in this case    
 */
/*! \def XBEE_OFF
    \brief XBee Power Mode. OFF in this case    
 */
#define	XBEE_ON			1
#define	XBEE_HIBERNATE	2
#define	XBEE_OFF		3


/*! \def XBEE_RATE
    \brief XBee Baud Rate    
 */
#define XBEE_RATE	9600


/*! \def SENS_ON
    \brief Sensor Board Power Modes. ON in this case    
 */
/*! \def SENS_OFF
    \brief Sensor Board Power Modes. OFF in this case    
 */
#define	SENS_ON		0
#define	SENS_OFF	1


/*! \def SENS_3V3
    \brief Sensor Board Types. 3V3 switch in this case    
 */
/*! \def SENS_5V
    \brief Sensor Board Types. 5V switch in this case    
 */
#define	SENS_3V3	0
#define	SENS_5V		1


/*! \def DELAY_TIME
    \brief dealy time used in sensor boards    
 */
#define DELAY_TIME	500

/*! \def boolean
    \brief boolean is defined as a uint8_t  
 */
/*! \def byte
    \brief byte is defined as a uint8_t
 */
#define boolean	uint8_t
#define byte	uint8_t


/*! \def ENABLED
    \brief Constant to define a enabled state  
 */
/*! \def DISABLED
    \brief Constant to define a disabled state    
 */
#define	DISABLED	0
#define	ENABLED		1


// Used EEPROM addresses 
/*! \def AUTHKEY_ADDR
    \brief Authentication key EEPROM address
 */
/*! \def MOTEID_ADDR
    \brief MOTEID EEPROM address
 */
/*! \def SEQUENCE_ADDR
    \brief Sequence EEPROM address
 */
#define MOTEID_ADDR		147
#define SEQUENCE_ADDR	163
#define AUTHKEY_ADDR	107

#endif
