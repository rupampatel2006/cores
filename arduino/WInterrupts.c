/* -*- mode: jde; c-basic-offset: 2; indent-tabs-mode: nil -*- */

/*
  Part of the Wiring project - http://wiring.uniandes.edu.co

  Copyright (c) 2004-05 Hernando Barragan

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
  
  Modified 24 November 2006 by David A. Mellis
  Modified 1 August 2010 by Mark Sproul
*/

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>

#include "WConstants.h"
#include "MakeSureVariables.h"
#ifndef __MakeSureCONSTANTS_H__
  #include "MakeSureConstants.h"
#endif
#include "wiring_private.h"

static volatile voidFuncPtr intFunc[EXTERNAL_NUM_INTERRUPTS];
// volatile static voidFuncPtr twiIntFunc;

void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode) {
  if(interruptNum < EXTERNAL_NUM_INTERRUPTS) {
    intFunc[interruptNum] = userFunc;
    
    // Configure the interrupt mode (trigger on low input, any change, rising
    // edge, or falling edge).  The mode constants were chosen to correspond
    // to the configuration bits in the hardware register, so we simply shift
    // the mode into place.
      
    // Enable the interrupt.
      
    switch (interruptNum) {
#if defined(__AVR_ATmega32U4__)
	// I hate doing this, but the register assignment differs between the 1280/2560
	// and the 32U4.  Since avrlib defines registers PCMSK1 and PCMSK2 that aren't 
	// even present on the 32U4 this is the only way to distinguish between them.
	case 0:
		EICRA = (EICRA & ~((1<<ISC00) | (1<<ISC01))) | (mode << ISC00);
		EIMSK |= (1<<INT0);
		break;
	case 1:
		EICRA = (EICRA & ~((1<<ISC10) | (1<<ISC11))) | (mode << ISC10);
		EIMSK |= (1<<INT1);
		break;	
    case 2:
        EICRA = (EICRA & ~((1<<ISC20) | (1<<ISC21))) | (mode << ISC20);
        EIMSK |= (1<<INT2);
        break;
    case 3:
        EICRA = (EICRA & ~((1<<ISC30) | (1<<ISC31))) | (mode << ISC30);
        EIMSK |= (1<<INT3);
        break;
#elif defined(EICRA) && defined(EICRB) && defined(EIMSK)
    case 2:
      EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      EIMSK |= (1 << INT0);
      break;
    case 3:
      EICRA = (EICRA & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      EIMSK |= (1 << INT1);
      break;
    case 4:
      EICRA = (EICRA & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      EIMSK |= (1 << INT2);
      break;
    case 5:
      EICRA = (EICRA & ~((1 << ISC30) | (1 << ISC31))) | (mode << ISC30);
      EIMSK |= (1 << INT3);
      break;
    case 0:
      EICRB = (EICRB & ~((1 << ISC40) | (1 << ISC41))) | (mode << ISC40);
      EIMSK |= (1 << INT4);
      break;
    case 1:
      EICRB = (EICRB & ~((1 << ISC50) | (1 << ISC51))) | (mode << ISC50);
      EIMSK |= (1 << INT5);
      break;
    case 6:
      EICRB = (EICRB & ~((1 << ISC60) | (1 << ISC61))) | (mode << ISC60);
      EIMSK |= (1 << INT6);
      break;
    case 7:
      EICRB = (EICRB & ~((1 << ISC70) | (1 << ISC71))) | (mode << ISC70);
      EIMSK |= (1 << INT7);
      break;
#else		
    case 0:
    #if defined(EICRA) && defined(ISC00) && defined(EIMSK)
      EICRA = (EICRA & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      EIMSK |= (1 << INT0);
    #elif defined(MCUCR) && defined(ISC00) && defined(GICR)
      MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      GICR |= (1 << INT0);
    #elif defined(MCUCR) && defined(ISC00) && defined(GIMSK)
      MCUCR = (MCUCR & ~((1 << ISC00) | (1 << ISC01))) | (mode << ISC00);
      GIMSK |= (1 << INT0);
    #else
      #error attachInterrupt not finished for this CPU (case 0)
    #endif
      break;

    case 1:
    #if defined(EICRA) && defined(ISC10) && defined(ISC11) && defined(EIMSK)
      EICRA = (EICRA & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      EIMSK |= (1 << INT1);
    #elif defined(MCUCR) && defined(ISC10) && defined(ISC11) && defined(GICR)
      MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      GICR |= (1 << INT1);
    #elif defined(MCUCR) && defined(ISC10) && defined(GIMSK) && defined(GIMSK)
      MCUCR = (MCUCR & ~((1 << ISC10) | (1 << ISC11))) | (mode << ISC10);
      GIMSK |= (1 << INT1);
    #else
      #warning attachInterrupt may need some more work for this cpu (case 1)
    #endif
      break;
    
    case 2:
    #if defined(EICRA) && defined(ISC20) && defined(ISC21) && defined(EIMSK)
      EICRA = (EICRA & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      EIMSK |= (1 << INT2);
    #elif defined(MCUCR) && defined(ISC20) && defined(ISC21) && defined(GICR)
      MCUCR = (MCUCR & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      GICR |= (1 << INT2);
    #elif defined(MCUCR) && defined(ISC20) && defined(GIMSK) && defined(GIMSK)
      MCUCR = (MCUCR & ~((1 << ISC20) | (1 << ISC21))) | (mode << ISC20);
      GIMSK |= (1 << INT2);
    #endif
      break;
#endif
    }
  }
}

void detachInterrupt(uint8_t interruptNum) {
  if(interruptNum < EXTERNAL_NUM_INTERRUPTS) {
    // Disable the interrupt.  (We can't assume that interruptNum is equal
    // to the number of the EIMSK bit to clear, as this isn't true on the 
    // ATmega8.  There, INT0 is 6 and INT1 is 7.)
    switch (interruptNum) {
#if defined(__AVR_ATmega32U4__)
    case 0:
        EIMSK &= ~(1<<INT0);
        break;
    case 1:
        EIMSK &= ~(1<<INT1);
        break;
    case 2:
        EIMSK &= ~(1<<INT2);
        break;
    case 3:
        EIMSK &= ~(1<<INT3);
        break;		
#elif defined(EICRA) && defined(EICRB) && defined(EIMSK)
    case 2:
      EIMSK &= ~(1 << INT0);
      break;
    case 3:
      EIMSK &= ~(1 << INT1);
      break;
    case 4:
      EIMSK &= ~(1 << INT2);
      break;
    case 5:
      EIMSK &= ~(1 << INT3);
      break;
    case 0:
      EIMSK &= ~(1 << INT4);
      break;
    case 1:
      EIMSK &= ~(1 << INT5);
      break;
    case 6:
      EIMSK &= ~(1 << INT6);
      break;
    case 7:
      EIMSK &= ~(1 << INT7);
      break;
#else
    case 0:
    #if defined(EIMSK) && defined(INT0)
      EIMSK &= ~(1 << INT0);
    #elif defined(GICR) && defined(ISC00)
      GICR &= ~(1 << INT0); // atmega32
    #elif defined(GIMSK) && defined(INT0)
      GIMSK &= ~(1 << INT0);
    #else
      #error detachInterrupt not finished for this cpu
    #endif
      break;

    case 1:
    #if defined(EIMSK) && defined(INT1)
      EIMSK &= ~(1 << INT1);
    #elif defined(GICR) && defined(INT1)
      GICR &= ~(1 << INT1); // atmega32
    #elif defined(GIMSK) && defined(INT1)
      GIMSK &= ~(1 << INT1);
    #else
      #warning detachInterrupt may need some more work for this cpu (case 1)
    #endif
      break;
#endif
    }
      
    intFunc[interruptNum] = 0;
  }
}

/*
void attachInterruptTwi(void (*userFunc)(void) ) {
  twiIntFunc = userFunc;
}
*/

#if defined(__AVR_ATmega32U4__)
SIGNAL(INT0_vect) {
	if(intFunc[EXTERNAL_INT_0])
		intFunc[EXTERNAL_INT_0]();
}

SIGNAL(INT1_vect) {
	if(intFunc[EXTERNAL_INT_1])
		intFunc[EXTERNAL_INT_1]();
}

SIGNAL(INT2_vect) {
    if(intFunc[EXTERNAL_INT_2])
		intFunc[EXTERNAL_INT_2]();
}

SIGNAL(INT3_vect) {
    if(intFunc[EXTERNAL_INT_3])
		intFunc[EXTERNAL_INT_3]();
}

#elif defined(EICRA) && defined(EICRB)

SIGNAL(INT0_vect) {
  if(intFunc[EXTERNAL_INT_2])
    intFunc[EXTERNAL_INT_2]();
}

SIGNAL(INT1_vect) {
  if(intFunc[EXTERNAL_INT_3])
    intFunc[EXTERNAL_INT_3]();
}

SIGNAL(INT2_vect) {
  if(intFunc[EXTERNAL_INT_4])
    intFunc[EXTERNAL_INT_4]();
}

SIGNAL(INT3_vect) {
  if(intFunc[EXTERNAL_INT_5])
    intFunc[EXTERNAL_INT_5]();
}

SIGNAL(INT4_vect) {
  if(intFunc[EXTERNAL_INT_0])
    intFunc[EXTERNAL_INT_0]();
}

SIGNAL(INT5_vect) {
  if(intFunc[EXTERNAL_INT_1])
    intFunc[EXTERNAL_INT_1]();
}

SIGNAL(INT6_vect) {
  if(intFunc[EXTERNAL_INT_6])
    intFunc[EXTERNAL_INT_6]();
}

SIGNAL(INT7_vect) {
  if(intFunc[EXTERNAL_INT_7])
    intFunc[EXTERNAL_INT_7]();
}

#else

SIGNAL(INT0_vect) {
  if(intFunc[EXTERNAL_INT_0])
    intFunc[EXTERNAL_INT_0]();
}

SIGNAL(INT1_vect) {
  if(intFunc[EXTERNAL_INT_1])
    intFunc[EXTERNAL_INT_1]();
}

#if defined(EICRA) && defined(ISC20)
SIGNAL(INT2_vect) {
  if(intFunc[EXTERNAL_INT_2])
    intFunc[EXTERNAL_INT_2]();
}
#endif

#endif

/*
SIGNAL(SIG_2WIRE_SERIAL) {
  if(twiIntFunc)
    twiIntFunc();
}
*/

/* onHAIwakeUP(void) - setup the default interrupt for the High Activate Interrupts
 * 
 * It setups the default interrupt for the High Activate Interrupts. There are some modules that activates interruptions
 * on HIGH. These modules are Accelerometer, UART1 (GPRS) and other additional modules like sensors.
 *
 * This subrutine is executed when the interruption is detected on the corresponding microcontroller pin.
 *
 * We have to check 'intConf' flag and the monitorization pin, to know what module has activated the interruption, due to
 * more than one module is multiplexed on the same microcontroller pin. When the two conditions match, 'intFlag' is activated
 * on the correct position to show the module that has activated the interruption. 
 * A counter called 'intCounter' is incremented each time an interruption is detected. 
 * A counter array called 'intArray' is incremented in the correct position to know how many times a module has activated
 * the interruption.
 */
void onHAIwakeUP(void) 
{
	// Disable all interrupts by clearing the global interrupt mask. 
	cli();
	
	// used to update the interrupt flag
	if( intConf & ACC_INT )
	{	
		if( digitalRead(ACC_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= ACC_INT;
			intArray[ACC_POS]++;
		}
	}
	
	if( intConf & RTC_INT )
	{
		if( digitalRead(RTC_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= RTC_INT;
			intArray[RTC_POS]++;
		}
	}
	
	if( intConf & HAI_INT )
	{
		if( digitalRead(HAI_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= HAI_INT;
			intArray[HAI_POS]++;
		}
	}
	
	if( intConf & SENS_INT )
	{	
		if( digitalRead(SENS_INT_PIN_MON) )
		{
			intCounter++;
			digitalWrite(SENS_INT_ENABLE,LOW);
			intFlag |= SENS_INT;
			intArray[SENS_POS]++;
		}
	}
	if( intConf & XBEE_INT )
	{
		if( digitalRead(XBEE_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= XBEE_INT;
			intArray[XBEE_POS]++;
		}
	}
	
	if( intConf & PIR_3G_INT )
	{
		if( digitalRead(PIR_3G_PIN_MON) )
		{
			intCounter++;
			intFlag |= PIR_3G_INT;
			intArray[PIR_3G_POS]++;
		}
	}
	
	if( intConf & PLV_INT )
	{
		// If there is not any interruption configured for ACC, RTC and UART1
		// or although they are configured, they are NOT active!!
		if( ( !(intConf & ACC_INT) || !digitalRead(ACC_INT_PIN_MON) ) && 
			( !(intConf & RTC_INT) || !digitalRead(RTC_INT_PIN_MON) ) && 
			( !(intConf & UART1_INT) || !digitalRead(UART1_INT_PIN_MON) )	)
		{
			intCounter++;
			intFlag |= PLV_INT;
			intArray[PLV_POS]++;
						
			// time filter so as to avoid multiple interruptions
			unsigned long int i;
			for(i = 0; i < 1000; i++)
			{
				asm volatile("nop\n\t");				
			}
		}
	}
	
	// Enable interrupts by setting the global interrupt mask
	sei();
}

/* onLAIwakeUP(void) - setup the default interrupt for the Low Activate Interrupts
 * 
 * It setups the default interrupt for the Low Activate Interrupts. There are some modules that activates interruptions
 * on LOW. These modules are RTC, Low Battery, internal Watchdog and other additional modules like sensors.
 *
 * This subrutine is executed when the interruption is detected on the corresponding microcontroller pin.
 *
 * We have to check 'intConf' flag and the monitorization pin, to know what module has activated the interruption, due to
 * more than one module is multiplexed on the same microcontroller pin. When the two conditions match, 'intFlag' is activated
 * on the correct position to show the module that has activated the interruption. 
 * A counter called 'intCounter' is incremented each time an interruption is detected. 
 * A counter array called 'intArray' is incremented in the correct position to know how many times a module has activated
 * the interruption.
 */

void onLAIwakeUP(void) 
{
	// Disable all interrupts by clearing the global interrupt mask. 
	cli();
	
	// used to update the interrupt flag
	
	if( intConf & BAT_INT )
	{
		if( !digitalRead(BAT_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= BAT_INT;
			intArray[BAT_POS]++;
			disableInterrupts(BAT_INT);
		}
	}
	
	if( intConf & RAD_INT )
    {
        if( !digitalRead(RAD_INT_PIN_MON) )
        {
			
            intCounter++;
            intFlag |= RAD_INT;
            intArray[RAD_POS]++;
            disableInterrupts(RAD_INT);
        }
    }

	if( intConf & WTD_INT )
	{
		if( !digitalRead(WTD_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= WTD_INT;
			intArray[WTD_POS]++;
			digitalWrite(WTD_INT_PIN_MON,HIGH);
		}
	}
	
	if( intConf & UART1_INT )
	{
		if( digitalRead(UART1_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= UART1_INT;
			intArray[UART1_POS]++;
		}
	}

	if( intConf & LAI_INT )
	{
		if( !digitalRead(LAI_INT_PIN_MON) )
		{
			intCounter++;
			intFlag |= LAI_INT;
			intArray[LAI_POS]++;
		}
	}


	// Enable interrupts by setting the global interrupt mask
	sei();
}

/* clearIntFlag() - clears 'intFlag' global variable
 *
 */
void	clearIntFlag()
{
	intFlag=0;
}


/* enableInterrupts( conf ) - enables the specified interruption
 *
 * It enables the specified interruption by 'conf' input.
 *
 * When this function is called, 'intConf' flag is updated with the new active 
 * interruption. After that, it is attached to
 * the corresponding microcontroller pin and associated subrutine.
 */
void enableInterrupts(uint32_t conf)
{
	intConf |= conf;
		
	if( conf & HAI_INT )
	{
		pinMode(MUX_RX,INPUT);
		attachInterrupt(HAI_INT_ACT, onHAIwakeUP, HIGH);
	}
	if( conf & LAI_INT )
	{
		pinMode(MUX_TX, INPUT);
		attachInterrupt(LAI_INT_ACT, onLAIwakeUP, LOW);
	}
	if( conf & ACC_INT )
	{
		pinMode(MUX_RX,INPUT);
		pinMode(ACC_INT_PIN_MON, INPUT);
		attachInterrupt(ACC_INT_ACT, onHAIwakeUP, HIGH);
	}
	if( conf & BAT_INT )
	{
		pinMode(MUX_TX, INPUT);
		pinMode(BAT_INT_PIN_MON,INPUT);
		digitalWrite(MUX_TX, HIGH);
		attachInterrupt(BAT_INT_ACT, onLAIwakeUP, LOW);
	}	
	if( conf & RTC_INT )
	{
		pinMode(MUX_RX, INPUT);
		pinMode(RTC_INT_PIN_MON,INPUT);
		attachInterrupt(RTC_INT_ACT, onHAIwakeUP, HIGH);
	}
	if( conf & WTD_INT )
	{
		pinMode(WTD_INT_PIN_MON,OUTPUT);
		digitalWrite(WTD_INT_PIN_MON,HIGH);
		attachInterrupt(WTD_INT_ACT, onLAIwakeUP, LOW);
	}
	if( conf & UART1_INT )
	{
		pinMode(MUX_RX, INPUT);
		pinMode(UART1_INT_PIN_MON,INPUT);
		attachInterrupt(UART1_INT_ACT, onLAIwakeUP, LOW);
	}
	if( conf & SENS_INT )
	{
		pinMode(MUX_RX, INPUT);
		pinMode(SENS_INT_PIN_MON,INPUT);
		attachInterrupt(SENS_INT_ACT, onHAIwakeUP, HIGH);
	}
	if( conf & PLV_INT )
	{
		pinMode(MUX_RX, INPUT);
		pinMode(MUX_TX, INPUT);
		pinMode(SENS2_INT_PIN_MON,INPUT);
		pinMode(SENS2_INT_PIN2_MON,INPUT);
		attachInterrupt(PLV_INT_ACT, onLAIwakeUP, LOW);
	}
	
	if( conf & RAD_INT )
	{
		pinMode(RAD_INT_PIN_MON,INPUT);
        pinMode(MUX_TX, INPUT);
        attachInterrupt(RAD_INT_ACT, onLAIwakeUP, LOW);
	}
	
	if( conf & XBEE_INT )
	{
		pinMode(XBEE_INT_PIN_MON,INPUT);
        pinMode(MUX_RX, INPUT);
        attachInterrupt(XBEE_INT_ACT, onHAIwakeUP, HIGH);
	}
	
	if( conf & PIR_3G_INT )
	{
		pinMode(MUX_RX, INPUT);
		pinMode(PIR_3G_PIN_MON,INPUT);
		attachInterrupt(PIR_3G_INT_ACT, onHAIwakeUP, HIGH);
	}
}


/* disableInterrupts( conf ) - disables the specified interruption
 *
 * It disables the specified interruption by 'conf' input.
 *
 * When this function is called, 'intConf' flag is updated with the interruption that has been detached. After that, it is
 * deatached from the corresponding microcontroller pin and associated subrutine.
 */
void disableInterrupts(uint32_t conf)
{
	// Disable all interrupts by clearing the global interrupt mask. 
	cli();
	
	if( conf & HAI_INT )
	{
		detachInterrupt(HAI_INT_ACT);
	}
	if( conf & LAI_INT )
	{
		detachInterrupt(LAI_INT_ACT);
	}
	if( conf & ACC_INT )
	{
		detachInterrupt(ACC_INT_ACT);
	}
	if( conf & BAT_INT )
	{
		detachInterrupt(BAT_INT_ACT);
	}	
	if( conf & RTC_INT )
	{
		detachInterrupt(RTC_INT_ACT);
	}
	if( conf & UART1_INT )
	{
		detachInterrupt(UART1_INT_ACT);
	}
	if( conf & WTD_INT )
	{
		detachInterrupt(WTD_INT_ACT);
	}
	if( conf & SENS_INT )
	{
		detachInterrupt(SENS_INT_ACT);
	}
	if( conf & PLV_INT )
	{
		detachInterrupt(PLV_INT_ACT);
	}
	if( conf & XBEE_INT )
	{
		detachInterrupt(XBEE_INT_ACT);
	}
	if( conf & PIR_3G_INT )
	{
		detachInterrupt(PIR_3G_INT_ACT);
	}
	if( conf & RAD_INT )
	{
		detachInterrupt(RAD_INT_ACT);
	}
	intConf &= ~(conf);
	
	// Enable interrupts by setting the global interrupt mask
	sei();
}

