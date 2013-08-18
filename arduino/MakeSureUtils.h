/*! \file MakeSureUtils.h
    \brief Library containing useful general functions
    
    Copyright (C) 2012 Libelium Comunicaciones Distribuidas S.L.
    http://www.libelium.com
 
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 2.1 of the License, or
    (at your option) any later version.
   
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
  
    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  
    Version:		1.0
    Design:			David Gascón
    Implementation:	Alberto Bielsa, David Cuartielles

*/
  
  
/*! \def MakeSureutils_h
    \brief The library flag
    
 */
#ifndef MakeSureutils_h
#define MakeSureutils_h

/******************************************************************************
 * Includes
 ******************************************************************************/
 
#include <inttypes.h>
#include <avr/eeprom.h>

/******************************************************************************
 * Definitions & Declarations
 ******************************************************************************/

/*! \def LED_ON
    \brief sets LED ON
 */
#define	LED_ON	1

/*! \def LED_OFF
    \brief sets LED OFF
 */
#define	LED_OFF	0

/*! \def MUX_TO_HIGH
    \brief sets mux high
 */
#define	MUX_TO_HIGH	1

/*! \def MUX_TO_LOW
    \brief sets mux low
 */
#define	MUX_TO_LOW	0

/*! \def EEPROM_START
    \brief First EEPROM's writable address. There is a 1kB reserved area from 
    address 0 to address 1023.
 */
#define EEPROM_START 1024

/*! \def SD_SELECT
    \brief select SD on SPI bus
 */
#define	SD_SELECT	0

/*! \def SOCKET0_SELECT
    \brief select SOCKET 0 on SPI bus
 */
#define	SOCKET0_SELECT	1

/*! \def SOCKET1_SELECT
    \brief select SOCKET 1 on SPI bus
 */
#define	SOCKET1_SELECT	2

/*! \def ALL_DESELECTED
    \brief deselect all devides on SPI bus
 */
#define	ALL_DESELECTED	3


/******************************************************************************
 * Class
 ******************************************************************************/
 
//! MakeSureUtils Class
/*!
	MakeSureUtils Class defines all the variables and functions used to set LEDs, 
	multiplexor and useful general functions
 */
class MakeSureUtils
{
  private:
  
  public:

  //! class constructor
  /*!
  It does nothing
  \param void
  \return void
  */
  MakeSureUtils(void);


  //! It sets the specified LED to the specified state(ON or OFF)
  /*!
  \param uint8_t led : the LED to set ON/OFF
  \param uint8_t state : the state to set the LED
  \return void
  \sa getLED(uint8_t led), blinkLEDs(uint16_t time)
   */
  void	setLED(uint8_t led, uint8_t state);
  
  
   //! It sets external LED to the specified state(ON or OFF)
  /*!
   \param uint8_t state : the state to set the LED
  \return void
  */
  void	setExternalLED(uint8_t state);
  
  
  //! It gets the state of the specified LED
  /*!
  \param uint8_t led : the LED to get the state
  \return the state of the LED
  \sa setLED(uint8_t led, uint8_t state), blinkLEDs(uint16_t time)
   */
  uint8_t getLED(uint8_t led);
  
  //! It gets the state of external LED
  /*!
  
  \return the state of the LED
  \sa setLED(uint8_t led, uint8_t state), blinkLEDs(uint16_t time)
   */
  uint8_t getExternalLED();
  
  
  
  
  //! It blinks LEDs, with the specified time for blinking
  /*!
  \param uint16_t time : time for blinking
  \return void
  \sa setLED(uint8_t led, uint8_t state), getLED(uint8_t led)
   */
  void blinkLEDs(uint16_t time);
  
  
  //! It blinks LED, with the specified time for blinking
  /*!
  \param uint16_t time : time for blinking
  \return void
  \sa setLED(uint8_t led, uint8_t state), getLED(uint8_t led)
   */
  void externalLEDBlink(uint16_t time);
  
  
  //! It maps 'x' from the read range to the specified range
  /*!
  \param long x : value to map
  \param long in_min : minimum input value for 'x'
  \param long in_max : maximum input value for 'x'
  \param long out_min : minimum output value for 'x'
  \param long out_max : maximum output value for 'x'
  \return the value 'x' mapped to the [out_min,out_max] range
   */
  long map(long x, long in_min, long in_max, long out_min, long out_max);

  //! It sets multiplexer on UART_1 to the desired combination
  /*! It sets multiplexer on UART_1 to the desired combination. 
    Possible combinations are:  
  	MUX_LOW = 0 & MUX_HIGH = 1 ---> GPS MODULE
  	MUX_LOW = 1 & MUX_HIGH = 1 ---> SOCKET1
  	MUX_LOW = 1 & MUX_HIGH = 0 ---> AUX1 MODULE
  	MUX_LOW = 0 & MUX_HIGH = 0 ---> AUX2 MODULE
  
  \param uint8_t MUX_LOW : low combination part
  \param uint8_t MUX_HIGH : high combination part
  \return void
  \sa setMuxGPS(), setMuxSocket1(), setMuxAux1(), setMuxAux2()
   */
  void setMux(uint8_t MUX_LOW, uint8_t MUX_HIGH);
  
  //! It sets multiplexer on UART_1 to GPS module
  /*!  
  \return void
   */
  void setMuxGPS();
  
  //! It sets multiplexer on UART_1 to SOCKET1
  /*!  
  \return void
   */
  void setMuxSocket1();
  
  //! It sets multiplexer on UART_1 to enable AUX1 module
  /*!  
  \return void
   */
  void setMuxAux1();
  
  //! It sets multiplexer on UART_1 to enable AUX2 module
  /*!  
  \return void
   */
  void setMuxAux2();  
   
  //! It sets multiplexer on UART_0 to USB
  /*!
  \return void
   */
  void setMuxUSB();  

   
  //! It switches off the multiplexer on UART_0 
  /*!
  \return void
   */
  void muxOFF(); 
  
  //! set multiplexer on UART_0 to SOCKET0
  /*!
  \return void
   */
  void setMuxSocket0();

  //! It reads a value from the specified EEPROM address
  /*!
  \param int address : EEPROM address to read from
  \return the value read from EEPROM address
  \sa writeEEPROM(int address, uint8_t value)
   */
  uint8_t readEEPROM(int address);
  
  //! It writes the specified value to the specified EEPROM address
  /*!
  \param int address : EEPROM address to write to
  \param uint8_t value: value to write to the EEPROM
  \return void
  \sa readEEPROM(int address)
   */
  void writeEEPROM(int address, uint8_t value);
   
  //! It writes the mote identifier to the EEPROM[147-162]
  /*!
  \param char* moteID: identifier to write to the EEPROM
  \return void 
   */
  void setID(char* moteID);
   
  //! It writes the authentication key to the EEPROM[107-114]
  /*!
  \param char* authkey: authentication key to write to the EEPROM
  \return void 
   */
  void setAuthKey(char* authkey);
   
  //! It reads the MakeSuremote unique serial identifier
  /*!
  \return unsigned long: MakeSuremote unique serial identifier
  \sa 
   */
  //unsigned long readSerialID();
  
  //! It reads the DS1820 temperature sensor
  /*!
  \return long: Temperature of DS1820 sensor
  \sa 
   */
  //float readTempDS1820(uint8_t pin);
  
  //! It reads the temperature sensor
  /*!
  \return float: Temperature of the sensor
   */
  float readTemperature();
  
  //! It reads the humidity sensor
  /*!
  \return uint8_t: Value of the humidity sensor
   */
  uint8_t readHumidity();
  
  //! It reads the light sensor
  /*!
  \return uint8_t: Value of the light sensor
   */
  uint8_t readLight();
  
 
    
  //! It converts a decimal number into a string
  /*!
  \param long num : number to convert
  \param char* numb : string where store the converted number
  \return the number of digits of the number
  \sa  str2hex(char* str), str2hex(uint8_t* str)
   */
  uint8_t long2array(long num, char* numb);
  
  //! It converts a number stored in a string into a hexadecimal number
  /*!
  \param char* str : string where the number is stored
  \return the converted number
  \sa long2array(long num, char* numb), str2hex(uint8_t* str)
   */
  uint8_t str2hex(char* str);
  
  //! It converts a number stored in a string into a hexadecimal number
  /*!
  \param char* str : string where thember is stored
  \return the converted number
  \sa long2array(long num, char* numb), str2hex(char* str)
   */
  uint8_t str2hex(uint8_t* str);
  
  //! It converts a hexadecimal number stored in an array to a string (8 Byte numbers)
  /*!
  \param uint8_t* number : hexadecimal array to conver to a string
  \param const char* macDest : char array where the converted number is stored
  \return void
  \sa long2array(long num, char* numb), str2hex(char* str), str2hex(uint8_t* str)
   */
  void hex2str(uint8_t* number, char* macDest);
  
   //! It converts a hexadecimal number stored in an array to a string (8 Byte numbers)
  /*!
  \param uint8_t* number : hexadecimal array to conver to a string
  \param const char* macDest : char array where the converted number is stored
  \param uint8_t length : length to copy
  \return void
  \sa long2array(long num, char* numb), str2hex(char* str), str2hex(uint8_t* str)
   */
  void hex2str(uint8_t* number, char* macDest, uint8_t length);
  
  //! It clears the arguments[][] data matrix
  /*!
  \param void
  \return void
   */
  void clearArguments(void);  

  
  //! It breaks a string into its arguments separated by "separators". The pieces are stored in 'arguments' array
  /*!
  \param const char* str : string to separate
  \param char separator : the separator used to separate the string in pieces
  \return void
  \sa clearArguments(),clearBuffer()
   */
  void strExplode(const char* str, char separator);
  
  //! It generates a decimal number from two ASCII characters which were numbers
  /*!
  \param uint8_t conv1 : the ASCII number first digit to convert
  \param uint8_t conv2 : the ASCII number second digit to convert
  \return the converted number
  */
  uint8_t converter(uint8_t conv1, uint8_t conv2);
  
  //! It converts a float into a string
  /*!
  \param float fl : the float to convert
  \param char str[] : the string where store the float converted
  \param int N : the number of decimals
  \return void
   */
  void float2String(float fl, char str[], int N);
  
  //! It selects the slave on SPI bus to use
  /*!
  	Possibilities:
   	SD_SELECT
  	SRAM_SELECT
  	SOCKET0_SELECT
  	SOCKET1_SELECT
  	ALL_DESELECTED
  \param uint8_t SELECTION : the selection
  \return void
  */
  void setSPISlave(uint8_t SELECTION);
  
  //! It writes into the EEPROM the name of the OTA file
  /*!
  \return void
  */
  void loadOTA(const char* filename, uint8_t version);
  
  //! It reads the EEPROM from position 2 to 34 and shows it by USB
  /*!
  \return void
  */
  void readEEPROM();
  
  //! It checks the new firmware upgrade
  /*!
  \return '0' if reprogramming error, '1' if reprogramming OK and '2' for normal restart
  */
  int8_t checkNewProgram();
  
  //! It reads program ID (PID) from EEPROM
  /*!
  \param char* program_ID : string pointer to store the PID
  \return void
  */
  void getProgramID(char* program_ID);
  
  //! It reads mote ID from EEPROM
  /*!
  \param char* moteID : string pointer to store the mote ID
  \return void
  */
  void getID(char* moteID);
  
   //! It stores the version of the program to EEPROM
  /*!
  \param uint8_t version : version of te program. Values from 0 to 255
  \return void
  */
  void setProgramVersion(uint8_t version);
  
   //! It reads the version of the program from EEPROM
  /*!
  \return the version of the actual program
  */
  uint8_t getProgramVersion();
  
};

extern MakeSureUtils Utils;

#endif
