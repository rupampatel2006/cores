/*! \file MakeSureVariables.h
    \brief General variables used through the libraries
    
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
 
/******************************************************************************
 * Includes
 ******************************************************************************/
 
#include <inttypes.h>

/******************************************************************************
 * Definitions & Declarations
 ******************************************************************************/

/*! \def intFlag
    \brief Flag used for storing the modules that have generated an interruption
 */
/*! \def intConf
    \brief Flag used for storing the modules that are enabled to generate an interruption
 */
/*! \def intCounter
    \brief Flag that stores the number of interruptions have been generated
 */
/*! \def intArray
    \brief Flag used for storing the number of times each different kind of interruption has been generated
 */

volatile	uint32_t 	intFlag;
volatile	uint32_t	intConf;

volatile 	uint8_t		intCounter;
volatile 	uint8_t		intArray[17];

