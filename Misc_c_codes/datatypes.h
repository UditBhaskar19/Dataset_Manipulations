/*****************************************************************************
 *
 * File name: datatypes.h
 *            Includes the custom data type definitions
 *
 * Module:    DataTypes
 * 
 * Author:    Udit Bhaskar Talukdar
 *
 * Description  : 
 * 
 * History      :
 *
 * Version      [BY]					[DESCRIPTION]							[DATE]
 *
 *************************************************************************************/
 
 #ifndef _DATA_TYPES_H_
 #define _DATA_TYPES_H_
 /* ------------------------------
  *        HEADER FILES
  * ----------------------------- */
#include<stdio.h>
#include<conio.h>
#include<stdlib.h>
/* --------------------------------
 *        TYPE DEFINITIONS
 * ------------------------------- */
 typedef unsigned char      uint8_bit;       //unsigned char takes 1 byte , range: 0 to 255
 typedef signed char        sint8_bit;       //signed char takes 1 byte , range: -128 to 127
 typedef unsigned short int uint16_bit;      //unsigned short int takes 2 bytes , range: 0 to 65535
 typedef signed short int   sint16_bit;      //signed short int takes 2 bytes , range: -32767 to 32768
 typedef unsigned int       uint32_bit;      //signed int takes 4 bytes , range: 0 to 2^32-1
 typedef signed int         sint32_bit;      //unsigned int takes 4 bytes , range: (-2^31 to 2^31-1)
 typedef float              float32_bit;     //float takes 4 bytes , range: 3.4e^(-38) to 3.4e^(+38)
 typedef double             double64_bit;    //double takes 8 bytes , range: 1.7e^(-308) to 1.7e^(+308)
 typedef long double        ldouble128_bit;  //long doubles takes 64 bytes , range:
 
 
 