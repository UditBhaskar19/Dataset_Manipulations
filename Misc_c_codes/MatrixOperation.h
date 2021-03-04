/*****************************************************************************
 *
 * File name: MatrixOperation.h
 *            Includes the helper functions for the following matrix operation
 *            1. Matrix Multiplication
 *            2. Matrix Inverse
 *            3. Vector add
 *            4. Vector subtract
 *
 * Module:    NumericalMethods
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

#ifndef _MATRIX_OPP_H_
#define _MATRIX_OPP_H_
/* -------------------------------------------
 *             INCLUDE FILES
 * ------------------------------------------ */
#include<stdio.h>
#include<conio.h>
#include<stdlib.h>
#include<math.h>
#include "datatypes.h"
/* --------------------------------------------
 *             FUNCTION PROTOTYPES
 * -------------------------------------------- */
void MatrixInverse(float32_bit *Matrix_ptr , float32_bit *Inverse , uint16_bit MatrixSize);
void EnforcePositiveDefinitedness(float32_bit *Matrix_ptr , uint16_bit MatrixSize);
void MatrixMultiply3WithTranspose(float32_bit *Matrix_ptr_A , float32_bit *Matrix_ptr_B , float32_bit *Matrix_ptr , uint16_bit MatrixSize);
void MatrixMultiply(float32_bit *Matrix_ptr_A , float32_bit *Matrix_ptr_B , float32_bit *Matrix , uint16_bit MatrixSize);
void MatrixAdd(float32_bit *Matrix_ptr_A , float32_bit *Matrix_ptr_B , uint16_bit MatrixSize);


 
 
 