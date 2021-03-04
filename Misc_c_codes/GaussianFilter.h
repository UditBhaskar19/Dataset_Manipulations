/**********************************************************************************************
 *  
 * File name	: GaussianFilter.h
 *                Includes the helper functions for the following class of assumed density filtering:
 *                1. Linear Kalman Filter (LKF)
 *                2. Extended Kalman Filter (EKF)
 *                3. Unscented Kalman Filter (UKF)
 *                4. Particle Filter (PF)
 *                5. NN Filter
 *                6. PDAF
 *                7. Gaussian Sum Filter
 *                8. Global NN Filter
 *                9. JPDAF
 *                10.TO-MHT
 *                11.PHD Filtering
 *                12.PMBM Filter
 *
 * Module       : FilteringAndDataAssociation
 *
 * Author       : Udit Bhaskar Talukdar
 *
 * Description  : 
 * 
 * History      :
 *
 * Version      [BY]					[DESCRIPTION]							[DATE]
 *
 **********************************************************************************************/ 
#ifndef _GAUSSIAN_FILTER_H_
#define _GAUSSIAN_FILTER_H_ 

/*--------------------------------------------------------------------------
 *			INCLUDE FILES
 * ------------------------------------------------------------------------- */
#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <math.h>
#include "datatypes.h"
/*--------------------------------------------------------------------------
 *			MACRO DEFINITION
 * ------------------------------------------------------------------------- */
 #define STATE_DIM_CONST_VELOCITY      (4)
 #define STATE_DIM_CONST_ACCELERATION  (6)
 #define MEAS_DIM_PX_PY_VX_VY          (4)
/*-------------------------------------------------------------------------
*			STRUCTURE DECLARATION
* ------------------------------------------------------------------------- */
typedef struct StateParameters2D_CV
{
    float32_bit f32_ErrorCov2D[STATE_DIM_CONST_VELOCITY][STATE_DIM_CONST_VELOCITY];		// Error Covariance matrix
	float32_bit f32_X;			// Longitudinal distance
    float32_bit f32_Y;		    // Lateral distance
	float32_bit f32_Vx;			// Longitudinal velocity
    float32_bit f32_Vy;			// Lateral velocity
	
} STATE_PARAM_STRUCT_2D_CV;

typedef struct StateParameters2D_CA
{
    float32_bit f32_ErrorCov2D[STATE_DIM_CONST_ACCELERATION][STATE_DIM_CONST_ACCELERATION];		// Error Covariance matrix
	float32_bit f32_X;			// Longitudinal distance
    float32_bit f32_Y;		    // Lateral distance
	float32_bit f32_Vx;			// Longitudinal velocity
    float32_bit f32_Vy;			// Lateral velocity
	float32_bit f32_Ax;			// Longitudinal velocity
    float32_bit f32_Ay;			// Lateral velocity
	
} STATE_PARAM_STRUCT_2D_CA;
/*--------------------------------------------------------------------------
 *			FUNCTION PROTOTYPES
 * ------------------------------------------------------------------------- */
/* COORDINATE CONVERTION */
void CartesianToPolarCoordinate(float32_bit *cart, float32_bit *polar);
void PolarToCartesianCoordinate(float32_bit *polar, float32_bit *cart);
//void CartesianToCylindricalCoordinate(float32_bit *cart, float32_bit *cyl);
//void CylindricalToCartesianCoordinate(float32_bit *cyl, float32_bit *cart);

/* KF COMMON OPERATIONS */
void KalmanStateUpdate()
void KalmanStatePrediction()
//void ComputeJacobian()
//void ComputeLogLikelihood()
//void EllipsoidalGate()
//void HypothesisPrune()
//void HypothesisMerge()
//void HypothesisCap()

/* KF MATRIX INITIALIZATION */
void InitFilterParam_CV_PXVXPYVY(STATE_PARAM_STRUCT_2D_CV *obj , const int snsr_obj);
void InitFilterParam_CA_PXVXPYVY(STATE_PARAM_STRUCT_2D_CA *obj , const int snsr_obj);
void InitFilterParam_CV_PXPY(STATE_PARAM_STRUCT_2D_CV *obj , const int snsr_obj);
void InitFilterParam_CA_PXPY(STATE_PARAM_STRUCT_2D_CA *obj , const int snsr_obj);
//void SetProcessNoiseCovarience()
//void SetMeasurementNoiseCovarience()
//void SetInitPosteriorErrorCovarience()
//void SetProcessModel()
//void SetMeasurementModel()

#endif /* _GAUSSIAN_FILTER_H_ */






















