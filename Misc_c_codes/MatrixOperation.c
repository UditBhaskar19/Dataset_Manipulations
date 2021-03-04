/*****************************************************************************
 *
 * File name: MatrixOperation.c
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
 #include "MatrixOperation.h"
 
 /* ----------------------------------------------------------------------------------
    Function	: MatrixInverse
            
    Parameters	:         
		Matrix_ptr  - [IN]  input matrix
	    Inverse_ptr - [OUT] inversed output matrix
		MatrixSize  - [IN]  matrix size
        
    Returns		: 
		
    Description : Matrix inverse using Gauss-Jordan method
-----------------------------------------------------------------------------------------*/
void MatrixInverse(float32_bit *Matrix_ptr , float32_bit *Inverse_ptr , uint16_bit MatrixSize)
{
	uint16_bit idx_dim = 0;
	uint16_bit idx_row = 0;
	uint16_bit idx_col = 0;
	
	float32_bit val = 0.0;
	// create the identity matrix
	for(idx_row = 0 ; idx_row < MatrixSize ; idx_row++)
	{
		for(idx_col = 0; idx_col < MatrixSize ; idx_col++)
		{
			if(idx_row == idx_col)
			{
				Inverse_ptr[idx_row * MatrixSize + idx_row] = 1;
			}
			else
			{
				Inverse_ptr[idx_row * MatrixSize + idx_row] = 0;
			}
		}
	}
	
	for(idx_dim = 0; idx_dim < MatrixSize; idx_dim++)
    // extract each diagonal element
	{
		val = Matrix_ptr[idx_dim * MatrixSize + idx_dim];
		for(idx_col = 0; idx_col < MatrixSize; idx_col++)
		// divide the entire row to make the diagonal element 1 
		{
			Matrix_ptr[idx_dim * MatrixSize + idx_col] = Matrix_ptr[idx_dim * MatrixSize + idx_col]/val;
			Inverse_ptr[idx_dim * MatrixSize + idx_col] = Inverse_ptr[idx_dim * MatrixSize + idx_col]/val;
		}
		// row operation to make the the non doagonal elements '0' for 'Matrix'
		for(idx_row = 0; idx_row < idx_dim; idx_row++)
		{
			val = Matrix_ptr[idx_row * MatrixSize + idx_dim];
			for(idx_col = 0; idx_col < MatrixSize; idx_col++)
			{
				Matrix_ptr[idx_row * MatrixSize + idx_col] -= (val*Matrix_ptr[idx_dim * MatrixSize + idx_col]);
				Inverse_ptr[idx_row * MatrixSize + idx_col] -= (val*Inverse_ptr[idx_dim * MatrixSize + idx_col]);
			}
		}
		for(idx_row = idx_dim + 1; idx_row < MatrixSize ; idx_row++)
		{
			val = Matrix_ptr[idx_row * MatrixSize + idx_dim];
			for(idx_col = 0; idx_col < MatrixSize; idx_col++)
			{
				Matrix_ptr[idx_row * MatrixSize + idx_col] -= (val*Matrix_ptr[idx_dim * MatrixSize + idx_col]);
				Inverse_ptr[idx_row * MatrixSize + idx_col] -= (val*Inverse_ptr[idx_dim * MatrixSize + idx_col]);
			}
		}
	}
}
/**********************************************************************************************
    Function	: MatrixMultiply
            
    Parameters	:         
		Matrix_ptr  - [IN]  input matrix
	    Inverse_ptr - [OUT] inversed output matrix
		MatrixSize  - [IN]  matrix size
        
    Returns		: 
		
    Description : 
**********************************************************************************************/
void MatrixMultiply(float32_bit *Matrix_ptr_A, float32_bit *Matrix_ptr_B, float32_bit *Matrix_ptr, uint16_bit MatrixSize)
{
    uint16_bit idx_dim = 0;
	uint16_bit idx_row = 0;
	uint16_bit idx_col = 0;
	//float32_bit val = 0.0;

    for(idx_row = 0; idx_row < MatrixSize; idx_row++)
	{
        for(idx_col = 0; idx_col < MatrixSize; idx_col++)
        {
			Matrix_ptr[idx_row * MatrixSize + idx_col] = 0.0;
            for(idx_dim = 0; idx_dim < MatrixSize; idx_dim++)
            {
                //val = Matrix_ptr_A[idx_row * MatrixSize + idx_dim] * Matrix_ptr_B[idx_dim * MatrixSize + idx_col];				
			    Matrix_ptr[idx_row * MatrixSize + idx_col] += Matrix_ptr_A[idx_row * MatrixSize + idx_dim] * Matrix_ptr_B[idx_dim * MatrixSize + idx_col];
			}
		}
	}
}
/**********************************************************************************************
    Function	: MatrixMultiply3WithTranspose
            
    Parameters	:         
		Matrix_ptr  - [IN]  input matrix
	    Inverse_ptr - [OUT] inversed output matrix
		MatrixSize  - [IN]  matrix size
        
    Returns		: 
		
    Description : 
**********************************************************************************************/
void MatrixMultiply3WithTranspose(float32_bit *Matrix_ptr_A, float32_bit *Matrix_ptr_B, float32_bit *Matrix_ptr, uint16_bit MatrixSize)
{
    uint16_bit idx_dim = 0;
	uint16_bit idx_row = 0;
	uint16_bit idx_col = 0;
	//float32_bit val = 0.0;

    for(idx_row = 0; idx_row < MatrixSize; idx_row++)
	{
        for(idx_col = 0; idx_col < MatrixSize; idx_col++)
        {
			Matrix_ptr[idx_row * MatrixSize + idx_col] = 0.0;
            for(idx_dim = 0; idx_dim < MatrixSize; idx_dim++)
            {
                //val = Matrix_ptr_A[idx_row * MatrixSize + idx_dim] * Matrix_ptr_B[idx_dim * MatrixSize + idx_col];				
			    Matrix_ptr[idx_row * MatrixSize + idx_col] += Matrix_ptr_A[idx_row * MatrixSize + idx_dim] * Matrix_ptr_B[idx_col * MatrixSize + idx_dim];
			}
		}
	}
}
/**********************************************************************************************
    Function	: MatrixAdd
            
    Parameters	:         
		Matrix_ptr  - [IN]  input matrix
	    Inverse_ptr - [OUT] inversed output matrix
		MatrixSize  - [IN]  matrix size
        
    Returns		: 
		
    Description : 
**********************************************************************************************/			
void MatrixAdd(float32_bit *Matrix_ptr_A , float32_bit *Matrix_ptr_B , uint16_bit MatrixSize)
{
    uint16_bit idx_row = 0;
	uint16_bit idx_col = 0;

    for(idx_row = 0; idx_row < MatrixSize; idx_row++)
	{
        for(idx_col = 0; idx_col < MatrixSize; idx_col++)
        {		
			Matrix_ptr_A[idx_row * MatrixSize + idx_col] += Matrix_ptr_B[idx_row * MatrixSize + idx_col];
		}
	}
}
/**********************************************************************************************
    Function	: EnforcePositiveDefinitedness
            
    Parameters	:         
		Matrix_ptr  - [IN]  input matrix
	    Inverse_ptr - [OUT] inversed output matrix
		MatrixSize  - [IN]  matrix size
        
    Returns		: 
		
    Description : 
**********************************************************************************************/					
void EnforcePositiveDefinitedness(float32_bit *Matrix_ptr , uint16_bit MatrixSize)
{
	uint16_bit idx_row = 0;
	uint16_bit idx_col = 0;
	
	for(idx_row = 0; idx_row < MatrixSize; idx_row++)
	{
		for(idx_col = 0; idx_col < MatrixSize; idx_col++)
		{
			Matrix_ptr[idx_row * MatrixSize + idx_col] = 0.5*(Matrix_ptr[idx_row * MatrixSize + idx_col])*(Matrix_ptr[idx_col * MatrixSize + idx_row]);
		}
	}
}
			
	
			