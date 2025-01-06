/*
 * File: main.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 09-Sep-2020 19:11:59
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/
#ifndef SPO2ALGO_H
#define SPO2ALGO_H

/* Include Files */
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>


/* Function Declarations */
extern float vcSpo2Calculate(float r_ppg, float ir_ppg);
extern void vcSpo2AlgoInit(void);
extern void vcSpo2AlgoTerminate(void);

#endif

/*
 * File trailer for main.h
 *
 * [EOF]
 */
