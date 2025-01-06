/*
 * File: main.h
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 13-Jun-2020 11:18:06
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
#ifndef MAIN_SP_H
#define MAIN_SP_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>


/* Function Declarations */
extern float vcSportMotionCalculate(float x, float y, float z);
void vcSportMotionAlgoInit(void);
void vcSportMotionAlgoTerminate(void);

#endif

/*
 * File trailer for main.h
 *
 * [EOF]
 */
