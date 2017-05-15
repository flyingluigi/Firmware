//
// File: main.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "main.h"
#include "ndi_control_terminate.h"
#include "ndi_control_initialize.h"

// Function Declarations
static void argInit_12x1_real32_T(float result[12]);
static void argInit_3x1_real32_T(float result[3]);
static float argInit_real32_T();
static void main_ndi_control();

// Function Definitions

//
// Arguments    : float result[12]
// Return Type  : void
//
static void argInit_12x1_real32_T(float result[12])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 12; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real32_T();
  }
}

//
// Arguments    : float result[3]
// Return Type  : void
//
static void argInit_3x1_real32_T(float result[3])
{
  int idx0;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real32_T();
  }
}

//
// Arguments    : void
// Return Type  : float
//
static float argInit_real32_T()
{
  return 0.0F;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_ndi_control()
{
  float fv2[12];
  float fv3[3];
  float fv4[3];
  float acc_com[3];

  // Initialize function 'ndi_control' input arguments.
  // Initialize function input argument 'x'.
  // Initialize function input argument 'vel_dot_ref'.
  // Initialize function input argument 'vel_ff'.
  // Call the entry-point 'ndi_control'.
  argInit_12x1_real32_T(fv2);
  argInit_3x1_real32_T(fv3);
  argInit_3x1_real32_T(fv4);
  ndi_control(fv2, fv3, fv4, argInit_real32_T(), argInit_real32_T(),
              argInit_real32_T(), argInit_real32_T(), argInit_real32_T(),
              argInit_real32_T(), argInit_real32_T(), acc_com);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  ndi_control_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_ndi_control();

  // Terminate the application.
  // You do not need to do this more than one time.
  ndi_control_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
