//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: quad_ndi_haller.h
//
// Code generated for Simulink model 'quad_ndi_haller'.
//
// Model version                  : 1.656
// Simulink Coder version         : 8.13 (R2017b) 24-Jul-2017
// C/C++ source code generated on : Tue Apr 10 13:21:33 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_quad_ndi_haller_h_
#define RTW_HEADER_quad_ndi_haller_h_
#include <math.h>
#ifndef quad_ndi_haller_COMMON_INCLUDES_
# define quad_ndi_haller_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // quad_ndi_haller_COMMON_INCLUDES_

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Forward declaration for rtModel
typedef struct tag_RTM RT_MODEL;

// Block signals and states (auto storage) for system '<Root>'
typedef struct {
  real32_T Integrator_DSTATE;          // '<S8>/Integrator'
  real32_T Filter_DSTATE;              // '<S8>/Filter'
  real32_T Integrator_DSTATE_m;        // '<S7>/Integrator'
  real32_T Filter_DSTATE_n;            // '<S7>/Filter'
  real32_T Integrator_DSTATE_d;        // '<S9>/Integrator'
  real32_T Filter_DSTATE_a;            // '<S9>/Filter'
} DW;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T * volatile errorStatus;
};

// Class declaration for model quad_ndi_haller
class quad_ndi_class {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(const real32_T arg_vehicle_attitude[6], const real32_T
            arg_vehicle_attitude_setpoint[4], const real32_T arg_param[18],
            real32_T arg_cmd[4], real32_T arg_debug[4]);

  // Constructor
  quad_ndi_class();

  // Destructor
  ~quad_ndi_class();

  // Real-Time Model get method
  RT_MODEL * getRTM();

  // private data and function members
 private:
  // Block signals and states
  DW rtDW;

  // Real-Time Model
  RT_MODEL rtM;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'quad_ndi_haller'
//  '<S1>'   : 'quad_ndi_haller/manual_mode_blocks'
//  '<S2>'   : 'quad_ndi_haller/manual_mode_mixer'
//  '<S3>'   : 'quad_ndi_haller/mixer'
//  '<S4>'   : 'quad_ndi_haller/rate_controller'
//  '<S5>'   : 'quad_ndi_haller/stab_controller'
//  '<S6>'   : 'quad_ndi_haller/rate_controller/Subsystem'
//  '<S7>'   : 'quad_ndi_haller/rate_controller/Subsystem/PID Controller'
//  '<S8>'   : 'quad_ndi_haller/rate_controller/Subsystem/PID Controller1'
//  '<S9>'   : 'quad_ndi_haller/rate_controller/Subsystem/PID Controller2'
//  '<S10>'  : 'quad_ndi_haller/stab_controller/PID Controller1'
//  '<S11>'  : 'quad_ndi_haller/stab_controller/PID Controller2'

#endif                                 // RTW_HEADER_quad_ndi_haller_h_

//
// File trailer for generated code.
//
// [EOF]
//
