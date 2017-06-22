//
// File: quad_ndi.h
//
// Code generated for Simulink model 'quad_ndi'.
//
// Model version                  : 1.577
// Simulink Coder version         : 8.10 (R2016a) 10-Feb-2016
// C/C++ source code generated on : Thu Jun 22 10:09:51 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#ifndef RTW_HEADER_quad_ndi_h_
#define RTW_HEADER_quad_ndi_h_
#ifndef quad_ndi_COMMON_INCLUDES_
# define quad_ndi_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // quad_ndi_COMMON_INCLUDES_

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
  real32_T FilterState_DSTATE;         // '<S15>/Filter State'
  real32_T Integrator_DSTATE;          // '<S15>/Integrator'
  real32_T FilterState_DSTATE_e;       // '<S16>/Filter State'
  real32_T Integrator_DSTATE_f;        // '<S16>/Integrator'
  real32_T FilterState_DSTATE_f;       // '<S17>/Filter State'
  real32_T Integrator_DSTATE_fp;       // '<S17>/Integrator'
} DW;

// Constant parameters (auto storage)
typedef struct {
  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<Root>/Gain1'

  real32_T Gain1_Gain[16];
} ConstP;

// Real-time Model Data Structure
struct tag_RTM {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Constant parameters (auto storage)
extern const ConstP rtConstP;

// Class declaration for model quad_ndi
class quad_ndi_class {
  // public data and function members
 public:
  // model initialize function
  void initialize();

  // model step function
  void step(const real32_T arg_vehicle_attitude[6], const real32_T
            arg_vehicle_attitude_setpoint[4], const real32_T arg_param[18],
            real32_T arg_cmd[4]);

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
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<Root>/Display1' : Unused code path elimination
//  Block '<Root>/Display2' : Unused code path elimination
//  Block '<Root>/Scope1' : Unused code path elimination
//  Block '<S13>/POut' : Unused code path elimination
//  Block '<S7>/Sum1' : Unused code path elimination


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
//  '<Root>' : 'quad_ndi'
//  '<S1>'   : 'quad_ndi/MATLAB Function'
//  '<S2>'   : 'quad_ndi/MATLAB Function1'
//  '<S3>'   : 'quad_ndi/NDI LAW'
//  '<S4>'   : 'quad_ndi/Subsystem'
//  '<S5>'   : 'quad_ndi/Subsystem1'
//  '<S6>'   : 'quad_ndi/angle_control'
//  '<S7>'   : 'quad_ndi/control_attitude'
//  '<S8>'   : 'quad_ndi/control_attitude_rates'
//  '<S9>'   : 'quad_ndi/mixer'
//  '<S10>'  : 'quad_ndi/rate_control'
//  '<S11>'  : 'quad_ndi/control_attitude/PID Controller1'
//  '<S12>'  : 'quad_ndi/control_attitude/PID Controller2'
//  '<S13>'  : 'quad_ndi/control_attitude/PID Controller3'
//  '<S14>'  : 'quad_ndi/control_attitude/euler_rates_2_body_rates'
//  '<S15>'  : 'quad_ndi/control_attitude_rates/Discrete_PID_measurement_D'
//  '<S16>'  : 'quad_ndi/control_attitude_rates/Discrete_PID_measurement_D1'
//  '<S17>'  : 'quad_ndi/control_attitude_rates/Discrete_PID_measurement_D2'

#endif                                 // RTW_HEADER_quad_ndi_h_

//
// File trailer for generated code.
//
// [EOF]
//
