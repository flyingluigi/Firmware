/*
 * File: _coder_ndi_control_api.c
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 11-May-2017 13:59:48
 */

/* Include Files */
#include "tmwtypes.h"
#include "_coder_ndi_control_api.h"
#include "_coder_ndi_control_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true, false, 131434U, NULL, "ndi_control",
  NULL, false, { 2045744189U, 2170104910U, 2743257031U, 4284093946U }, NULL };

/* Function Declarations */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[12]);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *vel_dot_ref,
  const char_T *identifier, real32_T y[3]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[3]);
static real32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *max_bank,
  const char_T *identifier);
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *x, const
  char_T *identifier, real32_T y[12]);
static const mxArray *emlrt_marshallOut(const real32_T u[3]);
static real32_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[12]);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[3]);
static real32_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId);

/* Function Definitions */

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[12]
 * Return Type  : void
 */
static void b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[12])
{
  g_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *vel_dot_ref
 *                const char_T *identifier
 *                real32_T y[3]
 * Return Type  : void
 */
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *vel_dot_ref,
  const char_T *identifier, real32_T y[3])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(vel_dot_ref), &thisId, y);
  emlrtDestroyArray(&vel_dot_ref);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 *                real32_T y[3]
 * Return Type  : void
 */
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real32_T y[3])
{
  h_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *max_bank
 *                const char_T *identifier
 * Return Type  : real32_T
 */
static real32_T e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *max_bank,
  const char_T *identifier)
{
  real32_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = f_emlrt_marshallIn(sp, emlrtAlias(max_bank), &thisId);
  emlrtDestroyArray(&max_bank);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *x
 *                const char_T *identifier
 *                real32_T y[12]
 * Return Type  : void
 */
static void emlrt_marshallIn(const emlrtStack *sp, const mxArray *x, const
  char_T *identifier, real32_T y[12])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  b_emlrt_marshallIn(sp, emlrtAlias(x), &thisId, y);
  emlrtDestroyArray(&x);
}

/*
 * Arguments    : const real32_T u[3]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real32_T u[3])
{
  const mxArray *y;
  const mxArray *m0;
  static const int32_T iv0[1] = { 3 };

  real32_T *pData;
  int32_T i;
  y = NULL;
  m0 = emlrtCreateNumericArray(1, iv0, mxSINGLE_CLASS, mxREAL);
  pData = (real32_T *)mxGetData(m0);
  for (i = 0; i < 3; i++) {
    pData[i] = u[i];
  }

  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real32_T
 */
static real32_T f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real32_T y;
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[12]
 * Return Type  : void
 */
static void g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[12])
{
  static const int32_T dims[1] = { 12 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single|double", false, 1U, dims);
  emlrtImportArrayR2015b(sp, src, ret, 4, false);
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 *                real32_T ret[3]
 * Return Type  : void
 */
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real32_T ret[3])
{
  static const int32_T dims[1] = { 3 };

  emlrtCheckBuiltInR2012b(sp, msgId, src, "single|double", false, 1U, dims);
  emlrtImportArrayR2015b(sp, src, ret, 4, false);
  emlrtDestroyArray(&src);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real32_T
 */
static real32_T i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId)
{
  real32_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "single|double", false, 0U, &dims);
  emlrtImportArrayR2015b(sp, src, &ret, 4, false);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const mxArray * const prhs[10]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void ndi_control_api(const mxArray * const prhs[10], const mxArray *plhs[1])
{
  real32_T x[12];
  real32_T vel_dot_ref[3];
  real32_T vel_ff[3];
  real32_T max_bank;
  real32_T mass;
  real32_T L;
  real32_T delta;
  real32_T kT_max;
  real32_T kMT;
  real32_T gf;
  real32_T acc_com[3];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;

  /* Marshall function inputs */
  emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[0]), "x", x);
  c_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[1]), "vel_dot_ref",
                     vel_dot_ref);
  c_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[2]), "vel_ff",
                     vel_ff);
  max_bank = e_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[3]),
    "max_bank");
  mass = e_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[4]), "mass");
  L = e_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[5]), "L");
  delta = e_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[6]), "delta");
  kT_max = e_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[7]),
    "kT_max");
  kMT = e_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[8]), "kMT");
  gf = e_emlrt_marshallIn(&st, emlrtAliasP((const mxArray *)prhs[9]), "gf");

  /* Invoke the target function */
  ndi_control(x, vel_dot_ref, vel_ff, max_bank, mass, L, delta, kT_max, kMT, gf,
              acc_com);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(acc_com);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ndi_control_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  ndi_control_xil_terminate();
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ndi_control_initialize(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void ndi_control_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_ndi_control_api.c
 *
 * [EOF]
 */
