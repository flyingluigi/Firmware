//
// File: ndi_control.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 11-May-2017 13:59:48
//

// Include Files
#include "rt_nonfinite.h"
#include "ndi_control.h"
#include "svd1.h"

// Function Definitions

//
// Arguments    : const float x[12]
//                const float vel_dot_ref[3]
//                const float vel_ff[3]
//                float max_bank
//                float mass
//                float L
//                float delta
//                float kT_max
//                float kMT
//                float gf
//                float acc_com[3]
// Return Type  : void
//
void ndi_control(const float x[12], const float vel_dot_ref[3], const float
                 vel_ff[3], float max_bank, float mass, float L, float delta,
                 float kT_max, float kMT, float gf, float acc_com[3])
{
  float b_x;
  float h;
  float m;
  float fcnOutput;
  float b_fcnOutput;
  float c_x;
  float d_x;
  float e_x;
  float f_x;
  float cthe;
  float g_x;
  float h_x;
  float i_x;
  float xdot_s[6];
  int k;
  float j_x;
  float k_x;
  float t;
  float b_cthe;
  float tthe;
  float b_t;
  float tol;
  float scale;
  float absxk;
  float X[9];
  float b_vel_ff[3];
  int i0;
  float c_t;
  float R[24];
  float c_cthe[9];
  float l_x[24];
  float fv0[24];
  float m_x[24];
  float b_k[8];
  int vcol;
  float c_k[24];
  float fv1[48];
  float d_k[48];
  float e_k[18];
  float d_cthe[18];
  float U[9];
  float S[9];
  float V[9];
  int r;
  float e_cthe[18];
  int ar;
  int ic;
  int ib;
  int ia;
  b_x = cosf(delta);
  h = b_x / (2.0F * gf);
  m = sinf(delta) * kMT / (10.0F * gf);
  fcnOutput = fminf(fmaxf(x[9], -max_bank), max_bank);
  b_fcnOutput = fminf(fmaxf(x[10], -max_bank), max_bank);

  //  Frequently used functions
  c_x = sinf(x[9]);
  d_x = cosf(x[9]);
  e_x = sinf(x[10]);
  f_x = cosf(x[10]);
  cthe = f_x;
  g_x = sinf(x[11]);
  h_x = cosf(x[11]);
  if (fabsf(f_x) <= 1.0E-5F) {
    if (f_x < 0.0F) {
      i_x = -1.0F;
    } else if (f_x > 0.0F) {
      i_x = 1.0F;
    } else if (f_x == 0.0F) {
      i_x = 0.0F;
    } else {
      i_x = f_x;
    }

    cthe = 1.0E-5F * i_x;
  }

  //  state part of state derivatives xdot = f(x)+g(x)*u
  //  Author: Bernd Messnarz, Last Revision: 2015|03|22
  //  definition of states
  //  x =[u; v; w; x; y; z; p; q; r; phi; theta; psi]
  //  Kinematic velocity in body frame
  //  Angular velocity of the body in body frame
  //  Initialization
  for (k = 0; k < 6; k++) {
    xdot_s[k] = 0.0F;
  }

  //  DCM to transform wind into body axes
  f_x = sinf(x[9]);
  j_x = cosf(x[9]);
  k_x = sinf(x[10]);
  t = cosf(x[10]);
  b_cthe = t;
  if (fabsf(t) <= 1.0E-5F) {
    if (t < 0.0F) {
      b_t = -1.0F;
    } else if (t > 0.0F) {
      b_t = 1.0F;
    } else if (t == 0.0F) {
      b_t = 0.0F;
    } else {
      b_t = t;
    }

    b_cthe = 1.0E-5F * b_t;
  }

  tthe = k_x / b_cthe;

  //  Body drag force and moment (damping)
  //  for drag force
  //  Drag force
  tol = 0.0F;
  scale = 1.17549435E-38F;
  for (k = 0; k < 3; k++) {
    absxk = fabsf(x[k]);
    if (absxk > scale) {
      t = scale / absxk;
      tol = 1.0F + tol * t * t;
      scale = absxk;
    } else {
      t = absxk / scale;
      tol += t * t;
    }
  }

  tol = scale * sqrtf(tol);
  tol *= -0.062F;

  //  cD = 1.24*1*0.1*1/2  (rho*CD*A*0.5) (=0.062)
  //  Gravity in body frame
  //  EOM
  //  H_eulerdot_omega
  //  Force (drag, gravity and inertia)
  xdot_s[0] = (9.80665F * -k_x + tol * x[0] / mass) - (x[7] * x[2] - x[8] * x[1]);
  xdot_s[1] = (9.80665F * (f_x * b_cthe) + tol * x[1] / mass) - (x[8] * x[0] -
    x[6] * x[2]);
  xdot_s[2] = (9.80665F * (j_x * b_cthe) + tol * x[2] / mass) - (x[6] * x[1] -
    x[7] * x[0]);

  //  Moment (drag, gravity and inertia)
  //  Attitude
  X[0] = 1.0F;
  X[3] = f_x * tthe;
  X[6] = j_x * tthe;
  X[1] = 0.0F;
  X[4] = j_x;
  X[7] = -f_x;
  X[2] = 0.0F;
  X[5] = f_x / b_cthe;
  X[8] = j_x / b_cthe;
  for (i0 = 0; i0 < 3; i0++) {
    b_vel_ff[i0] = 0.0F;
    for (k = 0; k < 3; k++) {
      b_vel_ff[i0] += X[i0 + 3 * k] * x[6 + k];
    }

    xdot_s[3 + i0] = b_vel_ff[i0];
  }

  f_x = sinf(fcnOutput);
  j_x = cosf(fcnOutput);
  k_x = sinf(b_fcnOutput);
  t = cosf(b_fcnOutput);
  b_cthe = t;
  tol = sinf(x[11]);
  scale = cosf(x[11]);
  if (fabsf(t) <= 1.0E-5F) {
    if (t < 0.0F) {
      c_t = -1.0F;
    } else if (t > 0.0F) {
      c_t = 1.0F;
    } else if (t == 0.0F) {
      c_t = 0.0F;
    } else {
      c_t = t;
    }

    b_cthe = 1.0E-5F * c_t;
  }

  //  Control part of state derivatives xdot = f(x)+gmatrix(x)*um
  //  L = 1; % length from arm!
  //  delta = 40 * pi/180;% motor tilt angle
  //  kT_max = 11; % Max thrust
  //  mass = 3.5000; % mass copter
  //  Kinematic velocity in body frame
  //  Angular velocity vector in body frame
  //  Velocities of rotors with respect to air
  //  VA = VK - VW
  //  define cross product matrix
  R[0] = L;
  R[1] = 0.0F;
  R[2] = 0.0F;
  R[3] = L / 1.41421354F;
  R[4] = L / 1.41421354F;
  R[5] = 0.0F;
  R[6] = 0.0F;
  R[7] = L;
  R[8] = 0.0F;
  R[9] = -L / 1.41421354F;
  R[10] = L / 1.41421354F;
  R[11] = 0.0F;
  R[12] = -L;
  R[13] = 0.0F;
  R[14] = 0.0F;
  R[15] = -L / 1.41421354F;
  R[16] = -L / 1.41421354F;
  R[17] = 0.0F;
  R[18] = 0.0F;
  R[19] = -L;
  R[20] = 0.0F;
  R[21] = L / 1.41421354F;
  R[22] = -L / 1.41421354F;
  R[23] = 0.0F;
  t = sinf(delta);
  absxk = cosf(delta);

  //  Matrix for forces:  Frotb = Fmat * un;
  //  Thrust reduction factor, s/m;
  c_cthe[0] = 0.0F;
  c_cthe[3] = -x[8];
  c_cthe[6] = x[7];
  c_cthe[1] = x[8];
  c_cthe[4] = 0.0F;
  c_cthe[7] = -x[6];
  c_cthe[2] = -x[7];
  c_cthe[5] = x[6];
  c_cthe[8] = 0.0F;
  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 8; k++) {
      l_x[i0 + 3 * k] = x[i0];
      fv0[i0 + 3 * k] = 0.0F;
      for (vcol = 0; vcol < 3; vcol++) {
        fv0[i0 + 3 * k] += c_cthe[i0 + 3 * vcol] * R[vcol + 3 * k];
      }
    }
  }

  for (i0 = 0; i0 < 8; i0++) {
    for (k = 0; k < 3; k++) {
      m_x[k + 3 * i0] = l_x[k + 3 * i0] + fv0[k + 3 * i0];
    }

    b_k[i0] = kT_max * (1.0F + 0.05F * m_x[2 + 3 * i0]);
  }

  //  Rot  1        2     3        4     5         6     7          8
  // Fxb
  // Fyb
  // Fzb
  //  gmatrix dimension = 6 (outputs) x 8 (inputs um)
  //  [p;q;r] dot
  //  [phi;theta;psi] dot
  //  NDI control law
  for (i0 = 0; i0 < 9; i0++) {
    X[i0] = 0.0F;
  }

  c_k[0] = -b_k[0] * t;
  c_k[3] = 0.0F;
  c_k[6] = 0.0F;
  c_k[9] = 0.0F;
  c_k[12] = b_k[4] * t;
  c_k[15] = 0.0F;
  c_k[18] = 0.0F;
  c_k[21] = 0.0F;
  c_k[1] = 0.0F;
  c_k[4] = 0.0F;
  c_k[7] = -b_k[2] * t;
  c_k[10] = 0.0F;
  c_k[13] = 0.0F;
  c_k[16] = 0.0F;
  c_k[19] = b_k[6] * t;
  c_k[22] = 0.0F;
  c_k[2] = -b_k[0] * absxk;
  c_k[5] = -b_k[1];
  c_k[8] = -b_k[2] * absxk;
  c_k[11] = -b_k[3];
  c_k[14] = -b_k[4] * absxk;
  c_k[17] = -b_k[5];
  c_k[20] = -b_k[6] * absxk;
  c_k[23] = -b_k[7];
  fv1[0] = -1.0F;
  fv1[8] = 0.0F;
  fv1[16] = b_x;
  fv1[24] = 0.0F;
  fv1[32] = 0.0F;
  fv1[40] = -b_x;
  fv1[1] = h + m;
  fv1[9] = h + m;
  fv1[17] = 1.0F;
  fv1[25] = -1.0F;
  fv1[33] = 1.0F;
  fv1[41] = -1.0F;
  fv1[2] = 0.0F;
  fv1[10] = -1.0F;
  fv1[18] = b_x;
  fv1[26] = 0.0F;
  fv1[34] = 0.0F;
  fv1[42] = b_x;
  fv1[3] = -(h - m);
  fv1[11] = h - m;
  fv1[19] = 1.0F;
  fv1[27] = -1.0F;
  fv1[35] = -1.0F;
  fv1[43] = 1.0F;
  fv1[4] = 1.0F;
  fv1[12] = 0.0F;
  fv1[20] = b_x;
  fv1[28] = -0.0F;
  fv1[36] = 0.0F;
  fv1[44] = -b_x;
  fv1[5] = -(h + m);
  fv1[13] = -(h + m);
  fv1[21] = 1.0F;
  fv1[29] = 1.0F;
  fv1[37] = -1.0F;
  fv1[45] = -1.0F;
  fv1[6] = 0.0F;
  fv1[14] = 1.0F;
  fv1[22] = b_x;
  fv1[30] = 0.0F;
  fv1[38] = 0.0F;
  fv1[46] = b_x;
  fv1[7] = h - m;
  fv1[15] = -(h - m);
  fv1[23] = 1.0F;
  fv1[31] = 1.0F;
  fv1[39] = 1.0F;
  fv1[47] = 1.0F;
  for (i0 = 0; i0 < 8; i0++) {
    for (k = 0; k < 3; k++) {
      d_k[k + 6 * i0] = c_k[k + 3 * i0] / mass;
      d_k[(k + 6 * i0) + 3] = 0.0F;
    }
  }

  d_cthe[0] = b_cthe * scale;
  d_cthe[3] = f_x * k_x * scale - j_x * tol;
  d_cthe[6] = j_x * k_x * scale + f_x * tol;
  d_cthe[9] = ((x[1] * j_x * k_x * scale + x[1] * f_x * tol) - x[2] * f_x * k_x *
               scale) + x[2] * j_x * tol;
  d_cthe[12] = (-x[0] * k_x * scale + x[1] * f_x * b_cthe * scale) + x[2] * j_x *
    b_cthe * scale;
  d_cthe[15] = (((-x[0] * b_cthe * tol - x[1] * f_x * k_x * tol) - x[1] * j_x *
                 scale) - x[2] * j_x * k_x * tol) + x[2] * f_x * scale;
  d_cthe[1] = b_cthe * tol;
  d_cthe[4] = f_x * k_x * tol + j_x * scale;
  d_cthe[7] = j_x * k_x * tol - f_x * scale;
  d_cthe[10] = ((x[1] * j_x * k_x * tol - x[1] * f_x * scale) - x[2] * f_x * k_x
                * tol) - x[2] * j_x * scale;
  d_cthe[13] = ((-x[0] * k_x * tol + x[1] * f_x * b_cthe * tol) + x[1] * j_x *
                scale) + x[2] * j_x * b_cthe * tol;
  d_cthe[16] = (((x[0] * b_cthe * scale + x[1] * f_x * k_x * scale) - x[1] * j_x
                 * tol) + x[2] * j_x * k_x * scale) + x[2] * f_x * tol;
  d_cthe[2] = -k_x;
  d_cthe[5] = f_x * b_cthe;
  d_cthe[8] = j_x * b_cthe;
  d_cthe[11] = x[1] * j_x * b_cthe - x[2] * f_x * b_cthe;
  d_cthe[14] = (-x[0] * b_cthe - x[1] * k_x * f_x) - x[2] * j_x * k_x;
  d_cthe[17] = 0.0F;
  for (i0 = 0; i0 < 6; i0++) {
    for (k = 0; k < 3; k++) {
      e_k[i0 + 6 * k] = 0.0F;
      for (vcol = 0; vcol < 8; vcol++) {
        e_k[i0 + 6 * k] += d_k[i0 + 6 * vcol] * fv1[vcol + (k << 3)];
      }
    }
  }

  for (i0 = 0; i0 < 3; i0++) {
    for (k = 0; k < 3; k++) {
      c_cthe[i0 + 3 * k] = 0.0F;
      for (vcol = 0; vcol < 6; vcol++) {
        c_cthe[i0 + 3 * k] += d_cthe[i0 + 3 * vcol] * e_k[vcol + 6 * k];
      }
    }
  }

  svd(c_cthe, U, S, V);
  tol = 3.0F * S[0] * 1.1920929E-7F;
  r = 0;
  k = 0;
  while ((k + 1 < 4) && (S[k + 3 * k] > tol)) {
    r++;
    k++;
  }

  if (r > 0) {
    vcol = 0;
    for (ar = 0; ar + 1 <= r; ar++) {
      tol = 1.0F / S[ar + 3 * ar];
      for (k = vcol; k + 1 <= vcol + 3; k++) {
        V[k] *= tol;
      }

      vcol += 3;
    }

    for (k = 0; k <= 7; k += 3) {
      for (ic = k; ic + 1 <= k + 3; ic++) {
        X[ic] = 0.0F;
      }
    }

    vcol = -1;
    for (k = 0; k <= 7; k += 3) {
      ar = -1;
      vcol++;
      i0 = (vcol + 3 * (r - 1)) + 1;
      for (ib = vcol; ib + 1 <= i0; ib += 3) {
        if (U[ib] != 0.0F) {
          ia = ar;
          for (ic = k; ic + 1 <= k + 3; ic++) {
            ia++;
            X[ic] += U[ib] * V[ia];
          }
        }

        ar += 3;
      }
    }
  }

  e_cthe[0] = cthe * h_x;
  e_cthe[3] = c_x * e_x * h_x - d_x * g_x;
  e_cthe[6] = d_x * e_x * h_x + c_x * g_x;
  e_cthe[9] = ((x[1] * d_x * e_x * h_x + x[1] * c_x * g_x) - x[2] * c_x * e_x *
               h_x) + x[2] * d_x * g_x;
  e_cthe[12] = (-x[0] * e_x * h_x + x[1] * c_x * cthe * h_x) + x[2] * d_x * cthe
    * h_x;
  e_cthe[15] = (((-x[0] * cthe * g_x - x[1] * c_x * e_x * g_x) - x[1] * d_x *
                 h_x) - x[2] * d_x * e_x * g_x) + x[2] * c_x * h_x;
  e_cthe[1] = cthe * g_x;
  e_cthe[4] = c_x * e_x * g_x + d_x * h_x;
  e_cthe[7] = d_x * e_x * g_x - c_x * h_x;
  e_cthe[10] = ((x[1] * d_x * e_x * g_x - x[1] * c_x * h_x) - x[2] * c_x * e_x *
                g_x) - x[2] * d_x * h_x;
  e_cthe[13] = ((-x[0] * e_x * g_x + x[1] * c_x * cthe * g_x) + x[1] * d_x * h_x)
    + x[2] * d_x * cthe * g_x;
  e_cthe[16] = (((x[0] * cthe * h_x + x[1] * c_x * e_x * h_x) - x[1] * d_x * g_x)
                + x[2] * d_x * e_x * h_x) + x[2] * c_x * g_x;
  e_cthe[2] = -e_x;
  e_cthe[5] = c_x * cthe;
  e_cthe[8] = d_x * cthe;
  e_cthe[11] = x[1] * d_x * cthe - x[2] * c_x * cthe;
  e_cthe[14] = (-x[0] * cthe - x[1] * e_x * c_x) - x[2] * d_x * e_x;
  e_cthe[17] = 0.0F;
  for (i0 = 0; i0 < 3; i0++) {
    tol = 0.0F;
    for (k = 0; k < 6; k++) {
      tol += e_cthe[i0 + 3 * k] * xdot_s[k];
    }

    b_vel_ff[i0] = (vel_ff[i0] - tol) + vel_dot_ref[i0];
  }

  for (i0 = 0; i0 < 3; i0++) {
    acc_com[i0] = 0.0F;
    for (k = 0; k < 3; k++) {
      acc_com[i0] += X[i0 + 3 * k] * b_vel_ff[k];
    }
  }

  //  u = G^(-1)*[rdot - F + K*e];
}

//
// File trailer for ndi_control.cpp
//
// [EOF]
//
