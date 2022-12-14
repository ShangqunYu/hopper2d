//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// jacobian_b.cpp
//
// Code generation for function 'jacobian_b'
//

// Include files
#include "jacobian_b.h"
#include <cmath>

// Function Definitions
void jacobian_b(const double in1[12], const double in2[17], double J_b[18])
{
  double J_b_tmp;
  double t10;
  double t11;
  double t2;
  double t4;
  // JACOBIAN_B
  //     J_b = JACOBIAN_B(IN1,IN2)
  //     This function was generated by the Symbolic Math Toolbox version 9.1.
  //     25-Sep-2022 00:50:44
  t2 = in1[2] + in1[3];
  t4 = in1[4] + t2;
  t10 = in2[12] * std::cos(t4);
  t11 = in2[12] * std::sin(t4);
  J_b[0] = 1.0;
  J_b[1] = 0.0;
  J_b[2] = 0.0;
  J_b[3] = 0.0;
  J_b[4] = 1.0;
  J_b[5] = 0.0;
  J_b_tmp = in2[11] * std::cos(t2) + t10;
  J_b[6] = J_b_tmp + in2[16] * std::cos(in1[2]) / 2.0;
  t4 = in2[11] * std::sin(t2) + t11;
  J_b[7] = t4 + in2[16] * std::sin(in1[2]) / 2.0;
  J_b[8] = 1.0;
  J_b[9] = J_b_tmp;
  J_b[10] = t4;
  J_b[11] = 1.0;
  J_b[12] = t10;
  J_b[13] = t11;
  J_b[14] = 1.0;
  J_b[15] = 0.0;
  J_b[16] = 0.0;
  J_b[17] = 1.0;
}

// End of code generation (jacobian_b.cpp)
