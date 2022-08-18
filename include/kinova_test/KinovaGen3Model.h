#ifndef _KINOVAGEN3MODEL_H_
#define _KINOVAGEN3MODEL_H_

#include "Matrix.h"

#define TOMETER(num) (double)num / 10000

// Type Definitions from MATLAB
void kinova_M(double q1, double q2, double q3, double q4, double q5,
              double q6, double q7, double M[49]);

void kinova_C(double q1, double q2, double q3, double q4, double q5,
              double q6, double q7, double dq1, double dq2, double dq3,
              double dq4, double dq5, double dq6, double dq7, double C[49]);

void kinova_G(double g, double q1, double q2, double q3, double q4,
              double q5, double q6, double q7, double G[7]);

void kinova_M_gripper(double q1, double q2, double q3, double q4,
                      double q5, double q6, double q7, double M[49]);

void kinova_C_gripper(double q1, double q2, double q3, double q4, double q5,
                      double q6, double q7, double dq1, double dq2, double dq3,
                      double dq4, double dq5, double dq6, double dq7, double C[49]);

void kinova_G_gripper(double g, double q1, double q2, double q3,
                      double q4, double q5, double q6, double q7,
                      double G[7]);

void kinova_M_gripper2(double q1, double q2, double q3, double q4,
                       double q5, double q6, double q7, double M[49]);

void kinova_C_gripper2(double q1, double q2, double q3, double q4,
                       double q5, double q6, double q7, double dq1,
                       double dq2, double dq3, double dq4, double dq5,
                       double dq6, double dq7, double C[49]);

void kinova_J(double q1, double q2, double q3, double q4, double q5,
              double q6, double J[42]);

void kinova_J_and_Jinv(double q1, double q2, double q3, double q4, double q5, double q6, double J[42], double Jinv[42]);

// Parameters
enum class Params
{
    // unit: meter*10000
    d0 = 1564,
    d1 = 1284,
    d2 = 54,
    d3 = 2104,
    d4 = 64,
    d5 = 2104,
    d6 = 64,
    d7 = 2084,
    d8 = 64,
    d9 = 1059,
    d10 = 1059,
    d11 = 615,
};

Matrix<double> forward_kinematic(Matrix<double> q);
#endif
