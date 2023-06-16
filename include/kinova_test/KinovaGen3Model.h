#ifndef _KINOVAGEN3MODEL_H_
#define _KINOVAGEN3MODEL_H_

#include "Matrix.h"

#define DEG2RAD M_PI / 180
#define GRAVITY 9.80665
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

void kinova_J_and_Jinv(double q1, double q2, double q3, double q4, double q5, double q6, double J[42], double Jinv[42]);

void kinova_psi_jointAngleLimits_all(
    double q1, double q2, double q3, double q4, double q5, double q6, double q7,
    double q1min, double q2min, double q3min, double q4min, double q5min,
    double q6min, double q7min, double q1max, double q2max, double q3max,
    double q4max, double q5max, double q6max, double q7max, double psi[7]);

void kinova_psi_jointAngleLimits_246(double q2, double q4, double q6,
                                     double q2max, double q4max,
                                     double q6max, double q2min,
                                     double q4min, double q6min,
                                     double psi2[7]);

void kinova_psi_manipulability(double q1, double q2, double q3, double q4,
                               double q5, double q6, double psi[7]);

double kinova_manipulability(double q1, double q2, double q3, double q4,
                             double q5, double q6);

void kinova_FK(double q1, double q2, double q3, double q4, double q5,
               double q6, double q7, double X[6]);

void kinova_FK_axisAngle(double q1, double q2, double q3, double q4,
                         double q5, double q6, double q7, double X[7]);

// Parameters
enum class KinovaParams
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
    d12 = 1200
};

Matrix<double> forward_kinematic_3dof(const Matrix<double> &q);
Matrix<double> forward_kinematic_6dof(const Matrix<double> &q);
Matrix<double> kinova_axisAngle(const Matrix<double> &q);
Matrix<double> forward_kinematic_6dof_matlab(const Matrix<double> &q);
#endif
