#ifndef KINOVAGEN3MODEL_H
#define KINOVAGEN3MODEL_H

// Type Definitions
void kinova_M(double q1, double q2, double q3, double q4, double q5,
              double q6, double q7, double M[49]);

void kinova_G(double g, double q1, double q2, double q3, double q4,
              double q5, double q6, double q7, double G[7]);

void kinova_C(double q1, double q2, double q3, double q4, double q5,
              double q6, double q7, double dq1, double dq2, double dq3,
              double dq4, double dq5, double dq6, double dq7, double C[49]);
#endif