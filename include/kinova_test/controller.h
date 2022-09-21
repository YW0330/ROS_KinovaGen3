#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <cmath>
#include "kinova_test/Matrix.h"
#include "kinova_test/KinovaGen3Model.h"

// Parameters
#define DOF 3U
#define NODE 300U
const double kLambda = 20;
const double kK = 8;
const double kKj = 1;
const double kKr = 8;
const double kEta = 2;
const double kBj = 1;
const double kCj_v[2] = {-1, 1};
const double kCj_a[2] = {-1, 1};
const double kCj_q[2] = {-2 * M_PI, 2 * M_PI};
const double kCj_dq[2] = {-2, 2};
const double q_max[7] = {100, 100 * DEG2RAD, 100, 147.8 * DEG2RAD, 100, 120.3 * DEG2RAD, 100};
const double q_min[7] = {-100, -100 * DEG2RAD, -100, -147.8 * DEG2RAD, -100, -120.3 * DEG2RAD, -100};

// functions
void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r);
void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi);
void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat);
void controller(const Matrix<double> &J, const Matrix<double> &de, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &phi, const Matrix<double> &W_hat, Matrix<double> &tau);
void joint_angle_limit_psi(const Matrix<double> &q, Matrix<double> &psi);
void manipulability_psi(const Matrix<double> &q, Matrix<double> &psi);
void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, Matrix<double> &subtasks);
#endif