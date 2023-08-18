#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <cmath>
#include "kinova_test/Matrix.h"
#include "kinova_test/KinovaGen3Model.h"
#include "kinova_test/HumanState.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Controller Parameters
#define DOF 6U
#define NODE 25U

#if (DOF == 3)
#define LAMBDA_INITLIST \
    {                   \
        15, 15, 25      \
    }
#else
#define LAMBDA_INITLIST     \
    {                       \
        10, 10, 12, 8, 8, 8 \
    }
#endif

#define K_INITLIST           \
    {                        \
        5, 10, 5, 8, 5, 8, 5 \
    }
#define Kj 0.5
#define Kr 25U
#define Gamma 2.5
#define Bj 20

#define Ks_MANIPULABILITY 2
#define Ks_JOINT_LIMIT 2

// all: JML_JOINT_ALL, Only even joints (2,4,6): JML_JOINT_246
#define JML_JOINT_246
#define q1_MAX (100 * DEG2RAD)
#define q1_MIN (-100 * DEG2RAD)
#define q2_MAX (110 * DEG2RAD)
#define q2_MIN (-110 * DEG2RAD)
#define q3_MAX (100 * DEG2RAD)
#define q3_MIN (-100 * DEG2RAD)
#define q4_MAX (147.8 * DEG2RAD)
#define q4_MIN (-147.8 * DEG2RAD)
#define q5_MAX (100 * DEG2RAD)
#define q5_MIN (-100 * DEG2RAD)
#define q6_MAX (120.3 * DEG2RAD)
#define q6_MIN (-120.3 * DEG2RAD)
#define q7_MAX (100 * DEG2RAD)
#define q7_MIN (-100 * DEG2RAD)

#define PLATFORM_pLINEAR_MAX 1
#define PLATFORM_nLINEAR_MAX (-1)
#define USER_pZ_MAX 0.6
#define USER_pZ_MIN 0.2
#define USER_nZ_MAX (-0.6)
#define USER_nZ_MIN (-0.2)
#define PLATFORM_pANGULAR_MAX 0.5
#define PLATFORM_nANGULAR_MAX (-0.5)
#define USER_pY_MAX (0.45)
#define USER_pY_MIN (0.05)
#define USER_nY_MAX (-0.5)
#define USER_nY_MIN (-0.1)

// functions
namespace chang
{
    void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi, unsigned joint);
    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat, unsigned joint);
    void controller(const Matrix<double> &J, const Matrix<double> &dx, const Matrix<double> &dxd, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &sigma, Matrix<double> &tau);
}
void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r);
void joint_angle_limit_psi(const Matrix<double> &q, Matrix<double> &psi);
void manipulability_psi(const Matrix<double> &q, Matrix<double> &psi);
void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, Matrix<double> &subtasks);
void humanPos2platformVel(Matrix<double> &Xd, geometry_msgs::Twist &twist);
void emergency_stop(ros::Publisher &platform_pub);
#endif