#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <cmath>
#include "kinova_test/Matrix.h"
#include "kinova_test/KinovaGen3Model.h"
#include "kinova_test/HumanState.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Parameters
#define DOF 6U
#define NODE 25U

#if (DOF == 3)
#define LAMBDA_INITLIST \
    {                   \
        20, 20, 40      \
    }
#else
#define LAMBDA_INITLIST        \
    {                          \
        20, 20, 40, 10, 10, 10 \
    }
#endif

#define K_INITLIST            \
    {                         \
        8, 15, 8, 10, 5, 8, 5 \
    }
#define Kj 0.8
#define Kr 10U
#define Gamma 2
#define Bj 20
#define Cj_v_UP 1U
#define Cj_v_LOW (-1)
#define Cj_a_UP 1U
#define Cj_a_LOW (-1)
#define Cj_q_UP (2 * M_PI)
#define Cj_q_LOW (-2 * M_PI)
#define Cj_dq_UP 2U
#define Cj_dq_LOW (-2)

#define q1_MAX 100U
#define q1_MIN (-100)
#define q2_MAX (110 * DEG2RAD)
#define q2_MIN (-110 * DEG2RAD)
#define q3_MAX 100U
#define q3_MIN (-100)
#define q4_MAX (147.8 * DEG2RAD)
#define q4_MIN (-147.8 * DEG2RAD)
#define q5_MAX 100U
#define q5_MIN (-100)
#define q6_MAX (120.3 * DEG2RAD)
#define q6_MIN (-120.3 * DEG2RAD)
#define q7_MAX 100U
#define q7_MIN (-100)

#define Ks_MANIPULABILITY 3U
#define Ks_JOINT_LIMIT (-5) // 全部的 qmax 跟 qmin 反向

// functions
namespace hsu
{
    void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi);
    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat);
    void controller(const Matrix<double> &J, const Matrix<double> &dx, const Matrix<double> &dxd, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &phi, const Matrix<double> &W_hat, Matrix<double> &tau);
}
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
void humanPos2platformVel(HumanState &humanState, geometry_msgs::Twist &twist);
void emergency_stop(ros::Publisher &platform_pub);
#endif