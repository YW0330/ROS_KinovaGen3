#include "kinova_test/controller.h"

void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r)
{
    Matrix<double> lambda(DOF, DOF, MatrixType::Diagonal, LAMBDA_INITLIST);
    s = -Jinv * lambda * e + dq - subtasks;
    v = Jinv * lambda * e + subtasks;
    a = dJinv * lambda * e + Jinv * lambda * de + dsubtasks;
    r = J * s;
}
namespace hsu
{
    void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi)
    {
        double distance_square, cj, X;
        for (unsigned i = 0; i < NODE; i++)
        {
            for (unsigned j = 0; j < 7; j++)
            {
                distance_square = 0;
                for (unsigned k = 0; k < 4; k++)
                {
                    if (k == 0)
                    {
                        X = v[j];
                        cj = Cj_v_LOW + ((Cj_v_UP - Cj_v_LOW) / (NODE - 1)) * i;
                    }
                    else if (k == 1)
                    {
                        X = a[j];
                        cj = Cj_a_LOW + ((Cj_a_UP - Cj_a_LOW) / (NODE - 1)) * i;
                    }
                    else if (k == 2)
                    {
                        X = q[j];
                        cj = Cj_q_LOW + ((Cj_q_UP - Cj_q_LOW) / (NODE - 1)) * i;
                    }
                    else
                    {
                        X = dq[j];
                        cj = Cj_dq_LOW + ((Cj_dq_UP - Cj_dq_LOW) / (NODE - 1)) * i;
                    }
                    distance_square += (X - cj) * (X - cj);
                }
                phi(i, j) = exp(-distance_square / (Bj * Bj));
            }
        }
    }

    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat)
    {
        dW_hat = -Gamma * phi * s;
    }

    void controller(const Matrix<double> &J, const Matrix<double> &dx, const Matrix<double> &dxd, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &phi, const Matrix<double> &W_hat, Matrix<double> &tau)
    {
        Matrix<double> K(7, 7, MatrixType::Diagonal, K_INITLIST);
        // Matrix<double> tau_bar = Kr * r - Kj * (dxd - dx) + PINV(r.transpose()) * dx.transpose() * Kj * dxd;
        Matrix<double> tau_bar = Kr * r;

        tau = phi.transpose() * W_hat - K * s - J.transpose() * tau_bar;
    }
}

namespace chang
{
    void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi, unsigned joint)
    {
        Matrix<double> X(4, 1, MatrixType::General, {v[joint], a[joint], q[joint], dq[joint]});
        for (unsigned i = 0; i < NODE; i++)
        {
            Matrix<double> cj(4, 1);
            // v
            cj[0] = -40 + (80 / (NODE - 1)) * i;

            // a
            cj[1] = -80 + (160 / (NODE - 1)) * i;

            // q
            if (joint == 1) // joint 2
                cj[2] = (-128.9 * DEG2RAD) + ((257.8 * DEG2RAD) / (NODE - 1)) * i;
            else if (joint == 3) // joint 4
                cj[2] = (-147.8 * DEG2RAD) + ((295.6 * DEG2RAD) / (NODE - 1)) * i;
            else if (joint == 5) // joint 5
                cj[2] = (-120.3 * DEG2RAD) + ((240.6 * DEG2RAD) / (NODE - 1)) * i;
            else // joint 1 3 5 7
                cj[2] = (-2 * M_PI) + ((4 * M_PI) / (NODE - 1)) * i;

            // dq
            if (joint < 4) // joint 1-4
                cj[3] = (-1.39) + (2.78 / (NODE - 1)) * i;
            else // joint 5-7
                cj[3] = (-1.22) + (2.44 / (NODE - 1)) * i;

            double norm = (X - cj).vec_norm2();
            phi[i] = exp(-(norm * norm) / (Bj * Bj));
        }
    }

    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat, unsigned joint)
    {
        dW_hat = -Gamma * phi * s[joint];
    }

    void controller(const Matrix<double> &J, const Matrix<double> &dx, const Matrix<double> &dxd, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &sigma, Matrix<double> &tau)
    {
        Matrix<double> K(7, 7, MatrixType::Diagonal, K_INITLIST);
        Matrix<double> tau_bar = Kr * r - Kj * (dxd - dx) + PINV(r.transpose()) * dx.transpose() * Kj * dxd;
        tau = sigma - K * s - J.transpose() * tau_bar;
    }
}

void joint_angle_limit_psi(const Matrix<double> &q, Matrix<double> &psi)
{
    double psi_arr[7];
    Matrix<double> psi_tmp(7, 1);
#ifdef JML_JOINT_ALL
    const double q_max[7] = {q1_MAX, q2_MAX, q3_MAX, q4_MAX, q5_MAX, q6_MAX, q7_MAX};
    const double q_min[7] = {q1_MIN, q2_MIN, q3_MIN, q4_MIN, q5_MIN, q6_MIN, q7_MIN};
    kinova_psi_jointAngleLimits_all(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q_max[0], q_max[1], q_max[2], q_max[3], q_max[4], q_max[5], q_max[6], q_min[0], q_min[1], q_min[2], q_min[3], q_min[4], q_min[5], q_min[6], psi_arr);
    psi_tmp.update_from_matlab(psi_arr);
    for (unsigned int i = 0, k = 0; i < 7; i++, k++)
#elif defined(JML_JOINT_246)
    const double q_max[3] = {q2_MAX, q4_MAX, q6_MAX};
    const double q_min[3] = {q2_MIN, q4_MIN, q6_MIN};
    kinova_psi_jointAngleLimits_246(q[1], q[3], q[5], q_max[0], q_max[1], q_max[2], q_min[0], q_min[1], q_min[2], psi_arr);
    psi_tmp.update_from_matlab(psi_arr);
    for (unsigned int i = 0, k = 1; i < 3; i++, k += 2)
#endif
        if ((q_max[i] > 0 && q_min[i] < 0) || (q_max[i] < 0 && q_min[i] > 0))
            psi_tmp[k] = -psi_tmp[k];
    psi += Ks_JOINT_LIMIT * psi_tmp;
}

void manipulability_psi(const Matrix<double> &q, Matrix<double> &psi)
{
    double psi_arr[7];
    Matrix<double> psi_tmp(7, 1);
    kinova_psi_manipulability(q[0], q[1], q[2], q[3], q[4], q[5], psi_arr);
    psi_tmp.update_from_matlab(psi_arr);
    psi += Ks_MANIPULABILITY * psi_tmp;
}

void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, Matrix<double> &subtasks)
{
    Matrix<double> eye(7, 7, MatrixType::Diagonal, {1, 1, 1, 1, 1, 1, 1});
    subtasks = (eye - Jinv * J) * psi;
    psi.zeros();
}

void humanPos2platformVel(Matrix<double> &Xd, geometry_msgs::Twist &twist)
{
    // 前進後退
    if (Xd[2] > USER_pZ_MIN)
        twist.linear.x = PLATFORM_pLINEAR_MAX * (Xd[2] - USER_pZ_MAX) / (USER_pZ_MAX - USER_pZ_MIN) + PLATFORM_pLINEAR_MAX;
    else if (Xd[2] < USER_nZ_MIN)
        twist.linear.x = PLATFORM_nLINEAR_MAX * (Xd[2] - USER_nZ_MAX) / (USER_nZ_MAX - USER_nZ_MIN) + PLATFORM_nLINEAR_MAX;
    else
        twist.linear.x = 0;

    // 旋轉
    if (Xd[1] > USER_pY_MIN)
        twist.angular.z = PLATFORM_pANGULAR_MAX * (Xd[1] - USER_pY_MAX) / (USER_pY_MAX - USER_pY_MIN) + PLATFORM_pANGULAR_MAX;
    else if (Xd[1] < USER_nY_MIN)
        twist.angular.z = PLATFORM_nANGULAR_MAX * (Xd[1] - USER_nY_MAX) / (USER_nY_MAX - USER_nY_MIN) + PLATFORM_nANGULAR_MAX;
    else
        twist.angular.z = 0;
}

void emergency_stop(ros::Publisher &platform_pub)
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    platform_pub.publish(twist);
}