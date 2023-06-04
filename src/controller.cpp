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
    const double q_max[7] = {q1_MAX, q2_MAX, q3_MAX, q4_MAX, q5_MAX, q6_MAX, q7_MAX};
    const double q_min[7] = {q1_MIN, q2_MIN, q3_MIN, q4_MIN, q5_MIN, q6_MIN, q7_MIN};
    double psi_arr[7];
    Matrix<double> psi_tmp(7, 1);
    kinova_psi_jointAngleLimits(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q_max[0], q_max[1], q_max[2], q_max[3], q_max[4], q_max[5], q_max[6], q_min[0], q_min[1], q_min[2], q_min[3], q_min[4], q_min[5], q_min[6], psi_arr);
    for (int i = 0; i < 7; i++)
        if ((q_max[i] > 0 && q_min[i] < 0) || (q_max[i] < 0 && q_min[i] > 0))
            psi[i] = -psi[i];
    // 僅限制第 2 4 6 軸
    for (int i = 0; i < 7; i += 2)
        psi_arr[i] = 0;
    psi_tmp.update_from_matlab(psi_arr);
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

void humanPos2platformVel(HumanState &humanState, geometry_msgs::Twist &twist)
{
    // 前進後退
    if (humanState.Xd[2] > 0.2)
        twist.linear.x = 1.25 * humanState.Xd[2] - 0.25; // z: 0.2~0.6
    else if (humanState.Xd[2] < -0.2)
        twist.linear.x = (5 / 3) * humanState.Xd[2] + (0.5 / 3); // z: -0.1~-0.4
    else
        twist.linear.x = 0;

    // 旋轉
    if (humanState.Xd[1] > -0.2)
        twist.angular.z = 5 * humanState.Xd[1] + 1; // y: -0.2~0
    else if (humanState.Xd[1] < -0.5)
        twist.angular.z = 5 * humanState.Xd[1] + 2.5; // y: -0.5~-0.7
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