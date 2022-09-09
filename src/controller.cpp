#include "kinova_test/controller.h"

void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r)
{
    s = -Jinv * kLambda * e + dq - subtasks;
    v = Jinv * kLambda * e + subtasks;
    a = dJinv * kLambda * e + Jinv * kLambda * de + dsubtasks;
    r = J * s;
}

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
                    cj = kCj_v[0] + (kCj_v[1] - kCj_v[0]) * i / (NODE - 1);
                }
                else if (k == 1)
                {
                    X = a[j];
                    cj = kCj_a[0] + (kCj_a[1] - kCj_a[0]) * i / (NODE - 1);
                }
                else if (k == 2)
                {
                    X = q[j];
                    cj = kCj_q[0] + (kCj_q[1] - kCj_q[0]) * i / (NODE - 1);
                }
                else
                {
                    X = dq[j];
                    cj = kCj_dq[0] + (kCj_dq[1] - kCj_dq[0]) * i / (NODE - 1);
                }
                distance_square += (X - cj) * (X - cj);
            }
            phi(i, j) = exp(-distance_square / (kBj * kBj));
        }
    }
}

void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat)
{
    dW_hat = -kEta * phi * s;
}

void controller(const Matrix<double> &J, const Matrix<double> &de, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &phi, const Matrix<double> &W_hat, Matrix<double> &tau)
{
    Matrix<double> tau_bar = kKr * r - kKj * de;
    tau = phi.transpose() * W_hat - kK * s - J.transpose() * tau_bar;
}
