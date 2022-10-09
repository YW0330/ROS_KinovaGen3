#ifndef _HUMANSTATE_H_
#define _HUMANSTATE_H_

#include "xsens_mtw_driver/xsens2kinova.h"
#include "Matrix.h"
#include "controller.h"

class HumanState
{
public:
    HumanState();
    Matrix<double> Xd;
    Matrix<double> dXd;
    double finger_pitch;
    void updateHumanData(const xsens_mtw_driver::xsens2kinova &msg);
};

HumanState::HumanState() : Xd(DOF, 1), dXd(DOF, 1) {}

void HumanState::updateHumanData(const xsens_mtw_driver::xsens2kinova &msg)
{
    if (DOF == 3)
    {
        for (unsigned i = 0; i < 3; i++)
        {
            Xd[i] = msg.position[i];
            dXd[i] = msg.velocity_pos[i];
        }
    }
    else
    {
        for (unsigned i = 0; i < 3; i++)
        {
            Xd[i] = msg.position[i];
            Xd[i + 3] = msg.attitude[i];
            dXd[i] = msg.velocity_pos[i];
            dXd[i + 3] = msg.velocity_att[i];
        }
    }
    finger_pitch = msg.finger_pitch;
}

#endif