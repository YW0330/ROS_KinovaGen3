#include "kinova_test/HumanState.h"
#include "kinova_test/controller.h"

HumanState::HumanState() : Xd(DOF, 1), dXd(DOF, 1), current_mode(ControlMode::Platform) {}

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
    Xd[0] *= 1.5;
    finger_pitch = msg.finger_pitch;
    current_mode = (ControlMode)msg.mode;
}