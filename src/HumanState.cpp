#include "kinova_test/HumanState.h"
#include "kinova_test/controller.h"

HumanState::HumanState() : Xd(DOF, 1), dXd(DOF, 1), current_mode(ControlMode::Platform), triggerVal(0), stop(false) {}

void HumanState::updateHumanData(const xsens_mtw_driver::xsens2kinova &msg)
{
#if (DOF == 3)
    for (unsigned i = 0; i < 3; i++)
    {
        Xd[i] = msg.position[i];
        dXd[i] = msg.velocity_pos[i];
    }
#else
    for (unsigned i = 0; i < 3; i++)
    {
        Xd[i] = msg.position[i];
        Xd[i + 3] = msg.attitude[i];
        dXd[i] = msg.velocity_pos[i];
        dXd[i + 3] = msg.velocity_att[i];
    }
#endif
}

void HumanState::updateTriggerValue(const std_msgs::Float32 &msg)
{
    triggerVal = msg.data;
}

void HumanState::updateStopState(const std_msgs::Bool &msg)
{
    stop = msg.data;
}

void HumanState::updateControlMode(const std_msgs::Bool &msg)
{
    current_mode = (ControlMode)msg.data;
    if (current_mode == ControlMode::Manipulator)
        manipulator_mapping();
}

void HumanState::manipulator_mapping()
{
    Xd[0] *= 2;
    Xd[2] *= 1.5;
}