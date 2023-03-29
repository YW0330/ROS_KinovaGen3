#ifndef _HUMANSTATE_H_
#define _HUMANSTATE_H_

#include "xsens_mtw_driver/xsens2kinova.h"
#include "kinova_test/Matrix.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

enum class ControlMode
{
    Platform,
    Manipulator
};

class HumanState
{
public:
    HumanState();
    Matrix<double> Xd;
    Matrix<double> dXd;
    float triggerVal;
    ControlMode current_mode;
    void updateHumanData(const xsens_mtw_driver::xsens2kinova &msg);
    void updateControlMode(const std_msgs::Bool &msg);
    void updateTriggerValue(const std_msgs::Float32 &msg);

private:
    void manipulator_mapping();
};

#endif