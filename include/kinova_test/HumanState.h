#ifndef _HUMANSTATE_H_
#define _HUMANSTATE_H_

#include "xsens_mtw_driver/xsens2kinova.h"
#include "kinova_test/Matrix.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#define X_MAPPING 2
#define Z_MAPPING 1.5

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
    bool stop;
    void updateHumanData(const xsens_mtw_driver::xsens2kinova &msg);
    void updateControlMode(const std_msgs::Bool &msg);
    void updateTriggerValue(const std_msgs::Float32 &msg);
    void updateStopState(const std_msgs::Bool &msg);

private:
    void manipulator_mapping();
};

#endif