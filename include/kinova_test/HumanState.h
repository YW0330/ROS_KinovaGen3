#ifndef _HUMANSTATE_H_
#define _HUMANSTATE_H_

#include "xsens_mtw_driver/xsens2kinova.h"
#include "kinova_test/Matrix.h"

class HumanState
{
public:
    HumanState();
    Matrix<double> Xd;
    Matrix<double> dXd;
    double finger_pitch;
    void updateHumanData(const xsens_mtw_driver::xsens2kinova &msg);
};

#endif