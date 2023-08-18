#ifndef _KINOVACONFIG_H_
#define _KINOVACONFIG_H_

#include <iomanip>
#include <cmath>

#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

#include "utilities.h"
#include "kinova_test/Matrix.h"
#include "kinova_test/KinovaGen3Model.h"

#include "ros/ros.h"

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define WAITING_TIME 20

namespace k_api = Kinova::Api;

int64_t GetTickUs();
bool example_move_to_home_position(k_api::Base::BaseClient *base);

/*
 * 回歸 Home 姿態同時傳送各關節角度。
 *
 * @param kinova_pub: 傳送 kinovaMsg 的 Publisher
 *
 */
bool move_to_home_position_with_ros(k_api::Base::BaseClient *base, ros::Publisher &kinova_pub);

/*
 * 輸出扭矩飽和器。
 *
 * @param tau: 7*1 的輸出扭矩向量
 *
 */
void torque_saturation(Matrix<double> &tau);

/*
 * 重力補償。
 *
 * @param q: 7*1 關節角度
 * @param init_tau: 7*1 初始扭矩
 * @param tau: 7*1 的輸出扭矩向量
 */
void gravity_compensation(const Matrix<double> &q, const double init_tau[7], Matrix<double> &tau);

/*
 * 將關節角度轉到 -inf 到 inf。
 *
 * @param curr_pos: 7*1 當前關節角度 (-pi~pi)
 * @param prev_q: 7*1 先前關節角度 (-inf~inf)
 * @param round: 7*1 各關節圈數
 * @param q: 7*1 當前關節角度 (-inf~inf)
 */
void q2inf(const Matrix<double> &curr_pos, const Matrix<double> &prev_q, Matrix<int> &round, Matrix<double> &q);

/*
 * Kinova 末端夾爪開闔。
 *
 * @param gripper_motor_command: k_api::GripperCyclic::MotorCommand 指標型態
 * @param triggerVal: 操作員左手控制器的 trigger button value
 */
void gripper_control(k_api::GripperCyclic::MotorCommand *gripper_motor_command, float triggerVal);
#endif