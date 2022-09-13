#ifndef _MYLIB_H_
#define _MYLIB_H_

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
 * 輸出扭矩飽和器。
 *
 * @param 7*1 的輸出扭矩向量
 *
 */
void torque_satuation(Matrix<double> &tau);

/*
 * 重力補償。
 *
 * @param q: 7*1 關節角度
 *        init_tau: 7*1 初始扭矩
 *        tau: 7*1 的輸出扭矩向量
 */
void gravity_compensation(const Matrix<double> &q, const double init_tau[7], Matrix<double> &tau);

/*
 * 將關節角度轉到 -inf 到 inf。
 *
 * @param curr_pos: 7*1 當前關節角度 (-pi~pi)
 *        prev_q: 7*1 先前關節角度 (-inf~inf)
 *        round: 7*1 各關節圈數
 *        q: 7*1 當前關節角度 (-inf~inf)
 */
void q2inf(const Matrix<double> &curr_pos, const Matrix<double> &prev_q, Matrix<int> &round, Matrix<double> &q);

#endif