#ifndef MYLIB_H
#define MYLIB_H

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
#include "utilities.h"
#include <google/protobuf/util/json_util.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define WAITING_TIME 20

#define Deg2Rad M_PI / 180
#define GRAVITY 9.80665

namespace k_api = Kinova::Api;

int64_t GetTickUs();
bool example_move_to_home_position(k_api::Base::BaseClient *base);
void torque_satuation(double tau[7]);
#endif