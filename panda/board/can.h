#pragma once

#define PANDA_CAN_CNT 3U
#define PANDA_BUS_CNT 3U

#include "opendbc/safety/can.h"

#define GET_BUS(msg) ((msg)->bus)
#define GET_ADDR(msg) ((msg)->addr)
