#pragma once

#ifndef PG_DYN_PID_CONFIG
#define PG_DYN_PID_CONFIG 1507
#endif

#define RC_RANGE_MIN 172
#define RC_RANGE_MAX 1811
#define RC_RANGE_TERM 992
#define RC_RANGE_WIDE 1639
#define RC_RANGE_TH_MIN 271
#define RC_RANGE_TH_MAX 1631
#define RC_RANGE_TH_WIDE 1360

#ifndef PI
#define PI 3.14159265358979323846
#endif

#define PACKED __attribute__((packed))
#define KFIFO_ATOMIC_PRIO NVIC_PRIO_TIMER
#define MIN_KFIFO_SIZE 16
#define MAX_KFIFO_SIZE 4096
#define IS_POWER_OF_2(x) ((x) != 0 && (((x) & ((x) - 1)) == 0))
#define KFIFO_MASK(size) ((size) - 1)

#define FOLLOW_CAMERA_ANGLE_RAD 1.5708f
#define FOLLOW_GUIDANCE_GAIN 10

#ifndef FOLLOW_VERSION
#define FOLLOW_VERSION "dev"
#endif

#ifndef FOLLOW_DESCRIPTION
#define FOLLOW_DESCRIPTION ""
#endif

#ifndef FOLLOW_WORK_SERIAL_PORT
#define FOLLOW_WORK_SERIAL_PORT SERIAL_PORT_UART4
#endif

#ifndef FOLLOW_SIM_SERIAL_PORT
#define FOLLOW_SIM_SERIAL_PORT SERIAL_PORT_USART3
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"
#include "build/debug.h"
#include "common/axis.h"
#include "common/printf.h"
#include "common/time.h"
#include "common/utils.h"
#include "config/config.h"
#include "drivers/flash.h"
#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "fc/dispatch.h"
#include "fc/runtime_config.h"
#include "io/serial.h"
#include "pg/pg.h"
#include "rx/crsf.h"
#include "scheduler/scheduler.h"
