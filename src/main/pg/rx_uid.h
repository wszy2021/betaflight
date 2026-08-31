#pragma once

#include "pg/pg.h"
#include "rx/crsf_protocol.h"

typedef struct rxUidConfig_s {
    uint8_t uid[CRSF_RX_UID_LENGTH];
    uint8_t valid;
} rxUidConfig_t;

PG_DECLARE(rxUidConfig_t, rxUidConfig);
