#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx_uid.h"

PG_REGISTER_WITH_RESET_TEMPLATE(rxUidConfig_t, rxUidConfig, PG_RX_UID_CONFIG, 0);

PG_RESET_TEMPLATE(rxUidConfig_t, rxUidConfig,
    .uid = { 0 },
    .valid = 0,
);
