#pragma once

#include "follow/follow_config.h"
#include "follow/follow_fifo.h"
#include "follow/follow_types.h"
#include "follow/follow_math.h"
#include "follow/follow_control.h"
#include "follow/follow_tracker.h"

bool followGetPidProfile(uint8_t index, followPidValues_t *out);
bool followSetPidProfile(uint8_t index, const float *values, uint8_t count);
bool followCliPrintPidProfile(uint8_t index);
bool followCliSetPidProfileString(uint8_t index, char *valueString);
void followCliDumpPidProfiles(bool bare, bool diff, bool hardwareOnly);
