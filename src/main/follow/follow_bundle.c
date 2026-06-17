#include "follow/follow_bundle.h"
#include "follow/follow_pid_defaults.h"

#ifdef USE_CLI
#include "cli/cli.h"
#include "common/typeconversion.h"

void cliPrintLinefeed(void);
void cliPrintLine(const char *str);
void cliPrintf(const char *format, ...);
#endif

followSystemData_t followSystemData = {0};
const char *followVersion = FOLLOW_VERSION;
const char *followDescription = FOLLOW_DESCRIPTION;

static followCommandHooks_t followCommandHooks = {
    .sendHeartbeat = followSendHeartbeatDefault,
    .startTrack = followStartTrackDefault,
    .stopTrack = followStopTrackDefault,
    .setExpand = followSetExpandDefault,
    .setTransfer = followSetTransferDefault,
    .moveUp = followMoveUpDefault,
    .moveDown = followMoveDownDefault,
    .moveLeft = followMoveLeftDefault,
    .moveRight = followMoveRightDefault,
    .confirmTrack = followConfirmTrackDefault,
    .switchLens = followSwitchLensDefault,
};

static followComputeTargetChannelFn followComputeTargetChannelHook = followComputeTargetChannelDefault;

#define FLASH_TAG 0x55555555

const followPidTable_t followDefaultPidTable = {
    .profile1 = FOLLOW_PID_PROFILE1_DEFAULTS,
    .profile2 = FOLLOW_PID_PROFILE2_DEFAULTS,
    .profile3 = FOLLOW_PID_PROFILE3_DEFAULTS,
    .profile4 = FOLLOW_PID_PROFILE4_DEFAULTS,
    .profile5 = FOLLOW_PID_PROFILE5_DEFAULTS,
    .profile6 = FOLLOW_PID_PROFILE6_DEFAULTS,
    .flashInitFlag = FLASH_TAG,
};

PG_RESET_TEMPLATE(followPidTable_t, dynPidGroup,
                  .profile1 = followDefaultPidTable.profile1,
                  .profile2 = followDefaultPidTable.profile2,
                  .profile3 = followDefaultPidTable.profile3,
                  .profile4 = followDefaultPidTable.profile4,
                  .profile5 = followDefaultPidTable.profile5,
                  .profile6 = followDefaultPidTable.profile6,
                  .flashInitFlag = FLASH_TAG, );

float followLimit(float input, float min, float max)
{
  if (input > max)
  {
    return max;
  }
  else if (input < min)
  {
    return min;
  }
  else
  {
    return input;
  }
}

float followAngleDiff(float a, float b)
{
  float return_angle;
  return_angle = a - b;
  if (return_angle > 180)
  {
    return_angle = return_angle - 360;
  }
  else if (return_angle < -180)
  {
    return_angle = 360 + return_angle;
  }

  return return_angle;
}

void followRotatePoint(float x, float y, float angle_radians, float *new_x, float *new_y)
{
  float cos_val = cos(angle_radians);
  float sin_val = sin(angle_radians);

  *new_x = x * cos_val - y * sin_val;
  *new_y = x * sin_val + y * cos_val;
}

uint8_t followChecksum(uint8_t *data, int32_t size)
{
  uint8_t sum = 0;
  for (int32_t i = 0; i < size; i++)
  {
    sum += data[i];
  }
  return sum;
}

void followRotateBodyToReference(const float v_body[3], const float angles[3], float v_ref[3])
{

  const float sp = sinf(-angles[1]);
  const float cp = cosf(-angles[1]);
  const float sr = sinf(-angles[0]);
  const float cr = cosf(-angles[0]);
  const float sy = sinf(-angles[2]);
  const float cy = cosf(-angles[2]);

  const float x = v_body[0];
  const float y = v_body[1];
  const float z = v_body[2];

  v_ref[0] = x * (cp * cy) + y * (sr * sp * cy - cr * sy) + z * (cr * sp * cy + sr * sy);

  v_ref[1] = x * (cp * sy) + y * (sr * sp * sy + cr * cy) + z * (cr * sp * sy - sr * cy);

  v_ref[2] = x * (-sp) + y * (sr * cp) + z * (cr * cp);
}

void followRotateReferenceToBody(const float v_ref[3], const float angles[3], float v_body[3])
{

  const float sp = sinf(-angles[1]);
  const float cp = cosf(-angles[1]);
  const float sr = sinf(-angles[0]);
  const float cr = cosf(-angles[0]);
  const float sy = sinf(-angles[2]);
  const float cy = cosf(-angles[2]);

  const float x = v_ref[0];
  const float y = v_ref[1];
  const float z = v_ref[2];

  v_body[0] = x * (cp * cy) + y * (cp * sy) + z * (-sp);

  v_body[1] = x * (sr * sp * cy - cr * sy) + y * (sr * sp * sy + cr * cy) + z * (sr * cp);

  v_body[2] = x * (cr * sp * cy + sr * sy) + y * (cr * sp * sy - sr * cy) + z * (cr * cp);
}

float followVectorDotProduct(float a[3], const float b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void followVectorCrossProduct(float res[3], float a[3], float b[3])
{
  res[0] = a[1] * b[2] - a[2] * b[1];
  res[1] = a[2] * b[0] - a[0] * b[2];
  res[2] = a[0] * b[1] - a[1] * b[0];
}

bool followVectorNormalize(float v[3])
{
  float lengthSq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  if (lengthSq < 1e-12f)
  {
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    return false;
  }
  float length = sqrtf(lengthSq);
  v[0] /= length;
  v[1] /= length;
  v[2] /= length;

  return true;
}

float followCalculateVectorAngle(const float a[3], const float b[3])
{

  float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  float cosTheta = dot;
  if (cosTheta > 1.0f)
    cosTheta = 1.0f;
  if (cosTheta < -1.0f)
    cosTheta = -1.0f;
  return acosf(cosTheta);
}

float followVectorLength(const float v[3])
{
  float lengthSq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
  if (lengthSq < 1e-12f)
  {
    return 0.0f;
  }
  return sqrtf(lengthSq);
}

void followAlphaBetaUpdate(followAlphaBetaFilter_t *filter, float measurement, float dt)
{
  float x_pred = filter->x + filter->v * dt;
  float v_pred = filter->v;

  float residual = measurement - x_pred;
  filter->x = x_pred + filter->alpha * residual;
  filter->v = v_pred + (filter->beta / dt) * residual;
}

void followUpdateFlightMode(const char *mode)
{
  if (strncmp(mode, "STAB", 4) == 0)
  {
    followSystemData.autoControlMode = FOLLOW_AUTO_CONTROL_ANGLE;
  }
  else if (strncmp(mode, "AIR", 3) == 0)
  {
    followSystemData.autoControlMode = FOLLOW_AUTO_CONTROL_ACRO;
  }
  else
  {
    followSystemData.autoControlMode = FOLLOW_AUTO_CONTROL_ACRO;
  }
}

void followUpdateAttitude(float pitch, float roll, float yaw)
{
  followSystemData.pitchDeg = -pitch;
  followSystemData.rollDeg = -roll;
  followSystemData.yawDeg = yaw;
  followSystemData.pitchRad = followSystemData.pitchDeg / 57.29578f;
  followSystemData.rollRad = followSystemData.rollDeg / 57.29578f;
  followSystemData.yawRad = followSystemData.yawDeg / 57.29578f;
  followSystemData.fpvAttitudeInfoValid = 1;
  followSystemData.attitudeUpdated = 1;
  DEBUG_SET(DEBUG_ATTITUDE, 3, followSystemData.pitchRad);
  DEBUG_SET(DEBUG_ATTITUDE, 4, followSystemData.rollRad);
  DEBUG_SET(DEBUG_ATTITUDE, 5, followSystemData.yawRad);
}

#include "arm_math.h"
#include <math.h>

volatile uint8_t followModeActive = 0;
volatile float followAdjustedChannels[4] = {0};
volatile uint8_t followRcDataUpdated = 0;
followRcChannels_t followRcChannels = {0};
followPidTable_t followPidTable = {0};

followPidValues_t followCurrentPid = {0};
PG_DECLARE(followPidTable_t, dynPidGroup);
PG_REGISTER_WITH_RESET_TEMPLATE(followPidTable_t, dynPidGroup, PG_DYN_PID_CONFIG, 1);

static followPidValues_t *followPidProfileMutableByIndex(uint8_t index)
{
  if (index >= 6)
  {
    return NULL;
  }

  return &dynPidGroupMutable()->profile1 + index;
}

static const followPidValues_t *followPidProfileByIndex(uint8_t index)
{
  if (index >= 6)
  {
    return NULL;
  }

  return &dynPidGroup()->profile1 + index;
}

static const followPidValues_t *followPidProfileCopyByIndex(uint8_t index)
{
  if (index >= 6)
  {
    return NULL;
  }

  return &dynPidGroup_Copy.profile1 + index;
}

static uint8_t followSelectedPidProfileIndex(void)
{
  if (followSystemData.autoControlMode == FOLLOW_AUTO_CONTROL_ACRO)
  {
    if (followSystemData.traceLensType == 0x00)
    {
      if (followSystemData.visibleLensType == 0x01 || followSystemData.visibleLensType == 0x02)
      {
        return 1;
      }

      return 0;
    }

    if (followSystemData.traceLensType == 0x08)
    {
      return 1;
    }

    if (followSystemData.traceLensType == 0x04)
    {
      return 2;
    }
  }
  else if (followSystemData.autoControlMode == FOLLOW_AUTO_CONTROL_ANGLE)
  {
    if (followSystemData.traceLensType == 0x00)
    {
      if (followSystemData.visibleLensType == 0x01 || followSystemData.visibleLensType == 0x02)
      {
        return 4;
      }

      return 3;
    }

    if (followSystemData.traceLensType == 0x08)
    {
      return 4;
    }

    if (followSystemData.traceLensType == 0x04)
    {
      return 5;
    }
  }

  return 0;
}

bool followGetPidProfile(uint8_t index, followPidValues_t *out)
{
  const followPidValues_t *pid = followPidProfileByIndex(index);

  if (!pid || !out)
  {
    return false;
  }

  *out = *pid;
  return true;
}

bool followSetPidProfile(uint8_t index, const float *values, uint8_t count)
{
  followPidValues_t *pid = followPidProfileMutableByIndex(index);

  if (!pid || !values)
  {
    return false;
  }

  float raw[FOLLOW_PID_VALUE_COUNT] = {0.0f};
  const uint8_t limitedCount = count > FOLLOW_PID_VALUE_COUNT ? FOLLOW_PID_VALUE_COUNT : count;
  const bool updateCurrentPid = (followSelectedPidProfileIndex() == index);
  memcpy(raw, values, limitedCount * sizeof(float));
  memcpy(pid, raw, sizeof(*pid));

  *(&followPidTable.profile1 + index) = *pid;

  if (updateCurrentPid)
  {
    followCurrentPid = *pid;
  }

  setConfigDirty();

  return true;
}

#ifdef USE_CLI
static void followCliPrintPidProfileValues(const char *prefix, uint8_t index, const followPidValues_t *pid)
{
  const float *values = (const float *)pid;
  char buf[FTOA_BUFFER_LENGTH];

  cliPrintf("%s %d", prefix, index);
  for (int i = 0; i < FOLLOW_PID_VALUE_COUNT; i++)
  {
    cliPrintf(" %s", ftoa(values[i], buf));
  }
  cliPrintLinefeed();
}

bool followCliPrintPidProfile(uint8_t index)
{
  followPidValues_t pid = {0};

  if (!followGetPidProfile(index, &pid))
  {
    return false;
  }

  followCliPrintPidProfileValues("followPID", index, &pid);
  return true;
}

bool followCliSetPidProfileString(uint8_t index, char *valueString)
{
  float values[FOLLOW_PID_VALUE_COUNT] = {0.0f};
  uint8_t valueCount = 0;
  char *saveptr = NULL;
  char *token = strtok_r(valueString, " ", &saveptr);

  while (token)
  {
    if (valueCount < FOLLOW_PID_VALUE_COUNT)
    {
      values[valueCount++] = (float)strtod(token, NULL);
    }
    token = strtok_r(NULL, " ", &saveptr);
  }

  return followSetPidProfile(index, values, valueCount);
}

void followCliDumpPidProfiles(bool bare, bool diff, bool hardwareOnly)
{
  if (hardwareOnly)
  {
    return;
  }

  if (!bare)
  {
    cliPrintLine("# follow pid");
  }

  for (uint8_t index = 0; index < 6; index++)
  {
    const followPidValues_t *pid = followPidProfileCopyByIndex(index);
    if (!pid)
    {
      continue;
    }

    followCliPrintPidProfileValues("followPID set", index, pid);
  }

  if (!diff && !bare)
  {
    cliPrintLine("followPID save");
  }
}
#else
bool followCliPrintPidProfile(uint8_t index)
{
  UNUSED(index);
  return false;
}

bool followCliSetPidProfileString(uint8_t index, char *valueString)
{
  UNUSED(index);
  UNUSED(valueString);
  return false;
}

void followCliDumpPidProfiles(bool bare, bool diff, bool hardwareOnly)
{
  UNUSED(bare);
  UNUSED(diff);
  UNUSED(hardwareOnly);
}
#endif

void followUpdateRcData(void *data, uint8_t size)
{
  (void)size;
  followCrsfChannels_t *ch = (followCrsfChannels_t *)&(((followCrsfHeader_t *)data)->data);
  followRcChannels.ch1 = ch->ch0;
  followRcChannels.ch2 = ch->ch1;
  followRcChannels.ch3 = ch->ch2;
  followRcChannels.ch4 = ch->ch3;
  followRcChannels.ch5 = ch->ch4;
  followRcChannels.ch6 = ch->ch5;
  followRcChannels.ch7 = ch->ch6;
  followRcChannels.ch8 = ch->ch7;
  followRcChannels.ch9 = ch->ch8;
  followRcChannels.ch10 = ch->ch9;
  followRcChannels.ch11 = ch->ch10;
  followRcChannels.ch12 = ch->ch11;
  followRcChannels.ch13 = ch->ch12;
  followRcChannels.ch14 = ch->ch13;
  followRcChannels.ch15 = ch->ch14;
  followRcChannels.ch16 = ch->ch15;

  followSystemData.rcChannelInfoValid = 1;
  followRcDataUpdated = 1;
}
static int followCrsfToInt(int ch)
{
  return (int)(0.62477120195241 * ch + 881);
}

void followDealWithRcValues(void)
{
  static bool ch6_toggled = false;
  static bool ch7_toggled = false;
  static bool ch8_toggled = false;
  static bool ch12_toggled = false;
  static int8_t ch9_value = 0;
  static int8_t adj_pos_on = 0;

  if (followRcDataUpdated)
  {
    followRcDataUpdated = 1;
    followRcChannels_t curr = followRcChannels;
    curr.ch6 = followCrsfToInt(curr.ch6);
    curr.ch7 = followCrsfToInt(curr.ch7);
    curr.ch8 = followCrsfToInt(curr.ch8);
    curr.ch9 = followCrsfToInt(curr.ch9);
    curr.ch1 = followCrsfToInt(curr.ch1);
    curr.ch2 = followCrsfToInt(curr.ch2);

    if (!ch6_toggled)
    {
      if (curr.ch6 > 1500)
      {
        followCommandHooks.startTrack();
        adj_pos_on = 0;
        ch6_toggled = true;

        followSystemData.mode = 1;
        followSystemData.traceState = 0;
        followStartTrackProcess();
      }
    }
    else
    {
      if (curr.ch6 < 1500)
      {
        followCommandHooks.stopTrack();
        adj_pos_on = 1;
        ch6_toggled = false;

        followSystemData.traceState = 0;
        followSystemData.mode = 0;
        followModeActive = 0;
      }
    }

    if (!ch12_toggled)
    {
      if (curr.ch12 > 1500)
      {
        followCommandHooks.setTransfer(true);
        adj_pos_on = 0;
        ch12_toggled = true;

        followSystemData.mode = 1;
        followSystemData.transferState = 1;
        followSystemData.traceState = 0;
      }
    }
    else
    {
      if (curr.ch12 < 1500)
      {
        followCommandHooks.setTransfer(false);
        followCommandHooks.stopTrack();
        adj_pos_on = 1;
        ch12_toggled = false;

        followSystemData.transferState = 0;
        followSystemData.traceState = 0;
        followSystemData.mode = 0;
        followModeActive = 0;
      }
    }

    if (1 != followSystemData.mode)
    {
      if (!ch7_toggled)
      {
        if (curr.ch7 > 1500)
        {
          followCommandHooks.setExpand(true);
          adj_pos_on = 1;
          ch7_toggled = true;
        }
      }
      else
      {
        if (curr.ch7 < 1500)
        {
          followCommandHooks.setExpand(false);
          adj_pos_on = 0;
          ch7_toggled = false;
        }
      }
    }
    else if (1 == followSystemData.mode)
    {
      if (!ch8_toggled)
      {
        if (curr.ch8 > 1500)
        {
          adj_pos_on = 1;
          ch8_toggled = true;
        }
      }
      else
      {
        if (curr.ch8 < 1500)
        {
          followCommandHooks.confirmTrack();
          adj_pos_on = 0;
          ch8_toggled = false;

          followSystemData.startYawDeg = followSystemData.yawDeg;
        }
      }
    }

    static int8_t adj_tick = 0;
    adj_tick++;
    if (1 == adj_pos_on)
    {
      if (curr.ch1 > 1500)
      {
        followCommandHooks.moveRight();
      }
      else if (curr.ch1 < 1000)
      {
        followCommandHooks.moveLeft();
      }
      if (curr.ch2 > 1500)
      {
        followCommandHooks.moveUp();
      }
      else if (curr.ch2 < 1000)
      {
        followCommandHooks.moveDown();
      }
    }

    if (curr.ch9 > 1800)
    {
      if (2 != ch9_value)
      {
        followCommandHooks.switchLens(2);
        ch9_value = 2;
      }
    }
    else if (curr.ch9 > 1000)
    {
      if (1 != ch9_value)
      {
        followCommandHooks.switchLens(1);
        ch9_value = 1;
      }
    }
    else
    {
      if (0 != ch9_value)
      {
        followCommandHooks.switchLens(0);
        ch9_value = 0;
      }
    }
  }
}

void followStartTrackProcess(void)
{
  const followPidTable_t *_ = dynPidGroup();
  memcpy((uint8_t *)&followPidTable, (uint8_t *)_, sizeof(followPidTable_t));

  followSystemData.traceState = 0;

  followSystemData.trackErrorX = 0;
  followSystemData.trackErrorY = 0;
  followSystemData.lastTrackErrorX = 0;
  followSystemData.lastTrackErrorY = 0;

  if (followSystemData.pitchDeg < 35)
  {
    followSystemData.trackPitchDeg = 35;
  }
  else if (followSystemData.pitchDeg > 60)
  {
    followSystemData.trackPitchDeg = 60;
  }
  else
  {
    followSystemData.trackPitchDeg = followSystemData.pitchDeg;
  }

  followSystemData.initialAccChannel = followRcChannels.ch3;
  followSystemData.targetAccChannel = followSystemData.initialAccChannel;

  followSystemData.targetPitchChannel = followRcChannels.ch2;
  followSystemData.targetRollChannel = followRcChannels.ch1;
  followSystemData.targetYawChannel = followRcChannels.ch4;

  followSystemData.startYawDeg = followSystemData.yawDeg;

  if (followSystemData.autoControlMode == FOLLOW_AUTO_CONTROL_ACRO)
  {
    followSystemData.startRollDeg = followSystemData.rollDeg;
  }

  followSystemData.calRollLockTime = 100;
  followSystemData.diffYawError = 0;
  followSystemData.yawError = 0;
  followSystemData.lastYawError = 0;

  followSystemData.pitchError = 0;
  followSystemData.lastPitchError = 0;

  followSystemData.diffRollError = 0;
  followSystemData.rollError = 0;
  followSystemData.lastRollError = 0;

  followSystemData.previousAutoControlMode = 0xff;
  followSystemData.previousTraceLensType = 0xff;

  followSystemData.inSwitchState = 0;

  followSystemData.firstView = 1;

  followSystemData.trackAttenuationXY = 0.0f;

  followSystemData.reduceTrackPitch = false;
  followSystemData.minTrackPitchDeg = followPidTable.profile6.pitchMinOutput;

  if ((ARMING_FLAG(ARMED)) && (followRcChannels.ch3 > RC_RANGE_TH_MIN))
  {
    followSystemData.controlLaunchStatus = FOLLOW_LAUNCH_STATUS_GUIDANCE;
  }
  else
  {
    followSystemData.controlLaunchStatus = FOLLOW_LAUNCH_STATUS_GUIDANCE;
  }

  followResetPitOffRateWindow();

  followSelectPidProfile();
}

void followCalcAttitude(void)
{
  if (0 == followSystemData.mode || 1 != followSystemData.traceState)
  {

    return;
  }
  if (followSystemData.transferState == 1)
  {
    followStartTrackProcess();
    followSystemData.transferState = 0;
  }

  if (followSystemData.traceUpdated)
  {
    followSystemData.traceUpdated = 0;

    followHandleGuidance();
  }

  followAdjustedChannels[0] = followSystemData.targetRollChannel;
  followAdjustedChannels[1] = followSystemData.targetPitchChannel;
  followAdjustedChannels[2] = followSystemData.targetYawChannel;
  followAdjustedChannels[3] = followSystemData.targetAccChannel;
}

void followHandleGuidance(void)
{

  static float output_acc;
  float temp_vec_test[3] = {0, 0, 1};
  float temp_vec_test1[3];
  float th_cos_res;
  float temp_eulerang[3] = {followSystemData.rollRad, followSystemData.pitchRad, 0.0f};
  float output_acc_1;
  switch (followSystemData.controlLaunchStatus)
  {
  case FOLLOW_LAUNCH_STATUS_PRE_ARMED:
    followSystemData.targetRollChannel = RC_RANGE_TERM;
    followSystemData.targetPitchChannel = RC_RANGE_TERM;
    followSystemData.targetYawChannel = RC_RANGE_TERM;
    followSystemData.targetAccChannel = RC_RANGE_MIN;
    if (ARMING_FLAG(ARMED))
    {
      followSystemData.controlLaunchStatus = FOLLOW_LAUNCH_STATUS_ARMED_PRE_LAUNCH;
      followSystemData.controlLaunchTimeMs = millis();
      followSystemData.targetRollChannel = RC_RANGE_TERM;
      followSystemData.targetPitchChannel = RC_RANGE_TERM;
      followSystemData.targetYawChannel = RC_RANGE_TERM;
      followSystemData.targetAccChannel = RC_RANGE_MIN;
    }
    break;
  case FOLLOW_LAUNCH_STATUS_ARMED_PRE_LAUNCH:

    if (millis() - followSystemData.controlLaunchTimeMs > 1000)
    {

      followRotateReferenceToBody(temp_vec_test, temp_eulerang, temp_vec_test1);
      th_cos_res = followVectorDotProduct(temp_vec_test, temp_vec_test1);
      th_cos_res = followLimit(th_cos_res, 0.3f, 1.0f);

      followSystemData.controlLaunchStatus = FOLLOW_LAUNCH_STATUS_TAKEOFF;
      followSystemData.targetRollChannel = RC_RANGE_TERM;
      followSystemData.targetPitchChannel = RC_RANGE_TERM;
      followSystemData.targetYawChannel = RC_RANGE_TERM;
      output_acc = followPidTable.profile6.accFilterAlpha / th_cos_res;
      output_acc_1 = followLimit(output_acc / th_cos_res, followCurrentPid.accMinOutput, followCurrentPid.accMaxOutput);
      followSystemData.targetAccChannel = RC_RANGE_TH_MIN + (RC_RANGE_TH_WIDE * output_acc_1);
    }
    break;
  case FOLLOW_LAUNCH_STATUS_TAKEOFF:

    if (millis() - followSystemData.controlLaunchTimeMs > 1500)
    {
      followSystemData.controlLaunchStatus = FOLLOW_LAUNCH_STATUS_GUIDANCE;
    }
    break;
  case FOLLOW_LAUNCH_STATUS_GUIDANCE:
    followComputeTargetChannel(0, 0);
    break;
  }
}

int8_t followAdjustCrsfDataIfNecessary(void)
{
  uint32_t *ptr = followGetCrsfChannelData();
  if (ptr && 1 == followModeActive)
  {
    ptr[0] = (uint32_t)followAdjustedChannels[0];
    ptr[1] = (uint32_t)followAdjustedChannels[1];
    ptr[3] = (uint32_t)followAdjustedChannels[2];
    ptr[2] = (uint32_t)followAdjustedChannels[3];
    return 1;
  }
  return 0;
}

void followSetCommandHooks(const followCommandHooks_t *hooks)
{
  followCommandHooks.sendHeartbeat = hooks && hooks->sendHeartbeat ? hooks->sendHeartbeat : followSendHeartbeatDefault;
  followCommandHooks.startTrack = hooks && hooks->startTrack ? hooks->startTrack : followStartTrackDefault;
  followCommandHooks.stopTrack = hooks && hooks->stopTrack ? hooks->stopTrack : followStopTrackDefault;
  followCommandHooks.setExpand = hooks && hooks->setExpand ? hooks->setExpand : followSetExpandDefault;
  followCommandHooks.setTransfer = hooks && hooks->setTransfer ? hooks->setTransfer : followSetTransferDefault;
  followCommandHooks.moveUp = hooks && hooks->moveUp ? hooks->moveUp : followMoveUpDefault;
  followCommandHooks.moveDown = hooks && hooks->moveDown ? hooks->moveDown : followMoveDownDefault;
  followCommandHooks.moveLeft = hooks && hooks->moveLeft ? hooks->moveLeft : followMoveLeftDefault;
  followCommandHooks.moveRight = hooks && hooks->moveRight ? hooks->moveRight : followMoveRightDefault;
  followCommandHooks.confirmTrack = hooks && hooks->confirmTrack ? hooks->confirmTrack : followConfirmTrackDefault;
  followCommandHooks.switchLens = hooks && hooks->switchLens ? hooks->switchLens : followSwitchLensDefault;
}

const followCommandHooks_t *followGetCommandHooks(void)
{
  return &followCommandHooks;
}

void followResetCommandHooks(void)
{
  followSetCommandHooks(NULL);
}

void followSetComputeTargetChannelHook(followComputeTargetChannelFn fn)
{
  followComputeTargetChannelHook = fn ? fn : followComputeTargetChannelDefault;
}

followComputeTargetChannelFn followGetComputeTargetChannelHook(void)
{
  return followComputeTargetChannelHook;
}

void followResetComputeTargetChannelHook(void)
{
  followComputeTargetChannelHook = followComputeTargetChannelDefault;
}

void followComputeTargetChannel(int x_offset, int y_offset)
{
  followComputeTargetChannelHook(x_offset, y_offset);
}

extern int writeReadEeprom(dispatchEntry_t *self);
static void followSaveConfigToFlash(void)
{
  if (ARMING_FLAG(ARMED))
  {
    return;
  }

  schedulerIgnoreTaskStateTime();

  {
    writeReadEeprom(NULL);
  }
}

void followParseSimData(uint8_t *data, uint32_t size)
{
  switch (data[4])
  {
  case 0x00:
  case 0x01:
    break;
  case 0x02:
  case 0x04:
  case 0x06:
  case 0x08:
  case 0x0a:
  case 0x0d:
    followReceivePidGroup(data[4], data, size);
    break;
  case 0x03:
  case 0x05:
  case 0x07:
  case 0x09:
  case 0x0b:
  case 0x0e:
    followSendPidGroup(data[4], data, size);
    break;
  case 0x0c:
    followSaveConfigToFlash();
    break;
  }
}

void followReceivePidGroup(uint8_t type, uint8_t *data, uint32_t size)
{
  (void)size;
  uint8_t idx = 0, wr = 0;
  if (0x02 == type)
  {
    idx = 0;
    wr = 0x82;
  }
  else if (0x04 == type)
  {
    idx = 1;
    wr = 0x84;
  }
  else if (0x06 == type)
  {
    idx = 2;
    wr = 0x86;
  }
  else if (0x08 == type)
  {
    idx = 3;
    wr = 0x88;
  }
  else if (0x0a == type)
  {
    idx = 4;
    wr = 0x8a;
  }
  else if (0x0d == type)
  {
    idx = 5;
    wr = 0x8d;
  }
  else
  {
    return;
  }
  uint8_t ret[] = {0xaa, 0x55, 0x01, 0x00, wr, wr, 0xa5, 0x5a};
  serialWriteBuf(followSimPort, ret, sizeof(ret));
  followPidValues_t *pid = &dynPidGroupMutable()->profile1 + idx;
  memcpy((uint8_t *)pid, data + 5, sizeof(followPidValues_t));
  pid = &followPidTable.profile1 + idx;
  memcpy((uint8_t *)pid, data + 5, sizeof(followPidValues_t));
}

void followSendPidGroup(uint8_t type, uint8_t *data, uint32_t size)
{
  (void)size;
  (void)data;
  uint8_t idx = 0, rd = 0;
  if (0x03 == type)
  {
    idx = 0;
  }
  else if (0x05 == type)
  {
    idx = 1;
  }
  else if (0x07 == type)
  {
    idx = 2;
  }
  else if (0x09 == type)
  {
    idx = 3;
  }
  else if (0x0b == type)
  {
    idx = 4;
  }
  else if (0x0e == type)
  {
    idx = 5;
  }
  else
  {
    return;
  }
  rd = (0x80 | type);
  const followPidValues_t *pid = &dynPidGroup()->profile1 + idx;
  uint8_t ret[120] = {0xaa, 0x55, 0x71, 0x00, rd};
  memcpy(ret + 5, (uint8_t *)pid, sizeof(followPidValues_t));
  ret[119] = 0x5a;
  ret[118] = 0xa5;
  ret[117] = followChecksum(ret + 4, 113);
  serialWriteBuf(followSimPort, ret, sizeof(ret));
}

void followSelectPidProfile(void)
{
  followPidValues_t *temp_pid = NULL;

  if ((followSystemData.previousAutoControlMode == followSystemData.autoControlMode) &&
      (followSystemData.previousTraceLensType == followSystemData.traceLensType))
  {
    return;
  }

  followSystemData.previousAutoControlMode = followSystemData.autoControlMode;
  followSystemData.previousTraceLensType = followSystemData.traceLensType;

  followSystemData.inSwitchState = true;

  if (followSystemData.autoControlMode == FOLLOW_AUTO_CONTROL_ACRO)
  {
    if (followSystemData.traceLensType == 0x00)
    {
      if (followSystemData.visibleLensType == 0x01 || followSystemData.visibleLensType == 0x02)
      {
        temp_pid = &(followPidTable.profile2);
        followSystemData.pixelFocalLength = 2375;
      }
      else
      {
        temp_pid = &(followPidTable.profile1);
        followSystemData.pixelFocalLength = 2083.33;
      }
    }
    else if (followSystemData.traceLensType == 0x08)
    {
      temp_pid = &(followPidTable.profile2);
      followSystemData.pixelFocalLength = 2375;
    }
    else if (followSystemData.traceLensType == 0x04)
    {
      temp_pid = &(followPidTable.profile3);
    }
  }
  else if (followSystemData.autoControlMode == FOLLOW_AUTO_CONTROL_ANGLE)
  {
    if (followSystemData.traceLensType == 0x00)
    {
      if (followSystemData.visibleLensType == 0x01 || followSystemData.visibleLensType == 0x02)
      {
        temp_pid = &(followPidTable.profile5);
      }
      else
      {
        temp_pid = &(followPidTable.profile4);
      }
    }
    else if (followSystemData.traceLensType == 0x08)
    {
      temp_pid = &(followPidTable.profile5);
    }
    else if (followSystemData.traceLensType == 0x04)
    {
      temp_pid = &(followPidTable.profile6);
    }
  }

  if (temp_pid == NULL)
  {
    temp_pid = &(followPidTable.profile1);
    followSystemData.pixelFocalLength = 4071;
  }

  *(&followCurrentPid) = *temp_pid;
}

serialPort_t *followWorkPort = NULL;
serialPort_t *followSimPort = NULL;
kfifo_t _serialInput = {0};
kfifo_t _simInput = {0};

extern void followOverrideRegisterHooks(void) __attribute__((weak));

static bool followProtectedHooksRegistered = false;

static void followMaybeRegisterProtectedHooks(void)
{
  if (followProtectedHooksRegistered)
  {
    return;
  }

  if (followOverrideRegisterHooks)
  {
    followOverrideRegisterHooks();
  }

  followProtectedHooksRegistered = true;
}

bool followTrackerInit(void)
{
  followMaybeRegisterProtectedHooks();

  const uint32_t kfifo_size = 1024;
  uint8_t *buf = (uint8_t *)malloc(kfifo_size);
  kfifo_init(&_serialInput, buf, kfifo_size);

  buf = (uint8_t *)malloc(kfifo_size * 2);
  kfifo_init(&_simInput, buf, kfifo_size * 2);

  int8_t serial_mask = 0;
  if (!followWorkPort)
  {
    followWorkPort = openSerialPort(FOLLOW_WORK_SERIAL_PORT, FUNCTION_NONE, NULL, NULL,
                                    115200, MODE_RXTX, SERIAL_NOT_INVERTED);
  }
  if (!followSimPort)
  {
    followSimPort = openSerialPort(FOLLOW_SIM_SERIAL_PORT, FUNCTION_NONE, NULL, NULL,
                                   115200, MODE_RXTX, SERIAL_NOT_INVERTED);
  }
  serial_mask |= (followWorkPort ? 1 : 0);
  serial_mask |= (followSimPort ? 2 : 0);

  followPrintDebug("followTrackerInit\n");

  return true;
}

void followTrackerTask(timeUs_t currentTimeUs)
{
  UNUSED(currentTimeUs);

  uint8_t rx_buf[128] = {0};
  uint32_t rx_len = 0;
  if (followWorkPort)
  {
    while (serialRxBytesWaiting(followWorkPort) > 0)
    {
      rx_buf[rx_len++] = serialRead(followWorkPort);
    }
    if (rx_len > 0)
    {
      kfifo_in(&_serialInput, rx_buf, rx_len);
      followAnalyzeSerialInput();
    }
  }
  if (followSimPort)
  {
    rx_len = 0;
    while (serialRxBytesWaiting(followSimPort) > 0)
    {
      rx_buf[rx_len++] = serialRead(followSimPort);
    }
    if (rx_len > 0)
    {
      kfifo_in(&_simInput, rx_buf, rx_len);

      followAnalyzeSimInput();
    }
  }
}

void followTimerCallback(timerOvrHandlerRec_t *cbRec, captureCompare_t capture)
{
  UNUSED(cbRec);
  UNUSED(capture);

  followDealWithRcValues();
  followCalcAttitude();

  static uint32_t tick = 0;
  tick++;
  if (tick % 10 == 0)
  {
    followCommandHooks.sendHeartbeat();
  }
}

void followTimerTask(timeUs_t currentTimeUs)
{
  UNUSED(currentTimeUs);

  followDealWithRcValues();
  followCalcAttitude();

  static uint32_t tick = 0;
  tick++;
  if (tick % 5 == 0)
  {
    followCommandHooks.sendHeartbeat();
  }
}

void followAnalyzeSerialInput(void)
{
  uint8_t frame[32] = {0};
  uint32_t bytes = 0, max_frm_size = 13;
  while ((bytes = kfifo_len(&_serialInput)) > 0)
  {
    if (bytes < max_frm_size)
    {
      break;
    }
    if (max_frm_size != kfifo_peek(&_serialInput, frame, max_frm_size))
    {
      break;
    }
    if (0x91 != frame[0] || 0x0d != frame[1])
    {
      kfifo_skip(&_serialInput, 1);
      continue;
    }
    uint8_t sum = followChecksum(frame, max_frm_size - 1);
    if (sum != frame[max_frm_size - 1])
    {
      kfifo_skip(&_serialInput, 1);
      continue;
    }
    if (0x3f == frame[2])
    {
      followSystemData.traceLensType = ((frame[3] & 0xf0) >> 4);

      followTraceInfo_t *curr = (followTraceInfo_t *)(frame + 3);
      if (0x01 == (frame[3] & 0x03))
      {

        if (followSystemData.traceLensType == 0x04)
        {
          switch (followSystemData.infraredLensType)
          {
          case 1:
          case 2:
            followSystemData.trackErrorX = curr->coordinateX - 325;
            followSystemData.trackErrorY = curr->coordinateY - 256;
            followSystemData.cameraResolutionX = 650;
            followSystemData.cameraResolutionY = 512;
            break;
          case 3:
            followSystemData.trackErrorX = curr->coordinateX - 192;
            followSystemData.trackErrorY = curr->coordinateY - 144;
            followSystemData.cameraResolutionX = 384;
            followSystemData.cameraResolutionY = 288;
            break;
          default:
            break;
          }
        }
        else
        {
          followSystemData.trackErrorX = curr->coordinateX - 960;
          followSystemData.trackErrorY = curr->coordinateY - 540;
          followSystemData.cameraResolutionX = 1920;
          followSystemData.cameraResolutionY = 1080;
        }

        followSystemData.traceState = 1;
        followSystemData.traceUpdated = 1;
      }
      else
      {
        followSystemData.traceState = 2;
      }

      if ((curr->traceStatus & 0x04) == 0)
      {
        followSystemData.traceStage = 0;
      }
      else
      {
        followSystemData.traceStage = 1;
      }

      if (1 == followSystemData.mode && 1 == followSystemData.traceState)
      {
        followModeActive = 1;
      }
    }
    if (0x40 == frame[2])
    {

      followSystemData.deviceCpuType = frame[3];
      followSystemData.visibleLensType = frame[4];
      followSystemData.infraredLensType = frame[5];
      frame[0] = 0x90;
      frame[1] = 0xeb;
      frame[2] = 0x0e;
    }
    kfifo_skip(&_serialInput, max_frm_size);
    followSelectPidProfile();
  }
}

void followAnalyzeSimInput(void)
{
  uint8_t frame[256] = {0};
  uint32_t bytes = 0, frm_size = 0;
  while ((bytes = kfifo_len(&_simInput)) > 0)
  {
    if (bytes < 8)
    {
      break;
    }
    bytes = kfifo_peek(&_simInput, frame, 8);
    if (0xaa != frame[0] || 0x55 != frame[1])
    {
      kfifo_skip(&_simInput, 1);
      continue;
    }
    frm_size = frame[2] + (frame[3] << 8);
    if (frm_size > 256 || frm_size < 1)
    {
      kfifo_skip(&_simInput, 1);
      continue;
    }
    uint32_t real_size = frm_size + 7;
    if (kfifo_peek(&_simInput, frame, real_size) != real_size)
    {
      break;
    }
    if (0xa5 != frame[real_size - 2] || 0x5a != frame[real_size - 1])
    {
      kfifo_skip(&_simInput, 1);
      continue;
    }
    uint8_t sum = followChecksum(frame + 4, frm_size);
    if (sum != frame[frm_size + 4])
    {
      kfifo_skip(&_simInput, 1);
      continue;
    }
    followParseSimData(frame, real_size);
    static int32_t valid_frm_cnt = 0;
    valid_frm_cnt++;
    kfifo_skip(&_simInput, real_size);
  }
}

static void putcp(void *p, char c)
{
  *(*((char **)p))++ = c;
}
void followPrintDebug(const char *msg, ...)
{
#ifdef PG_DYN_PID_CONFIG

#endif
  if (!followSimPort)
  {
    return;
  }
  char output[256] = {0};
  va_list args;
  va_start(args, msg);
  char *ptr = output;
  tfp_format(&ptr, putcp, msg, args);
  *ptr = '\0';
  va_end(args);
  serialWriteBuf(followSimPort, (uint8_t *)output, strlen(output));
}
