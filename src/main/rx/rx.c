/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/tasks.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/pwm.h"
#include "rx/fport.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/srxl2.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/crsf.h"
#include "rx/ghst.h"
#include "rx/rx_spi.h"
#include "rx/targetcustomserial.h"
#include "rx/msp_override.h"


const char rcChannelLetters[] = "AERT12345678abcdefgh";

static uint16_t rssi = 0;                  // range: [0;1023]
static uint16_t rssiRaw = 0;               // range: [0;1023]
static timeUs_t lastRssiSmoothingUs = 0;
#ifdef USE_RX_RSSI_DBM
static int8_t activeAntenna;
static int16_t rssiDbm = CRSF_RSSI_MIN;    // range: [-130,0]
static int16_t rssiDbmRaw = CRSF_RSSI_MIN; // range: [-130,0]
#endif //USE_RX_RSSI_DBM
#ifdef USE_RX_RSNR
static int16_t rsnr = CRSF_SNR_MIN;        // range: [-30,20]
static int16_t rsnrRaw = CRSF_SNR_MIN;     // range: [-30,20]
#endif //USE_RX_RSNR
static timeUs_t lastMspRssiUpdateUs = 0;

static pt1Filter_t frameErrFilter;
static pt1Filter_t rssiFilter;
#ifdef USE_RX_RSSI_DBM
static pt1Filter_t rssiDbmFilter;
#endif //USE_RX_RSSI_DBM
#ifdef USE_RX_RSNR
static pt1Filter_t rsnrFilter;
#endif //USE_RX_RSNR

#ifdef USE_RX_LINK_QUALITY_INFO
static uint16_t linkQuality = 0;
static uint8_t rfMode = 0;
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
static uint16_t uplinkTxPwrMw = 0;  //Uplink Tx power in mW
#endif

#define RSSI_ADC_DIVISOR (4096 / 1024)
#define RSSI_OFFSET_SCALING (1024 / 100.0f)

rssiSource_e rssiSource;
linkQualitySource_e linkQualitySource;

static bool rxDataProcessingRequired = false;
static bool auxiliaryProcessingRequired = false;

static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;
static uint8_t rxChannelCount;

static timeUs_t needRxSignalBefore = 0;
static timeUs_t suspendRxSignalUntil = 0;
static uint8_t  skipRxSamples = 0;

static float rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // last received raw value, as it comes
float rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];           // scaled, modified, checked and constrained values
uint32_t validRxSignalTimeout[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define MAX_INVALID_PULSE_TIME_MS 300                   // hold time in milliseconds after bad channel or Rx link loss
// will not be actioned until the nearest multiple of 100ms
#define PPM_AND_PWM_SAMPLE_COUNT 3

#define DELAY_20_MS (20 * 1000)                         // 20ms in us
#define DELAY_100_MS (100 * 1000)                       // 100ms in us
#define DELAY_1500_MS (1500 * 1000)                     // 1.5 seconds in us
#define SKIP_RC_SAMPLES_ON_RESUME  2                    // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeState_t rxRuntimeState;
static uint8_t rcSampleIndex = 0;

// ============================================================
// 双接收机全局变量
// ============================================================
 
rxMultiInstance_t rxMultiInstance = {
    .activeInstanceCount = 0,
    .primaryInstance = RX_INSTANCE_PRIMARY,
    .auxiliaryInstance = RX_INSTANCE_AUXILIARY,
    .auxiliaryRssiEnabled = false,
    .initialized = false,
};
 
auxiliaryRxRssi_t auxRssiData[MAX_AUX_RSSI_RECEIVERS];
 
bool rxDualModeEnabled = false;
 
// ============================================================

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfigs[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
    }
}

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfig->min = PWM_RANGE_MIN;
        rxChannelRangeConfig->max = PWM_RANGE_MAX;
        rxChannelRangeConfig++;
    }
}

static float nullReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return RX_FRAME_PENDING;
}

static bool nullProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return true;
}

STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIALRX
static bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    bool enabled = false;
    switch (rxRuntimeState->serialrxProvider) {
#ifdef USE_SERIALRX_SRXL2
    case SERIALRX_SRXL2:
        enabled = srxl2RxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SRXL:
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_GHST
    case SERIALRX_GHST:
        enabled = ghstRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_TARGET_CUSTOM
    case SERIALRX_TARGET_CUSTOM:
        enabled = targetCustomSerialRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_FPORT
    case SERIALRX_FPORT:
        enabled = fportRxInit(rxConfig, rxRuntimeState);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

void rxInit(void)
{
    // 首先初始化双接收机系统
    rxMultiInstanceInit(rxConfig());
    
    // 如果双接收机模式未启用，使用传统单接收机初始化
    if (!rxDualModeEnabled) {
        // 传统单接收机初始化逻辑
        if (featureIsEnabled(FEATURE_RX_PARALLEL_PWM)) {
            rxRuntimeState.rxProvider = RX_PROVIDER_PARALLEL_PWM;
        } else if (featureIsEnabled(FEATURE_RX_PPM)) {
            rxRuntimeState.rxProvider = RX_PROVIDER_PPM;
        } else if (featureIsEnabled(FEATURE_RX_SERIAL)) {
            rxRuntimeState.rxProvider = RX_PROVIDER_SERIAL;
        } else if (featureIsEnabled(FEATURE_RX_MSP)) {
            rxRuntimeState.rxProvider = RX_PROVIDER_MSP;
        } else if (featureIsEnabled(FEATURE_RX_SPI)) {
            rxRuntimeState.rxProvider = RX_PROVIDER_SPI;
        } else {
            rxRuntimeState.rxProvider = RX_PROVIDER_NONE;
        }
        rxRuntimeState.serialrxProvider = rxConfig()->serialrx_provider;
        rxRuntimeState.rcReadRawFn = nullReadRawRC;
        rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
        rxRuntimeState.rcProcessFrameFn = nullProcessFrame;
        rxRuntimeState.lastRcFrameTimeUs = 0;
        rcSampleIndex = 0;
 
        uint32_t now = millis();
        for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
            rcData[i] = rxConfig()->midrc;
            validRxSignalTimeout[i] = now + MAX_INVALID_PULSE_TIME_MS;
        }
 
        rcData[THROTTLE] = (featureIsEnabled(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;
 
        // Initialize ARM switch to OFF position
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(i);
            if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
                float value;
                if (modeActivationCondition->range.startStep > 0) {
                    value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
                } else {
                    value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
                }
                rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
            }
        }
 
        switch (rxRuntimeState.rxProvider) {
        default:
            break;
#ifdef USE_SERIALRX
        case RX_PROVIDER_SERIAL:
            {
                const bool enabled = serialRxInit(rxConfig(), &rxRuntimeState);
                if (!enabled) {
                    rxRuntimeState.rcReadRawFn = nullReadRawRC;
                    rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
                }
            }
            break;
#endif
 
#ifdef USE_RX_MSP
        case RX_PROVIDER_MSP:
            rxMspInit(rxConfig(), &rxRuntimeState);
            break;
#endif
 
#ifdef USE_RX_SPI
        case RX_PROVIDER_SPI:
            {
                const bool enabled = rxSpiInit(rxSpiConfig(), &rxRuntimeState);
                if (!enabled) {
                    rxRuntimeState.rcReadRawFn = nullReadRawRC;
                    rxRuntimeState.rcFrameStatusFn = nullFrameStatus;
                }
            }
            break;
#endif
 
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
        case RX_PROVIDER_PPM:
        case RX_PROVIDER_PARALLEL_PWM:
            rxPwmInit(rxConfig(), &rxRuntimeState);
            break;
#endif
        }
 
#if defined(USE_ADC)
        if (featureIsEnabled(FEATURE_RSSI_ADC)) {
            rssiSource = RSSI_SOURCE_ADC;
        } else
#endif
        if (rxConfig()->rssi_channel > 0) {
            rssiSource = RSSI_SOURCE_RX_CHANNEL;
        }
 
        pt1FilterInit(&frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig()->rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US * 1e-6f));
 
        float k = (256.0f - rxConfig()->rssi_smoothing) / 256.0f;
        pt1FilterInit(&rssiFilter, k);
 
#ifdef USE_RX_RSSI_DBM
        pt1FilterInit(&rssiDbmFilter, k);
#endif
 
#ifdef USE_RX_RSNR
        pt1FilterInit(&rsnrFilter, k);
#endif
 
        rxChannelCount = MIN(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT, rxRuntimeState.channelCount);
    } else {
        // 双接收机模式：初始化已由 rxMultiInstanceInit 完成
        // 只需要初始化过滤器和通道数据
        uint32_t now = millis();
        for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
            rcData[i] = rxConfig()->midrc;
            validRxSignalTimeout[i] = now + MAX_INVALID_PULSE_TIME_MS;
        }
        rcData[THROTTLE] = (featureIsEnabled(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;
 
        // 初始化过滤器
        pt1FilterInit(&frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig()->rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US * 1e-6f));
        float k = (256.0f - rxConfig()->rssi_smoothing) / 256.0f;
        pt1FilterInit(&rssiFilter, k);
#ifdef USE_RX_RSSI_DBM
        pt1FilterInit(&rssiDbmFilter, k);
#endif
#ifdef USE_RX_RSNR
        pt1FilterInit(&rsnrFilter, k);
#endif
        
        rxRuntimeState_t *primaryRx = &rxMultiInstance.rxRuntimeState[rxMultiInstance.primaryInstance];
        rxChannelCount = MIN(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT, primaryRx->channelCount);
    }
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

void suspendRxSignal(void)
{
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros() + DELAY_1500_MS;  // 1.5s
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    }
#endif
    failsafeOnRxSuspend(DELAY_1500_MS);  // 1.5s
}

void resumeRxSignal(void)
{
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros();
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    }
#endif
    failsafeOnRxResume();
}

#ifdef USE_RX_LINK_QUALITY_INFO
#define LINK_QUALITY_SAMPLE_COUNT 16

STATIC_UNIT_TESTED uint16_t updateLinkQualitySamples(uint16_t value)
{
    static uint16_t samples[LINK_QUALITY_SAMPLE_COUNT];
    static uint8_t sampleIndex = 0;
    static uint16_t sum = 0;

    sum += value - samples[sampleIndex];
    samples[sampleIndex] = value;
    sampleIndex = (sampleIndex + 1) % LINK_QUALITY_SAMPLE_COUNT;
    return sum / LINK_QUALITY_SAMPLE_COUNT;
}

void rxSetRfMode(uint8_t rfModeValue)
{
    rfMode = rfModeValue;
}
#endif

static void setLinkQuality(bool validFrame, timeDelta_t currentDeltaTimeUs)
{
    static uint16_t rssiSum = 0;
    static uint16_t rssiCount = 0;
    static timeDelta_t resampleTimeUs = 0;

#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_NONE) {
        // calculate new sample mean
        linkQuality = updateLinkQualitySamples(validFrame ? LINK_QUALITY_MAX_VALUE : 0);
    }
#endif

    if (rssiSource == RSSI_SOURCE_FRAME_ERRORS) {
        resampleTimeUs += currentDeltaTimeUs;
        rssiSum += validFrame ? RSSI_MAX_VALUE : 0;
        rssiCount++;

        if (resampleTimeUs >= FRAME_ERR_RESAMPLE_US) {
            setRssi(rssiSum / rssiCount, rssiSource);
            rssiSum = 0;
            rssiCount = 0;
            resampleTimeUs -= FRAME_ERR_RESAMPLE_US;
        }
    }
}

void setLinkQualityDirect(uint16_t linkqualityValue)
{
#ifdef USE_RX_LINK_QUALITY_INFO
    linkQuality = linkqualityValue;
#else
    UNUSED(linkqualityValue);
#endif
}

#ifdef USE_RX_LINK_UPLINK_POWER
void rxSetUplinkTxPwrMw(uint16_t uplinkTxPwrMwValue)
{
    uplinkTxPwrMw = uplinkTxPwrMwValue;
}
#endif

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    return taskUpdateRxMainInProgress() || rxDataProcessingRequired || auxiliaryProcessingRequired;
}

FAST_CODE_NOINLINE void rxFrameCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    bool signalReceived = false;
    bool useDataDrivenProcessing = true;
    timeDelta_t needRxSignalMaxDelayUs = DELAY_100_MS;

    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 2, MIN(2000, currentDeltaTimeUs / 100));

    if (taskUpdateRxMainInProgress()) {
        //  no need to check for new data as a packet is being processed already
        return;
    }

    switch (rxRuntimeState.rxProvider) {
    default:

        break;
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    case RX_PROVIDER_PPM:
        if (isPPMDataBeingReceived()) {
            signalReceived = true;
            resetPPMDataReceivedState();
        }

        break;
    case RX_PROVIDER_PARALLEL_PWM:
        if (isPWMDataBeingReceived()) {
            signalReceived = true;
            useDataDrivenProcessing = false;
        }

        break;
#endif
    case RX_PROVIDER_SERIAL:
    case RX_PROVIDER_MSP:
    case RX_PROVIDER_SPI:
    case RX_PROVIDER_UDP:
        {
            const uint8_t frameStatus = rxRuntimeState.rcFrameStatusFn(&rxRuntimeState);
            DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 1, (frameStatus & RX_FRAME_FAILSAFE));
            signalReceived = (frameStatus & RX_FRAME_COMPLETE) && !(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED));
            setLinkQuality(signalReceived, currentDeltaTimeUs);
            auxiliaryProcessingRequired |= (frameStatus & RX_FRAME_PROCESSING_REQUIRED);
        }

        break;
    }

    if (signalReceived) {
        //  true only when a new packet arrives
        needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        rxSignalReceived = true; // immediately process packet data
        if (useDataDrivenProcessing) {
            rxDataProcessingRequired = true;
            //  process the new Rx packet when it arrives
        }
    } else {
        //  watch for next packet
        if (cmpTimeUs(currentTimeUs, needRxSignalBefore) > 0) {
            //  initial time to signalReceived failure is 100ms, then we check every 100ms
            rxSignalReceived = false;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            //  review and process rcData values every 100ms in case failsafe changed them
            rxDataProcessingRequired = true;
        }
    }

#if defined(USE_RX_MSP_OVERRIDE)
    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && rxConfig()->msp_override_channels_mask && rxConfig()->msp_override_failsafe) {
        if (rxMspOverrideFrameStatus() & RX_FRAME_COMPLETE) {
            rxSignalReceived = true;
            rxDataProcessingRequired = true;
            needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }
    }
#endif
    
    DEBUG_SET(DEBUG_FAILSAFE, 1, rxSignalReceived);
    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 0, rxSignalReceived);
}

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
static uint16_t calculateChannelMovingAverage(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
    static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
    static bool rxSamplesCollected = false;

    const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

    // update the recent samples and compute the average of them
    rcSamples[chan][currentSampleIndex] = sample;

    // avoid returning an incorrect average which would otherwise occur before enough samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
    }

    rcDataMean[chan] = 0;
    for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
        rcDataMean[chan] += rcSamples[chan][sampleIndex];
    }
    return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
}
#endif

static uint16_t getRxfailValue(uint8_t channel)
{
    const rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigs(channel);
    const bool boxFailsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);

    switch (channelFailsafeConfig->mode) {
    case RX_FAILSAFE_MODE_AUTO:
        switch (channel) {
        case ROLL:
        case PITCH:
        case YAW:
            return rxConfig()->midrc;
        case THROTTLE:
            if (featureIsEnabled(FEATURE_3D) && !IS_RC_MODE_ACTIVE(BOX3D) && !flight3DConfig()->switched_mode3d) {
                return rxConfig()->midrc;
            } else {
                return rxConfig()->rx_min_usec;
            }
        }

    FALLTHROUGH;
    default:
    case RX_FAILSAFE_MODE_INVALID:
    case RX_FAILSAFE_MODE_HOLD:
        if (boxFailsafeSwitchIsOn) {
            return rcRaw[channel]; // current values are allowed through on held channels with switch induced failsafe
        } else {
            return rcData[channel]; // last good value
        }
    case RX_FAILSAFE_MODE_SET:
        return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step);
    }
}

STATIC_UNIT_TESTED float applyRxChannelRangeConfiguraton(float sample, const rxChannelRangeConfig_t *range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRangef(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    // out of range channel values are now constrained after the validity check in detectAndApplySignalLossBehaviour()
    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    for (int channel = 0; channel < rxChannelCount; channel++) {

        const uint8_t rawChannel = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig()->rcmap[channel] : channel;

        // sample the channel
        float sample;
#if defined(USE_RX_MSP_OVERRIDE)
        if (rxConfig()->msp_override_channels_mask) {
            sample = rxMspOverrideReadRawRc(&rxRuntimeState, rxConfig(), rawChannel);
        } else
#endif
        {
            sample = rxRuntimeState.rcReadRawFn(&rxRuntimeState, rawChannel);
        }

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }

        rcRaw[channel] = sample;
    }
}

void detectAndApplySignalLossBehaviour(void)
{
    const uint32_t currentTimeMs = millis();
    const bool boxFailsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    rxFlightChannelsValid = rxSignalReceived && !boxFailsafeSwitchIsOn;
    // rxFlightChannelsValid is false after 100ms of no packets, or as soon as use the BOXFAILSAFE switch is actioned
    // rxFlightChannelsValid is true the instant we get a good packet or the BOXFAILSAFE switch is reverted
    // can also go false with good packets but where one flight channel is bad > 300ms (PPM type receiver error)

    for (int channel = 0; channel < rxChannelCount; channel++) {
        float sample = rcRaw[channel]; // sample has latest RC value, rcData has last 'accepted valid' value
        const bool thisChannelValid = rxFlightChannelsValid && isPulseValid(sample);
        // if the whole packet is bad, or BOXFAILSAFE switch is actioned, consider all channels bad
        if (thisChannelValid) {
            //  reset the invalid pulse period timer for every good channel
            validRxSignalTimeout[channel] = currentTimeMs + MAX_INVALID_PULSE_TIME_MS;
        }

        if (failsafeIsActive()) {
            // we are in failsafe Stage 2, whether Rx loss or BOXFAILSAFE induced
            // pass valid incoming flight channel values to FC,
            // so that GPS Rescue can get the 30% requirement for termination of the rescue
            if (channel < NON_AUX_CHANNEL_COUNT) {
                if (!thisChannelValid) {
                    if (channel == THROTTLE ) {
                        sample = failsafeConfig()->failsafe_throttle;
                        // stage 2 failsafe throttle value. In GPS Rescue Flight mode, gpsRescueGetThrottle overrides, late in mixer.c
                    } else {
                        sample = rxConfig()->midrc;
                    }
                }
            } else {
                // set aux channels as per Stage 1 failsafe hold/set values, allow all for Failsafe and GPS rescue MODE switches
                sample = getRxfailValue(channel);
            }
        } else {
            // we are normal, or in failsafe stage 1
            if (boxFailsafeSwitchIsOn) {
                // BOXFAILSAFE active, but not in stage 2 yet, use stage 1 values
                sample = getRxfailValue(channel);
                //  set channels to Stage 1 values immediately failsafe switch is activated
            } else if (!thisChannelValid) {
                // everything is normal but this channel was invalid
                if (cmp32(currentTimeMs, validRxSignalTimeout[channel]) < 0) {
                    // first 300ms of Stage 1 failsafe
                    sample = rcData[channel];
                    //  HOLD last valid value on bad channel/s for MAX_INVALID_PULSE_TIME_MS (300ms)
                } else {
                    // remaining Stage 1 failsafe period after 300ms
                    if (channel < NON_AUX_CHANNEL_COUNT) {
                        rxFlightChannelsValid = false;
                        //  declare signal lost after 300ms of any one bad flight channel
                    }
                    sample = getRxfailValue(channel);
                    // set channels that are invalid for more than 300ms to Stage 1 values
                }
            }
            // everything is normal, ie rcData[channel] will be set to rcRaw(channel) via 'sample'
        }

        sample = constrainf(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
        if (rxRuntimeState.rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState.rxProvider == RX_PROVIDER_PPM) {
            //  smooth output for PWM and PPM using moving average
            rcData[channel] = calculateChannelMovingAverage(channel, sample);
        } else
#endif

        {
            //  set rcData to either validated incoming values, or failsafe-modified values
            rcData[channel] = sample;
        }
    }

    if (rxFlightChannelsValid) {
        failsafeOnValidDataReceived();
        //  --> start the timer to exit stage 2 failsafe 100ms after losing all packets or the BOXFAILSAFE switch is actioned
    } else {
        failsafeOnValidDataFailed();
        //  -> start stage 1 timer to enter stage2 failsafe the instant we get a good packet or the BOXFAILSAFE switch is reverted
    }

    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 3, rcData[THROTTLE]);
}

bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    // 处理辅助接收机（如果启用）
    if (rxDualModeEnabled) {
        rxProcessAuxiliaryReceiver();
        
        // 使用主接收机实例
        rxRuntimeState_t *primaryRxState = &rxMultiInstance.rxRuntimeState[rxMultiInstance.primaryInstance];
        
        // 检查超时
        if (currentTimeUs > needRxSignalBefore) {
            rxSignalReceived = false;
            rxFlightChannelsValid = false;
        }
        
        // 读取主接收机通道数据
        if (primaryRxState->rcReadRawFn) {
            rxChannelCount = primaryRxState->channelCount;
            
            for (int channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
                if (channel < rxChannelCount) {
                    rcData[channel] = primaryRxState->rcReadRawFn(primaryRxState, channel);
                }
            }
        }
    } else {
        // 传统单接收机模式（保持原有逻辑）
        // ... 原有代码保持不变 ...
        if (currentTimeUs > needRxSignalBefore) {
            rxSignalReceived = false;
            rxFlightChannelsValid = false;
        }
        
        if (rxRuntimeState.rcReadRawFn) {
            rxChannelCount = rxRuntimeState.channelCount;
            
            for (int channel = 0; channel < MAX_SUPPORTED_RC_CHANNEL_COUNT; channel++) {
                if (channel < rxChannelCount) {
                    rcData[channel] = rxRuntimeState.rcReadRawFn(&rxRuntimeState, channel);
                }
            }
        }
    }

    if (auxiliaryProcessingRequired) {
        rxRuntimeState.rcProcessFrameFn(&rxRuntimeState);
        auxiliaryProcessingRequired = false;
    }

    if (!rxDataProcessingRequired) {
        return false;
    }

    rxDataProcessingRequired = false;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples || currentTimeUs <= suspendRxSignalUntil) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }

        return true;
    }

    readRxChannelsApplyRanges();            // returns rcRaw
    detectAndApplySignalLossBehaviour();    // returns rcData

    rcSampleIndex++;

    return true;
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

void setRssiDirect(uint16_t newRssi, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssi = newRssi;
    rssiRaw = newRssi;
}

void setRssi(uint16_t rssiValue, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    // Filter RSSI value
    if (source == RSSI_SOURCE_FRAME_ERRORS) {
        rssiRaw = pt1FilterApply(&frameErrFilter, rssiValue);
    } else {
        rssiRaw = rssiValue;
    }
}

void setRssiMsp(uint8_t newMspRssi)
{
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_MSP;
    }

    if (rssiSource == RSSI_SOURCE_MSP) {
        rssi = ((uint16_t)newMspRssi) << 2;
        lastMspRssiUpdateUs = micros();
    }
}

static void updateRSSIPWM(void)
{
    // Read value of AUX channel as rssi
    int16_t pwmRssi = rcData[rxConfig()->rssi_channel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssiDirect(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
}
// static void updateRSSIPWM(void)
// {
//     // Read value of AUX channel as rssi from rcRaw to get unprocessed raw channel data
//     // This allows RSSI to be read even when unbound or in failsafe
//     const int16_t rssiChannelIndex = rxConfig()->rssi_channel - 1;
//     int16_t pwmRssi;
    
//     // Use rcRaw instead of rcData to get the actual receiver output
//     // rcRaw contains the latest raw channel data from the receiver
//     if (rssiChannelIndex >= 0 && rssiChannelIndex < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
//         pwmRssi = rcRaw[rssiChannelIndex];
//     } else {
//         pwmRssi = PWM_RANGE_MIN;
//     }
 
//     // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
//     setRssiDirect(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
// }

static void updateRSSIADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
    static uint32_t rssiUpdateAt = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAt) < 0) {
        return;
    }
    rssiUpdateAt = currentTimeUs + DELAY_20_MS;

    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    uint16_t rssiValue = adcRssiSample / RSSI_ADC_DIVISOR;

    setRssi(rssiValue, RSSI_SOURCE_ADC);
#endif
}

void updateRSSI(timeUs_t currentTimeUs)
{
    switch (rssiSource) {
    case RSSI_SOURCE_RX_CHANNEL:
        updateRSSIPWM();
        break;
    case RSSI_SOURCE_ADC:
        updateRSSIADC(currentTimeUs);
        break;
    case RSSI_SOURCE_MSP:
        if (cmpTimeUs(micros(), lastMspRssiUpdateUs) > DELAY_1500_MS) {  // 1.5s
            rssi = 0;
        }
        break;
    default:
        break;
    }

    if (cmpTimeUs(currentTimeUs, lastRssiSmoothingUs) > 250000) { // 0.25s
        lastRssiSmoothingUs = currentTimeUs;
    } else {
        if (lastRssiSmoothingUs != currentTimeUs) { // avoid div by 0
            float k = (256.0f - rxConfig()->rssi_smoothing) / 256.0f;
            float factor = ((currentTimeUs - lastRssiSmoothingUs) / 1000000.0f) / (1.0f / 4.0f);
            float k2  = (k * factor) / ((k * factor) - k + 1);

            if (rssi != rssiRaw) {
                pt1FilterUpdateCutoff(&rssiFilter, k2);
                rssi = pt1FilterApply(&rssiFilter, rssiRaw);
            }

#ifdef USE_RX_RSSI_DBM
            if (rssiDbm != rssiDbmRaw) {
                pt1FilterUpdateCutoff(&rssiDbmFilter, k2);
                rssiDbm = pt1FilterApply(&rssiDbmFilter, rssiDbmRaw);
            }
#endif //USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
            if (rsnr != rsnrRaw) {
                pt1FilterUpdateCutoff(&rsnrFilter, k2);
                rsnr = pt1FilterApply(&rsnrFilter, rsnrRaw);
            }
#endif //USE_RX_RSNR

            lastRssiSmoothingUs = currentTimeUs;
        }
    }
}

uint16_t getRssi(void)
{
    uint16_t rssiValue = rssi;

    // RSSI_Invert option
    if (rxConfig()->rssi_invert) {
        rssiValue = RSSI_MAX_VALUE - rssiValue;
    }

    return rxConfig()->rssi_scale / 100.0f * rssiValue + rxConfig()->rssi_offset * RSSI_OFFSET_SCALING;
}

uint8_t getRssiPercent(void)
{
    return scaleRange(getRssi(), 0, RSSI_MAX_VALUE, 0, 100);
}

#ifdef USE_RX_RSSI_DBM
int16_t getRssiDbm(void)
{
    return rssiDbm;
}

void setRssiDbm(int16_t rssiDbmValue, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssiDbmRaw = rssiDbmValue;
}

void setRssiDbmDirect(int16_t newRssiDbm, rssiSource_e source)
{
    if (source != rssiSource) {
        return;
    }

    rssiDbm = newRssiDbm;
    rssiDbmRaw = newRssiDbm;
}

int8_t getActiveAntenna(void)
{
    return activeAntenna;
}

void setActiveAntenna(int8_t antenna)
{
    activeAntenna = antenna;
}

#endif //USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
int16_t getRsnr(void)
{
    return rsnr;
}

void setRsnr(int16_t rsnrValue)
{
    rsnrRaw = rsnrValue;
}

void setRsnrDirect(int16_t newRsnr)
{
    rsnr = newRsnr;
    rsnrRaw = newRsnr;
}
#endif //USE_RX_RSNR

#ifdef USE_RX_LINK_QUALITY_INFO
uint16_t rxGetLinkQuality(void)
{
    return linkQuality;
}

uint8_t rxGetRfMode(void)
{
    return rfMode;
}

uint16_t rxGetLinkQualityPercent(void)
{
    return (linkQualitySource == LQ_SOURCE_NONE) ? scaleRange(linkQuality, 0, LINK_QUALITY_MAX_VALUE, 0, 100) : linkQuality;
}
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
uint16_t rxGetUplinkTxPwrMw(void)
{
    return uplinkTxPwrMw;
}
#endif

bool isRssiConfigured(void)
{
    return rssiSource != RSSI_SOURCE_NONE;
}

timeDelta_t rxGetFrameDelta(timeDelta_t *frameAgeUs)
{
    static timeUs_t previousFrameTimeUs = 0;
    static timeDelta_t frameTimeDeltaUs = 0;

    if (rxRuntimeState.rcFrameTimeUsFn) {
        const timeUs_t frameTimeUs = rxRuntimeState.rcFrameTimeUsFn();

        *frameAgeUs = cmpTimeUs(micros(), frameTimeUs);

        const timeDelta_t deltaUs = cmpTimeUs(frameTimeUs, previousFrameTimeUs);
        if (deltaUs) {
            frameTimeDeltaUs = deltaUs;
            previousFrameTimeUs = frameTimeUs;
        }
    }

    return frameTimeDeltaUs;
}

timeUs_t rxFrameTimeUs(void)
{
    return rxRuntimeState.lastRcFrameTimeUs;
}

// ============================================================
// 双接收机系统实现
// ============================================================
 
/**
 * 初始化双接收机系统
 * 检查配置并初始化主接收机和辅助接收机
 */
void rxMultiInstanceInit(const rxConfig_t *rxConfig)
{
    // 重置系统状态
    memset(&rxMultiInstance, 0, sizeof(rxMultiInstance));
    rxMultiInstance.activeInstanceCount = 0;
    rxMultiInstance.primaryInstance = RX_INSTANCE_PRIMARY;
    rxMultiInstance.auxiliaryInstance = RX_INSTANCE_AUXILIARY;
    rxMultiInstance.auxiliaryRssiEnabled = false;
    rxMultiInstance.initialized = false;
    rxDualModeEnabled = false;
    
    // 检查是否启用双接收机模式
    if (!rxConfig->rx_dual_mode) {
        // 单接收机模式，使用传统初始化
        return;
    }
    
    // 查找主接收机串口配置
    const serialPortConfig_t *primaryPortConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    // const serialPortConfig_t *primaryPortConfig = serialFindPortConfiguration(SERIAL_PORT_USART1);
    if (primaryPortConfig) {
        // 初始化主接收机实例
        memset(&rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY], 0, sizeof(rxRuntimeState_t));
        rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY].serialrxProvider = rxConfig->serialrx_provider;
        rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY].rxProvider = RX_PROVIDER_SERIAL;
        
        // 初始化函数指针
        rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY].rcReadRawFn = nullReadRawRC;
        rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY].rcFrameStatusFn = nullFrameStatus;
        rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY].rcProcessFrameFn = nullProcessFrame;
        rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY].lastRcFrameTimeUs = 0;
        
        // 尝试初始化串口接收机
        bool primaryEnabled = false;
        
#ifdef USE_SERIALRX_CRSF
        if (rxConfig->serialrx_provider == SERIALRX_CRSF) {
            primaryEnabled = crsfRxInit(rxConfig, &rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY]);
        }
#endif
#ifdef USE_SERIALRX_FPORT
        if (rxConfig->serialrx_provider == SERIALRX_FPORT) {
            primaryEnabled = fportRxInit(rxConfig, &rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY]);
        }
#endif
#ifdef USE_SERIALRX_SB
        if (rxConfig->serialrx_provider == SERIALRX_SBUS) {
            primaryEnabled = sbusInit(rxConfig, &rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY]);
        }
#endif
#ifdef USE_SERIALRX_IBUS
        if (rxConfig->serialrx_provider == SERIALRX_IBUS) {
            primaryEnabled = ibusInit(rxConfig, &rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY]);
        }
#endif
#ifdef USE_SERIALRX_GHST
        if (rxConfig->serialrx_provider == SERIALRX_GHST) {
            primaryEnabled = ghstRxInit(rxConfig, &rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY]);
        }
#endif
#ifdef USE_SERIALRX_SRXL2
        if (rxConfig->serialrx_provider == SERIALRX_SRXL2) {
            primaryEnabled = srxl2RxInit(rxConfig, &rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY]);
        }
#endif
#ifdef USE_SERIALRX_MAVLINK
        if (rxConfig->serialrx_provider == SERIALRX_MAVLINK) {
            primaryEnabled = mavlinkRxInit(rxConfig, &rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY]);
        }
#endif
        
        if (primaryEnabled) {
            rxMultiInstance.activeInstanceCount++;
            rxMultiInstance.primaryInstance = RX_INSTANCE_PRIMARY;
            rxDualModeEnabled = true;
            rxRuntimeState = rxMultiInstance.rxRuntimeState[RX_INSTANCE_PRIMARY];
        } else {
            return;
        }
    }

    for (int i = 0; i < MAX_AUX_RSSI_RECEIVERS; i++) {
        auxRssiData[i].rssi1Dbm = CRSF_RSSI_MIN;
        auxRssiData[i].rssi2Dbm = CRSF_RSSI_MIN;
        auxRssiData[i].activeAntenna = 0;
        auxRssiData[i].lastUpdateMs = 0;
        auxRssiData[i].valid = false;
    }

#ifdef USE_SERIALRX_CRSF
    if (crsfRxAuxInit(rxConfig)) {
        rxMultiInstance.auxiliaryRssiEnabled = true;
    }
#endif

    rxMultiInstance.initialized = true;
}
 
/**
 * 处理指定接收机实例的帧
 * @param instance 接收机实例
 * @param currentTimeUs 当前时间（微秒）
 * @return true 如果帧处理成功
 */
bool rxMultiInstanceProcessFrame(rxInstance_e instance)
{
    if (instance >= MAX_RX_INSTANCES || !rxMultiInstance.initialized) {
        return false;
    }
    
    rxRuntimeState_t *rxState = &rxMultiInstance.rxRuntimeState[instance];
    
    if (!rxState->rcFrameStatusFn) {
        return false;
    }
    
    uint8_t frameStatus = rxState->rcFrameStatusFn(rxState);
    
    if (frameStatus & RX_FRAME_COMPLETE) {
        if (rxState->rcProcessFrameFn(rxState)) {
            return true;
        }
    }
    
    return false;
}
 
/**
 * 设置主接收机实例
 * 用于在两个接收机之间切换飞行控制权
 */
void rxMultiInstanceSetPrimaryInstance(rxInstance_e instance)
{
    if (instance < MAX_RX_INSTANCES && rxMultiInstance.initialized) {
        rxMultiInstance.primaryInstance = instance;
        
        // 更新全局rxRuntimeState以保持兼容性
        rxRuntimeState = rxMultiInstance.rxRuntimeState[instance];
    }
}
 
/**
 * 获取当前主接收机实例
 */
rxInstance_e rxMultiInstanceGetPrimaryInstance(void)
{
    return rxMultiInstance.primaryInstance;
}
 
/**
 * 获取辅助RSSI数据指针
 */
auxiliaryRxRssi_t* rxGetAuxiliaryRssiData(uint8_t auxIndex)
{
    if (auxIndex >= MAX_AUX_RSSI_RECEIVERS) {
        return NULL;
    }

    return &auxRssiData[auxIndex];
}

void rxUpdateAuxiliaryRssi(uint8_t auxIndex, int16_t rssi1Dbm, int16_t rssi2Dbm, uint8_t activeAntenna)
{
    if (auxIndex >= MAX_AUX_RSSI_RECEIVERS) {
        return;
    }

    auxRssiData[auxIndex].rssi1Dbm = rssi1Dbm;
    auxRssiData[auxIndex].rssi2Dbm = rssi2Dbm;
    auxRssiData[auxIndex].activeAntenna = activeAntenna;
    auxRssiData[auxIndex].lastUpdateMs = millis();
    auxRssiData[auxIndex].valid = true;
}

bool rxIsAuxiliaryRssiEnabled(void)
{
    return rxMultiInstance.auxiliaryRssiEnabled;
}

bool rxIsDualModeEnabled(void)
{
    return rxDualModeEnabled;
}

void rxProcessAuxiliaryReceiver(void)
{
    // 辅助串口在ISR中仅处理Link Statistics，无需主循环帧处理
}
 
/**
 * 获取指定接收机实例的状态
 * @param instance 接收机实例
 * @return 接收机状态指针，如果实例无效则返回NULL
 */
rxRuntimeState_t* rxGetRuntimeState(rxInstance_e instance)
{
    if (instance >= MAX_RX_INSTANCES || !rxMultiInstance.initialized) {
        return NULL;
    }
    
    return &rxMultiInstance.rxRuntimeState[instance];
}
 
// ============================================================
