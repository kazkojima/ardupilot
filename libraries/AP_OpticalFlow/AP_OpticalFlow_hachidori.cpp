/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_OpticalFlow_hachidori.h"

#if AP_OPTICALFLOW_HACHIDORI_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <AP_AHRS/AP_AHRS.h>

#include "AP_OpticalFlow.h"

#include <unistd.h>

#include <AP_InertialSensor/AP_InertialSensor_hachidori.h>

#include <AP_HAL_Linux/hachidori/hachidori_packet.h>

#define TIMEOUT_MS 200

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_HACHIDORI::AP_OpticalFlow_HACHIDORI(AP_OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend)
{}

void AP_OpticalFlow_HACHIDORI::init(void)
{
    /* register callback to get gyro data */
    hal.opticalflow->init();
}

void AP_OpticalFlow_HACHIDORI::update()
{
    // record gyro values as long as they are being used
    // the sanity check of dt below ensures old gyro values are not used
    if (gyro_sum_count < 1000) {
        const Vector3f& gyro = AP::ahrs().get_gyro();
        gyro_sum.x += gyro.x;
        gyro_sum.y += gyro.y;
        gyro_sum_count++;
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_check_ms < 10) {
        return;
    }
    _last_check_ms = now;

    if (!HACHIDORI_PORT::updated[HACHIDORI_TOS_OF]) {
        return;
    }

    // deserialize packet bytes to floats
    uint8_t *d = HACHIDORI_PORT::buf[HACHIDORI_TOS_OF].data;
    le32_t *fp = reinterpret_cast<le32_t *>(d);
    union { float f; uint32_t u32; } flow_x, flow_y;
    flow_x.u32 = le32toh(fp[0]);
    flow_y.u32 = le32toh(fp[1]);
    uint8_t qual = d[8];
    uint8_t dt = d[10];
    HACHIDORI_PORT::updated[HACHIDORI_TOS_OF] = false;

    struct AP_OpticalFlow::OpticalFlow_state state;
    state.surface_quality = qual;
    if (dt > 0 && dt < TIMEOUT_MS) {
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

        // flow_? is in radians per second
        state.flowRate.x = flowScaleFactorX * flow_x.f;
        state.flowRate.y = flowScaleFactorY * flow_y.f;

        // copy average body rate to state structure
        state.bodyRate = Vector2f(gyro_sum.x / gyro_sum_count,
                                  gyro_sum.y / gyro_sum_count);

        _applyYaw(state.flowRate);
        _applyYaw(state.bodyRate);
    } else {
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    // copy results to front end
    _update_frontend(state);

    // reset gyro sum
    gyro_sum.zero();
    gyro_sum_count = 0;

#if 0
    hal.console->printf("FLOW_HACHIDORI qual:%u FlowRateX:%4.2f Y:%4.2f"
                        "BodyRateX:%4.2f Y:%4.2f\n",
                        (unsigned)state.surface_quality,
                        (double)state.flowRate.x,
                        (double)state.flowRate.y,
                        (double)state.bodyRate.x,
                        (double)state.bodyRate.y);
#endif
}

#endif  // AP_OPTICALFLOW_HACHIDORI_ENABLED
