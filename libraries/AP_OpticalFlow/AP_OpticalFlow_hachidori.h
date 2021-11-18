/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#pragma once

#include "AP_OpticalFlow.h"

#ifndef AP_OPTICALFLOW_HACHIDORI_ENABLED
#define AP_OPTICALFLOW_HACHIDORI_ENABLED AP_OPTICALFLOW_ENABLED
#endif

#if AP_OPTICALFLOW_HACHIDORI_ENABLED

#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_HACHIDORI : public OpticalFlow_backend
{
public:
    AP_OpticalFlow_HACHIDORI(AP_OpticalFlow &_frontend);
    void init() override;
    void update(void) override;

private:
    uint32_t _last_check_ms;
    Vector2f gyro_sum;        // sum of gyro sensor values since last frame from flow sensor
    uint16_t gyro_sum_count;  // number of gyro sensor values in sum
};

#endif  // AP_OPTICALFLOW_HACHIDORI_ENABLED
