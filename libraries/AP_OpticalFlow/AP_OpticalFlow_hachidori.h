/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#pragma once

#include "OpticalFlow.h"
#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_HACHIDORI : public OpticalFlow_backend
{
public:
    AP_OpticalFlow_HACHIDORI(OpticalFlow &_frontend);
    void init() override;
    void update(void) override;

private:
    uint32_t _last_check_ms;
    Vector2f gyro_sum;        // sum of gyro sensor values since last frame from flow sensor
    uint16_t gyro_sum_count;  // number of gyro sensor values in sum
};
