#pragma once

#include "../vehicle/vehicle_params.hpp"

#include <algorithm>
#include <cmath>

namespace fsae::driver {

inline int select_gear_from_vehicle_params(double speed, const vehicle::VehicleParams& params) {
    const auto& ratios = params.powertrain.gear_ratios;
    if (ratios.empty() || params.tire.radius <= 0.0) {
        return 1;
    }

    const double redline_rpm = params.powertrain.engine_rpm_points.empty()
                                   ? 14500.0
                                   : params.powertrain.engine_rpm_points.back();
    const double shift_up_rpm = 0.86 * redline_rpm;
    const double shift_down_rpm = 0.45 * redline_rpm;
    const double wheel_rpm = std::max(0.0, speed) / (2.0 * M_PI * params.tire.radius) * 60.0;

    int selected = 1;
    for (size_t idx = 0; idx < ratios.size(); ++idx) {
        const double engine_rpm =
            wheel_rpm * params.powertrain.primary_ratio * params.powertrain.final_ratio * ratios[idx];
        selected = static_cast<int>(idx) + 1;
        if (engine_rpm <= shift_up_rpm) {
            break;
        }
    }

    if (selected > 1) {
        const double current_rpm = wheel_rpm * params.powertrain.primary_ratio *
                                   params.powertrain.final_ratio * ratios[selected - 1];
        if (current_rpm < shift_down_rpm) {
            selected -= 1;
        }
    }

    return std::clamp(selected, 1, static_cast<int>(ratios.size()));
}

}  // namespace fsae::driver
