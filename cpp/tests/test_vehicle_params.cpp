#include <gtest/gtest.h>

#include "fsae/vehicle/vehicle_params.hpp"

#include <stdexcept>

using fsae::vehicle::VehicleParams;

TEST(VehicleParamsTest, FromJsonThrowsWhenFileCannotOpen) {
    EXPECT_THROW(
        VehicleParams::from_json("/tmp/fsae_missing_vehicle_params_for_test.json"),
        std::runtime_error);
}

TEST(VehicleParamsTest, FromJsonStringThrowsOnInvalidJson) {
    EXPECT_THROW(VehicleParams::from_json_string("{invalid-json"), nlohmann::json::parse_error);
}

TEST(VehicleParamsTest, FromJsonStringParsesSteeringDriverTorqueRateLimit) {
    const auto params = VehicleParams::from_json_string(R"({
        "steering": {
            "torque_mode": true,
            "T_driver_max": 40.0,
            "T_driver_rate_max": 180.0
        }
    })");

    EXPECT_TRUE(params.steering.torque_mode);
    EXPECT_DOUBLE_EQ(params.steering.T_driver_max, 40.0);
    EXPECT_DOUBLE_EQ(params.steering.T_driver_rate_max, 180.0);
}
