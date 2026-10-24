/// @file main_driver_benchmark.cpp
/// @brief Generate a vehicle-level driver capability profile before laptime simulation.

#include <fsae/driver/driver_benchmark.hpp>
#include <fsae/vehicle/vehicle_params.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace fsae;

namespace {

struct BenchmarkCliConfig {
    std::string params_file;
    std::string params_json;
    std::string output = "output/runtime/driver_benchmark/capability_profile.json";
    bool quick = false;
    double radius = 15.0;
    double slalom_spacing = 15.0;
    double dt = 0.002;
    double driver_torque_max = -1.0;
};

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "Options:\n"
        << "  --params_file <path>       Vehicle parameter JSON\n"
        << "  --params-json <json>       Inline vehicle parameter JSON\n"
        << "  --output <path>            Output capability profile JSON\n"
        << "  --quick                    Short benchmark sweep for development/CI\n"
        << "  --radius <m>               Constant-radius benchmark radius\n"
        << "  --slalom-spacing <m>       Slalom cone spacing proxy\n"
        << "  --dt <s>                   Benchmark integration step\n"
        << "  --driver-torque-max <Nm>   Override steering driver torque limit\n"
        << "  --help                     Show this message\n";
}

BenchmarkCliConfig parse_args(int argc, char** argv) {
    BenchmarkCliConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--params_file" && i + 1 < argc) {
            config.params_file = argv[++i];
        } else if (arg == "--params-json" && i + 1 < argc) {
            config.params_json = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            config.output = argv[++i];
        } else if (arg == "--quick") {
            config.quick = true;
        } else if (arg == "--radius" && i + 1 < argc) {
            config.radius = std::stod(argv[++i]);
        } else if (arg == "--slalom-spacing" && i + 1 < argc) {
            config.slalom_spacing = std::stod(argv[++i]);
        } else if (arg == "--dt" && i + 1 < argc) {
            config.dt = std::stod(argv[++i]);
        } else if ((arg == "--driver-torque-max"
                    || arg == "--steering-driver-torque-max"
                    || arg == "--steering-torque-max") && i + 1 < argc) {
            config.driver_torque_max = std::stod(argv[++i]);
        } else if (arg == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown or incomplete argument: " + arg);
        }
    }
    return config;
}

vehicle::VehicleParams load_params(const BenchmarkCliConfig& config) {
    if (!config.params_json.empty()) {
        return vehicle::VehicleParams::from_json_string(config.params_json);
    }
    if (!config.params_file.empty()) {
        return vehicle::VehicleParams::from_json(config.params_file);
    }
    return vehicle::VehicleParams::default_fsae();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto cli = parse_args(argc, argv);
        auto params = load_params(cli);
        if (cli.driver_torque_max >= 0.0) {
            if (cli.driver_torque_max <= 0.0) {
                throw std::runtime_error("--driver-torque-max must be positive");
            }
            params.steering.T_driver_max = cli.driver_torque_max;
        }
        driver::DriverBenchmarkConfig benchmark_config;
        benchmark_config.quick = cli.quick;
        benchmark_config.constant_radius_m = cli.radius;
        benchmark_config.slalom_spacing_m = cli.slalom_spacing;
        benchmark_config.dt = cli.dt;

        const auto profile = driver::run_driver_capability_benchmarks(params, benchmark_config);
        const auto json = driver::driver_capability_profile_to_json(profile);

        const std::filesystem::path output_path(cli.output);
        if (output_path.has_parent_path()) {
            std::filesystem::create_directories(output_path.parent_path());
        }
        std::ofstream output(output_path);
        if (!output) {
            throw std::runtime_error("Failed to write output: " + cli.output);
        }
        output << json.dump(2) << "\n";
        std::cout << json.dump(2) << "\n";
    } catch (const std::exception& e) {
        std::cerr << "driver_benchmark failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
