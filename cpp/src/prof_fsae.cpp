#include <fsae/vehicle/vehicle_14dof_new.hpp>
#include <fsae/vehicle/vehicle_params.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace fsae;

int main(int argc, char** argv) {
    int num_steps = 1000 * 20; // 20 simulated seconds at the canonical 1 ms step.
    bool diagnostics_enabled = false;
    double pinn_eval_rate_hz = vehicle::VehicleParams::default_fsae().tire.pinn_eval_rate_hz;
    bool rate_was_requested = false;
    bool steps_were_requested = false;
    const auto print_usage = [] {
        std::cerr
            << "Usage: prof_fsae [num_physics_steps] [--diagnostics] "
               "[--pinn-eval-rate-hz HZ]\n"
            << "  HZ=0 preserves evaluation at every RK4/K&C call; HZ>0 enables "
               "simulation-time ZOH.\n";
    };
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (argument == "--diagnostics") {
            diagnostics_enabled = true;
            continue;
        }
        if (argument == "--pinn-eval-rate-hz") {
            if (rate_was_requested || i + 1 >= argc) {
                print_usage();
                return 2;
            }
            try {
                std::size_t parsed = 0;
                const std::string value = argv[++i];
                pinn_eval_rate_hz = std::stod(value, &parsed);
                if (parsed != value.size() || !std::isfinite(pinn_eval_rate_hz) ||
                    pinn_eval_rate_hz < 0.0) {
                    throw std::invalid_argument("invalid rate");
                }
            } catch (const std::exception&) {
                std::cerr << "pinn eval rate must be a finite non-negative number\n";
                return 2;
            }
            rate_was_requested = true;
            continue;
        }
        if (steps_were_requested) {
            print_usage();
            return 2;
        }
        try {
            std::size_t parsed = 0;
            num_steps = std::stoi(argument, &parsed);
            if (parsed != argument.size() || num_steps <= 0) {
                throw std::invalid_argument("invalid step count");
            }
        } catch (const std::exception&) {
            std::cerr << "num_physics_steps must be a positive integer" << std::endl;
            return 2;
        }
        steps_were_requested = true;
    }

    vehicle::VehicleParams params;
    params.tire.pinn_eval_rate_hz = pinn_eval_rate_hz;
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(5.0);
    vehicle.set_nn_tire_runtime_diagnostics_enabled(diagnostics_enabled);
    vehicle.reset_nn_tire_runtime_diagnostics();
    
    vehicle::DriverInput input;
    input.steering = 0.0;
    input.throttle = 0.5;
    input.brake = 0.0;
    
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_steps; ++i) {
        vehicle.step(0.001, input);
    }
    auto end = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double, std::milli> diff = end - start;
    const double simulated_ms = static_cast<double>(num_steps);
    std::cout << "Time for " << num_steps << " physics steps: " << diff.count() << " ms" << std::endl;
    std::cout << "Time per physics step: " << diff.count() / num_steps << " ms" << std::endl;
    std::cout << "Wall/sim ratio: " << diff.count() / simulated_ms << std::endl;
    std::cout << "PINN eval rate: " << pinn_eval_rate_hz << " Hz\n";
    if (diagnostics_enabled || rate_was_requested) {
        const auto& final_state = vehicle.state();
        std::cout << std::setprecision(17);
        std::cout << "Final state:"
                  << " x=" << final_state.x
                  << " y=" << final_state.y
                  << " yaw=" << final_state.psi
                  << " vx=" << final_state.vx << '\n';
    }
    if (diagnostics_enabled) {
        const auto diagnostics = vehicle.nn_tire_runtime_diagnostics();
        const double calls_per_step = static_cast<double>(diagnostics.prediction_calls) / num_steps;
        const double forwards_per_prediction = diagnostics.prediction_calls == 0
            ? 0.0
            : static_cast<double>(diagnostics.forward_physical_calls) /
                static_cast<double>(diagnostics.prediction_calls);
        std::cout << "Diagnostics:\n";
        std::cout << "  Prediction calls: " << diagnostics.prediction_calls << '\n';
        std::cout << "  Prediction calls/physics step: " << calls_per_step << '\n';
        std::cout << "  forward_physical calls: " << diagnostics.forward_physical_calls << '\n';
        std::cout << "  forward_physical calls/prediction: " << forwards_per_prediction << '\n';
        std::cout << "  Zero-reference forwards: " << diagnostics.zero_reference_forwards << '\n';
        std::cout << "  Symmetry forwards: " << diagnostics.symmetry_forwards << '\n';
        std::cout << "  SA-zero forwards: " << diagnostics.sa_zero_forwards << '\n';
        std::cout << "  Friction-cap projections: " << diagnostics.friction_cap_projections << '\n';
    }
    return 0;
}
