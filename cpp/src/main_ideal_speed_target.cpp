/// @file main_ideal_speed_target.cpp
/// @brief Compute target-speed-only lap-time bounds for fixed-course driver planning.

#include <fsae/control/mpc_problem.hpp>
#include <fsae/driver/attack_speed_planner.hpp>
#include <fsae/driver/pid_driver.hpp>
#include <fsae/driver/track_phase_analyzer.hpp>
#include <fsae/laptime/velocity_profile.hpp>
#include <fsae/track/track_model.hpp>
#include <fsae/track/waypoint_loader.hpp>
#include <fsae/vehicle/vehicle_params.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

using namespace fsae;
using json = nlohmann::json;

namespace {

struct CliConfig {
    std::string track_name = "autocross";
    double track_radius = 30.0;
    double track_width = 3.0;
    double initial_speed = 6.0;
    std::string params_file;
    std::string params_json;
    std::string attack_speed_cap_file;
    bool torque_mode_override = false;
    bool torque_mode = false;
    bool optimize_grid = false;
    bool output_json = false;
    std::string csv_output;

    double max_lon_accel = 1.0 * 9.81;
    double max_lon_decel = 1.8 * 9.81;
    double max_speed = 50.0;
    double attack_speed_lateral_safety = 0.66;
    double attack_speed_braking_safety = 0.78;
    double attack_speed_accel_limit = 5.5;

    std::vector<double> mass_list = {200.0, 220.0, 250.0, 280.0};
    std::vector<double> cl_a_list = {0.0, 2.8, 5.0};
    std::vector<double> mu_y_list = {2.0, 3.0, 4.0, 6.0, 8.0};
    std::vector<double> mu_x_list = {1.6, 2.5, 4.0, 6.0};
    std::vector<double> max_lon_accel_list = {5.5, 9.81, 15.0, 25.0};
    std::vector<double> max_lon_decel_list = {12.0, 18.0, 25.0, 35.0};
    std::vector<double> max_speed_list = {30.0, 40.0, 50.0, 60.0};
};

struct DesignPoint {
    double mass = 0.0;
    double cl_a = 0.0;
    double mu_y = 0.0;
    double mu_x = 0.0;
    double max_lon_accel = 0.0;
    double max_lon_decel = 0.0;
    double max_speed = 0.0;
};

struct IdealSpeedMetrics {
    DesignPoint design;
    double track_length = 0.0;
    double base_profile_time = std::numeric_limits<double>::infinity();
    double attack_target_time = std::numeric_limits<double>::infinity();
    double attack_final_target_time = std::numeric_limits<double>::infinity();
    double attack_preview_min_target_time = std::numeric_limits<double>::infinity();
    double base_mean_speed = 0.0;
    double attack_mean_speed = 0.0;
    double attack_final_mean_speed = 0.0;
    double attack_preview_min_mean_speed = 0.0;
    double base_min_speed = std::numeric_limits<double>::infinity();
    double base_max_speed = 0.0;
    double attack_min_speed = std::numeric_limits<double>::infinity();
    double attack_max_speed = 0.0;
    std::array<int, 8> limiting_reason_counts{};
    std::size_t sample_count = 0;
};

void print_usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "Options:\n"
        << "  --track <name|csv>              Track name or CSV path (default: autocross)\n"
        << "  --track-width <m>               Track width override\n"
        << "  --radius <m>                    Radius for circle/skidpad/straight_corner\n"
        << "  --params_file <path>            Vehicle parameter JSON\n"
        << "  --params-json <json>            Inline vehicle parameter JSON\n"
        << "  --torque-mode <0|1>             Override steering torque_mode for planner\n"
        << "  --attack-speed-cap-file <path>  Optional fixed-course cap JSON\n"
        << "  --optimize-grid                 Evaluate all design grid combinations\n"
        << "  --output-json                   Print machine-readable JSON block\n"
        << "  --csv-output <path>             Write all grid rows to CSV\n"
        << "  --max-lon-accel <m/s2>          Baseline acceleration cap\n"
        << "  --max-lon-decel <m/s2>          Baseline braking cap\n"
        << "  --max-speed <m/s>               Baseline max speed\n"
        << "  --attack-speed-lateral-safety <x>\n"
        << "  --attack-speed-braking-safety <x>\n"
        << "  --attack-speed-accel-limit <m/s2>\n"
        << "  --mass-list <csv>               Grid masses; values below 200 are rejected\n"
        << "  --cl-a-list <csv>               Grid CL_A; values above 5 are rejected\n"
        << "  --mu-y-list <csv>\n"
        << "  --mu-x-list <csv>\n"
        << "  --max-lon-accel-list <csv>\n"
        << "  --max-lon-decel-list <csv>\n"
        << "  --max-speed-list <csv>\n";
}

std::vector<double> parse_double_list(const std::string& text, const std::string& name) {
    std::vector<double> values;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) {
            continue;
        }
        const double value = std::stod(item);
        if (!std::isfinite(value)) {
            throw std::runtime_error(name + " contains a non-finite value");
        }
        values.push_back(value);
    }
    if (values.empty()) {
        throw std::runtime_error(name + " must contain at least one value");
    }
    return values;
}

bool parse_bool_like(const std::string& text) {
    return text == "1" || text == "true" || text == "TRUE" || text == "yes" || text == "on";
}

CliConfig parse_args(int argc, char** argv) {
    CliConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error("Missing value for " + option);
            }
            return argv[++i];
        };

        if (arg == "--track") {
            config.track_name = require_value(arg);
        } else if (arg == "--track-width" || arg == "--width") {
            config.track_width = std::stod(require_value(arg));
        } else if (arg == "--radius") {
            config.track_radius = std::stod(require_value(arg));
        } else if (arg == "--initial-speed") {
            config.initial_speed = std::stod(require_value(arg));
        } else if (arg == "--params_file" || arg == "--params-file") {
            config.params_file = require_value(arg);
        } else if (arg == "--params-json") {
            config.params_json = require_value(arg);
        } else if (arg == "--torque-mode") {
            config.torque_mode_override = true;
            config.torque_mode = parse_bool_like(require_value(arg));
        } else if (arg == "--attack-speed-cap-file") {
            config.attack_speed_cap_file = require_value(arg);
        } else if (arg == "--optimize-grid") {
            config.optimize_grid = true;
        } else if (arg == "--output-json") {
            config.output_json = true;
        } else if (arg == "--csv-output") {
            config.csv_output = require_value(arg);
        } else if (arg == "--max-lon-accel") {
            config.max_lon_accel = std::stod(require_value(arg));
        } else if (arg == "--max-lon-decel") {
            config.max_lon_decel = std::stod(require_value(arg));
        } else if (arg == "--max-speed") {
            config.max_speed = std::stod(require_value(arg));
        } else if (arg == "--attack-speed-lateral-safety") {
            config.attack_speed_lateral_safety = std::stod(require_value(arg));
        } else if (arg == "--attack-speed-braking-safety") {
            config.attack_speed_braking_safety = std::stod(require_value(arg));
        } else if (arg == "--attack-speed-accel-limit") {
            config.attack_speed_accel_limit = std::stod(require_value(arg));
        } else if (arg == "--mass-list") {
            config.mass_list = parse_double_list(require_value(arg), arg);
        } else if (arg == "--cl-a-list") {
            config.cl_a_list = parse_double_list(require_value(arg), arg);
        } else if (arg == "--mu-y-list") {
            config.mu_y_list = parse_double_list(require_value(arg), arg);
        } else if (arg == "--mu-x-list") {
            config.mu_x_list = parse_double_list(require_value(arg), arg);
        } else if (arg == "--max-lon-accel-list") {
            config.max_lon_accel_list = parse_double_list(require_value(arg), arg);
        } else if (arg == "--max-lon-decel-list") {
            config.max_lon_decel_list = parse_double_list(require_value(arg), arg);
        } else if (arg == "--max-speed-list") {
            config.max_speed_list = parse_double_list(require_value(arg), arg);
        } else if (arg == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("Unknown argument: " + arg);
        }
    }
    return config;
}

std::string resolve_autocross_csv() {
    const char* env_path = std::getenv("FOPT_AUTOCROSS_CSV");
    if (env_path != nullptr && std::string(env_path).size() > 0) {
        return env_path;
    }
    const std::vector<std::string> candidates = {
        "data/ASE_autocross_sim.csv",
        "sim/data/ASE_autocross_sim.csv",
        "../data/ASE_autocross_sim.csv",
        "../sim/data/ASE_autocross_sim.csv",
        "/workspace/sim/data/ASE_autocross_sim.csv",
        "/workspace/data/ASE_autocross_sim.csv",
    };
    for (const auto& candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return "data/ASE_autocross_sim.csv";
}

track::CurvatureBasedTrack create_track(const CliConfig& config) {
    if (config.track_name.size() > 4
        && config.track_name.substr(config.track_name.size() - 4) == ".csv") {
        auto waypoints = track::WaypointTrackBuilder::load_from_csv(
            config.track_name,
            config.track_width);
        return track::WaypointTrackBuilder(waypoints, config.track_width).build();
    }
    if (config.track_name == "circle") {
        return track::create_circle_track(config.track_radius, config.track_width);
    }
    if (config.track_name == "circle_cw") {
        return track::create_circle_cw_track(config.track_radius, config.track_width);
    }
    if (config.track_name == "figure_eight") {
        return track::create_figure_eight_track(config.track_radius, config.track_width);
    }
    if (config.track_name == "skidpad") {
        const double radius = config.track_radius != 30.0 ? config.track_radius : 9.125;
        return track::create_skidpad_track(radius, config.track_width);
    }
    if (config.track_name == "straight_corner") {
        return track::create_straight_corner_track(50.0, config.track_radius, config.track_width);
    }
    if (config.track_name == "acceleration" || config.track_name == "accel") {
        return track::create_acceleration_track(75.0, config.track_width);
    }
    if (config.track_name == "autocross" || config.track_name == "ASE_autocross") {
        auto waypoints = track::WaypointTrackBuilder::load_from_csv(
            resolve_autocross_csv(),
            config.track_width);
        return track::WaypointTrackBuilder(waypoints, config.track_width).build();
    }
    throw std::runtime_error("Unknown track: " + config.track_name);
}

vehicle::VehicleParams load_vehicle_params(const CliConfig& config) {
    vehicle::VehicleParams params = vehicle::VehicleParams::default_fsae();
    if (!config.params_json.empty()) {
        params = vehicle::VehicleParams::from_json_string(config.params_json);
    } else if (!config.params_file.empty()) {
        params = vehicle::VehicleParams::from_json(config.params_file);
    }
    if (config.torque_mode_override) {
        params.steering.torque_mode = config.torque_mode;
    }
    return params;
}

std::vector<std::array<double, 3>> load_attack_speed_caps(const std::string& filepath) {
    if (filepath.empty()) {
        return {};
    }
    std::ifstream ifs(filepath);
    if (!ifs) {
        throw std::runtime_error("Failed to open attack speed cap file: " + filepath);
    }
    json root;
    ifs >> root;
    const json* caps_json = nullptr;
    if (root.is_array()) {
        caps_json = &root;
    } else if (root.is_object() && root.contains("speed_caps")) {
        caps_json = &root.at("speed_caps");
    } else if (root.is_object() && root.contains("caps")) {
        caps_json = &root.at("caps");
    }
    if (caps_json == nullptr || !caps_json->is_array()) {
        return {};
    }
    std::vector<std::array<double, 3>> caps;
    for (const auto& item : *caps_json) {
        double s = std::numeric_limits<double>::quiet_NaN();
        double speed = std::numeric_limits<double>::quiet_NaN();
        double radius = 15.0;
        if (item.is_array() && item.size() >= 2) {
            s = item.at(0).get<double>();
            speed = item.at(1).get<double>();
            if (item.size() >= 3) {
                radius = item.at(2).get<double>();
            }
        } else if (item.is_object()) {
            s = item.value("s", item.value("s_m", s));
            speed = item.value("speed", item.value("speed_mps", item.value("v", speed)));
            radius = item.value("radius", item.value("radius_m", radius));
        }
        if (std::isfinite(s) && std::isfinite(speed) && std::isfinite(radius)
                && speed > 0.0 && radius > 0.0) {
            caps.push_back({s, speed, radius});
        }
    }
    return caps;
}

std::vector<std::array<double, 3>> load_attack_speed_windows(const std::string& filepath) {
    if (filepath.empty()) {
        return {};
    }
    std::ifstream ifs(filepath);
    if (!ifs) {
        throw std::runtime_error("Failed to open attack speed cap file: " + filepath);
    }
    json root;
    ifs >> root;
    if (!root.is_object() || !root.contains("window_caps")
            || !root.at("window_caps").is_array()) {
        return {};
    }
    std::vector<std::array<double, 3>> windows;
    for (const auto& item : root.at("window_caps")) {
        double s_start = std::numeric_limits<double>::quiet_NaN();
        double s_end = std::numeric_limits<double>::quiet_NaN();
        double speed = std::numeric_limits<double>::quiet_NaN();
        if (item.is_array() && item.size() >= 3) {
            s_start = item.at(0).get<double>();
            s_end = item.at(1).get<double>();
            speed = item.at(2).get<double>();
        } else if (item.is_object()) {
            s_start = item.value("s_start", item.value("start", item.value("start_s", s_start)));
            s_end = item.value("s_end", item.value("end", item.value("end_s", s_end)));
            speed = item.value("speed", item.value("speed_mps", item.value("v", speed)));
        }
        if (std::isfinite(s_start) && std::isfinite(s_end) && std::isfinite(speed)
                && speed > 0.0) {
            windows.push_back({s_start, s_end, speed});
        }
    }
    return windows;
}

laptime::VehiclePerformance make_vehicle_performance(
    const vehicle::VehicleParams& params,
    const CliConfig& config,
    const DesignPoint& design)
{
    laptime::VehiclePerformance perf;
    perf.max_lon_accel = design.max_lon_accel;
    perf.max_lon_decel = design.max_lon_decel;
    perf.max_speed = design.max_speed;
    perf.use_physics_model = true;
    perf.mass = design.mass;
    perf.cl_a = design.cl_a;
    perf.cd_a = params.aero.CD_A;
    perf.rho = params.aero.rho > 0.1 ? params.aero.rho : 1.225;
    perf.mu_lat = design.mu_y;
    perf.mu_lon = design.mu_x;
    perf.safety_factor = 0.95;
    perf.use_load_sensitive_lateral_limit = params.tire.model_type == "tmeasy";
    perf.tire_nominal_load = params.tire.Fz_nom;
    perf.tire_load_sensitivity = params.tire.dmu_dFz;
    perf.front_static_load_fraction = params.mass.wheelbase > 1e-6
        ? params.mass.b / params.mass.wheelbase
        : 0.50;
    perf.front_aero_balance = params.aero.aero_balance;
    perf.track_front = params.mass.track_f;
    perf.track_rear = params.mass.track_r;
    perf.cg_height = params.mass.h_cg;
    perf.aero_nominal_front_ride_height_mm = params.aero.nominal_front_ride_height_mm > 0.0
        ? params.aero.nominal_front_ride_height_mm
        : 40.0;
    perf.aero_nominal_rear_ride_height_mm = params.aero.nominal_rear_ride_height_mm > 0.0
        ? params.aero.nominal_rear_ride_height_mm
        : 55.0;
    perf.aero_nominal_roll_rad = params.aero.nominal_roll_rad;
    perf.aero_nominal_yaw_rad = params.aero.nominal_yaw_rad;

    if (params.steering.torque_mode) {
        const auto torque_gains = driver::optimized_autocross_torque_gains();
        perf.use_steering_actuator_limits = true;
        perf.wheelbase = params.mass.wheelbase;
        perf.max_tire_steer_angle = params.steering.max_angle;
        perf.max_tire_steer_rate = torque_gains.max_tire_steer_rate;
        perf.steering_inertia = params.steering.I_steer;
        perf.steering_friction_torque = params.steering.T_fric;
        perf.steering_driver_torque_max = params.steering.T_driver_max;
        perf.steering_response_omega_n = torque_gains.inner_loop_omega_n;
        perf.steering_response_zeta = torque_gains.inner_loop_zeta;
        perf.steering_understeer_gradient = torque_gains.understeer_gradient;
        perf.steering_preview_time = torque_gains.torque_speed_cap_preview_time;
        perf.steering_preview_distance = torque_gains.torque_speed_cap_preview_distance;
        perf.steering_min_preview_distance = torque_gains.torque_speed_cap_min_preview_distance;
        perf.steering_max_preview_distance = torque_gains.torque_speed_cap_max_preview_distance;
    }

    (void)config;
    return perf;
}

double integrate_attack_profile(const driver::AttackSpeedProfile& profile) {
    const auto& samples = profile.samples();
    double lap_time = 0.0;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double ds = samples[i].s - samples[i - 1].s;
        const double v_avg = 0.5 * (samples[i].planned_speed + samples[i - 1].planned_speed);
        if (ds > 0.0 && v_avg > 1e-6) {
            lap_time += ds / v_avg;
        }
    }
    return lap_time;
}

double integrate_attack_preview_min_profile(const driver::AttackSpeedProfile& profile) {
    const auto& samples = profile.samples();
    double lap_time = 0.0;
    auto preview_target = [&](double s) {
        const double current_speed = std::max(profile.planned_speed_at(s), 0.5);
        const double preview_distance = std::clamp(current_speed * 1.10, 6.0, 32.0);
        return profile.preview_min_planned_speed(s, preview_distance);
    };
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const double ds = samples[i].s - samples[i - 1].s;
        const double v0 = preview_target(samples[i - 1].s);
        const double v1 = preview_target(samples[i].s);
        const double v_avg = 0.5 * (v0 + v1);
        if (ds > 0.0 && v_avg > 1e-6) {
            lap_time += ds / v_avg;
        }
    }
    return lap_time;
}

double average_speed_from_time(double length, double lap_time) {
    return lap_time > 1e-9 ? length / lap_time : 0.0;
}

std::array<int, 8> count_limiting_reasons(const driver::AttackSpeedProfile& profile) {
    std::array<int, 8> counts{};
    for (const auto& sample : profile.samples()) {
        const auto index = static_cast<std::size_t>(sample.limiting_reason);
        if (index < counts.size()) {
            counts[index]++;
        }
    }
    return counts;
}

double attack_profile_floor_scale(const control::MPCConfig& mpc, driver::TrackPhase phase) {
    switch (phase) {
        case driver::TrackPhase::Straight:
            return mpc.attack_floor_straight;
        case driver::TrackPhase::Exit:
            return mpc.attack_floor_exit;
        case driver::TrackPhase::Complex:
            return mpc.attack_floor_complex;
        case driver::TrackPhase::TurnIn:
            return mpc.attack_floor_turn_in;
        case driver::TrackPhase::Apex:
            return mpc.attack_floor_apex;
        case driver::TrackPhase::Brake:
            return mpc.attack_floor_brake;
    }
    return mpc.attack_floor_complex;
}

driver::AttackSpeedProfile apply_attack_driver_floor(
    const driver::AttackSpeedProfile& attack_profile,
    const driver::TrackPhaseProfile& phase_profile,
    const std::vector<laptime::VelocityPoint>& base_profile,
    const control::MPCConfig& mpc)
{
    std::vector<driver::AttackSpeedSample> samples;
    samples.reserve(attack_profile.samples().size());
    for (const auto& sample : attack_profile.samples()) {
        auto adjusted = sample;
        const auto phase = phase_profile.phase_at(sample.s);
        const double base_speed = [&]() {
            auto it = std::lower_bound(
                base_profile.begin(),
                base_profile.end(),
                sample.s,
                [](const laptime::VelocityPoint& point, double value) {
                    return point.s < value;
                });
            if (base_profile.empty()) {
                return 0.0;
            }
            if (it == base_profile.begin()) {
                return it->v;
            }
            if (it == base_profile.end()) {
                return base_profile.back().v;
            }
            const auto prev = std::prev(it);
            const double span = std::max(it->s - prev->s, 1e-9);
            const double alpha = std::clamp((sample.s - prev->s) / span, 0.0, 1.0);
            return prev->v + alpha * (it->v - prev->v);
        }();
        const bool hard_limited =
            sample.limiting_reason != driver::AttackSpeedLimitReason::Profile
            && sample.limiting_reason != driver::AttackSpeedLimitReason::Phase;
        const double profile_floor = base_speed * attack_profile_floor_scale(mpc, phase);
        adjusted.planned_speed = std::clamp(
            hard_limited ? sample.planned_speed : std::max(sample.planned_speed, profile_floor),
            mpc.vx_min,
            mpc.vx_max);
        samples.push_back(adjusted);
    }
    return driver::AttackSpeedProfile(std::move(samples), attack_profile.length(), true);
}

IdealSpeedMetrics evaluate_design(
    const track::CurvatureBasedTrack& track,
    const vehicle::VehicleParams& base_params,
    const CliConfig& cli,
    const DesignPoint& design,
    const std::vector<std::array<double, 3>>& external_caps,
    const std::vector<std::array<double, 3>>& external_windows)
{
    auto params = base_params;
    params.mass.m_total = design.mass;
    params.aero.CL_A = design.cl_a;
    params.tire.mu_y_max = design.mu_y;
    params.tire.mu_x_max = design.mu_x;

    const auto perf = make_vehicle_performance(params, cli, design);
    laptime::VelocityProfileGenerator generator(perf);
    const auto base_profile = generator.generate(track, 1.0, cli.initial_speed);
    const auto phase_profile = driver::TrackPhaseAnalyzer::analyze(track, base_profile);

    control::MPCConfig mpc;
    mpc.vx_min = 2.0;
    mpc.vx_max = design.max_speed;
    mpc.attack_speed_lateral_safety = cli.attack_speed_lateral_safety;
    mpc.attack_speed_braking_safety = cli.attack_speed_braking_safety;
    mpc.attack_speed_accel_limit = cli.attack_speed_accel_limit;
    mpc.attack_speed_caps = external_caps;
    mpc.attack_speed_windows = external_windows;

    driver::AttackSpeedPlannerConfig planner_config;
    planner_config.min_speed = mpc.vx_min;
    planner_config.max_speed = mpc.vx_max;
    planner_config.lateral_safety = mpc.attack_speed_lateral_safety;
    planner_config.braking_safety = mpc.attack_speed_braking_safety;
    planner_config.accel_limit = mpc.attack_speed_accel_limit;
    planner_config.external_speed_caps = external_caps;
    planner_config.external_speed_windows = external_windows;
    const auto raw_attack_profile =
        driver::AttackSpeedPlanner::plan(track, base_profile, params, phase_profile, planner_config);
    const auto attack_profile =
        apply_attack_driver_floor(raw_attack_profile, phase_profile, base_profile, mpc);

    IdealSpeedMetrics metrics;
    metrics.design = design;
    metrics.track_length = track.length();
    metrics.base_profile_time = laptime::compute_lap_time(base_profile);
    metrics.attack_target_time = integrate_attack_profile(attack_profile);
    metrics.attack_final_target_time = metrics.attack_target_time;
    metrics.attack_preview_min_target_time = integrate_attack_preview_min_profile(attack_profile);
    metrics.base_mean_speed = average_speed_from_time(metrics.track_length, metrics.base_profile_time);
    metrics.attack_mean_speed = average_speed_from_time(metrics.track_length, metrics.attack_target_time);
    metrics.attack_final_mean_speed =
        average_speed_from_time(metrics.track_length, metrics.attack_final_target_time);
    metrics.attack_preview_min_mean_speed =
        average_speed_from_time(metrics.track_length, metrics.attack_preview_min_target_time);
    metrics.limiting_reason_counts = count_limiting_reasons(raw_attack_profile);
    metrics.sample_count = raw_attack_profile.samples().size();

    for (const auto& point : base_profile) {
        metrics.base_min_speed = std::min(metrics.base_min_speed, point.v);
        metrics.base_max_speed = std::max(metrics.base_max_speed, point.v);
    }
    for (const auto& sample : attack_profile.samples()) {
        metrics.attack_min_speed = std::min(metrics.attack_min_speed, sample.planned_speed);
        metrics.attack_max_speed = std::max(metrics.attack_max_speed, sample.planned_speed);
    }
    return metrics;
}

json design_to_json(const DesignPoint& design) {
    return {
        {"mass", design.mass},
        {"CL_A", design.cl_a},
        {"mu_y", design.mu_y},
        {"mu_x", design.mu_x},
        {"max_lon_accel", design.max_lon_accel},
        {"max_lon_decel", design.max_lon_decel},
        {"max_speed", design.max_speed},
    };
}

json metrics_to_json(const IdealSpeedMetrics& metrics) {
    json reason_counts = json::object();
    const std::array<driver::AttackSpeedLimitReason, 8> reasons = {
        driver::AttackSpeedLimitReason::Profile,
        driver::AttackSpeedLimitReason::Phase,
        driver::AttackSpeedLimitReason::Curvature,
        driver::AttackSpeedLimitReason::Steering,
        driver::AttackSpeedLimitReason::Margin,
        driver::AttackSpeedLimitReason::ExternalCap,
        driver::AttackSpeedLimitReason::Braking,
        driver::AttackSpeedLimitReason::Acceleration,
    };
    for (std::size_t i = 0; i < reasons.size(); ++i) {
        reason_counts[driver::attack_speed_limit_reason_name(reasons[i])] =
            metrics.limiting_reason_counts[i];
    }
    return {
        {"design", design_to_json(metrics.design)},
        {"track_length", metrics.track_length},
        {"base_profile_time", metrics.base_profile_time},
        {"attack_target_time", metrics.attack_target_time},
        {"attack_final_target_time", metrics.attack_final_target_time},
        {"attack_preview_min_target_time", metrics.attack_preview_min_target_time},
        {"base_mean_speed", metrics.base_mean_speed},
        {"attack_mean_speed", metrics.attack_mean_speed},
        {"attack_final_mean_speed", metrics.attack_final_mean_speed},
        {"attack_preview_min_mean_speed", metrics.attack_preview_min_mean_speed},
        {"base_min_speed", metrics.base_min_speed},
        {"base_max_speed", metrics.base_max_speed},
        {"attack_min_speed", metrics.attack_min_speed},
        {"attack_max_speed", metrics.attack_max_speed},
        {"limiting_reason_counts", reason_counts},
        {"sample_count", metrics.sample_count},
    };
}

void write_csv(const std::string& path, const std::vector<IdealSpeedMetrics>& rows) {
    if (path.empty()) {
        return;
    }
    const std::filesystem::path output_path(path);
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }
    std::ofstream ofs(output_path);
    if (!ofs) {
        throw std::runtime_error("Failed to write CSV: " + path);
    }
    ofs << "mass,CL_A,mu_y,mu_x,max_lon_accel,max_lon_decel,max_speed,"
        << "base_profile_time,attack_target_time,base_mean_speed,attack_mean_speed,"
        << "attack_final_target_time,attack_final_mean_speed,"
        << "attack_preview_min_target_time,attack_preview_min_mean_speed,"
        << "base_min_speed,base_max_speed,attack_min_speed,attack_max_speed,"
        << "profile_count,phase_count,curvature_count,steering_count,margin_count,"
        << "external_cap_count,braking_count,acceleration_count\n";
    ofs << std::setprecision(15);
    for (const auto& row : rows) {
        ofs << row.design.mass << ','
            << row.design.cl_a << ','
            << row.design.mu_y << ','
            << row.design.mu_x << ','
            << row.design.max_lon_accel << ','
            << row.design.max_lon_decel << ','
            << row.design.max_speed << ','
            << row.base_profile_time << ','
            << row.attack_target_time << ','
            << row.base_mean_speed << ','
            << row.attack_mean_speed << ','
            << row.attack_final_target_time << ','
            << row.attack_final_mean_speed << ','
            << row.attack_preview_min_target_time << ','
            << row.attack_preview_min_mean_speed << ','
            << row.base_min_speed << ','
            << row.base_max_speed << ','
            << row.attack_min_speed << ','
            << row.attack_max_speed;
        for (int count : row.limiting_reason_counts) {
            ofs << ',' << count;
        }
        ofs << '\n';
    }
}

void validate_design_constraints(const DesignPoint& design) {
    if (design.mass < 200.0) {
        throw std::runtime_error("mass grid contains value below 200 kg");
    }
    if (design.cl_a > 5.0) {
        throw std::runtime_error("CL_A grid contains value above 5");
    }
    if (design.max_speed <= 0.0
            || design.max_lon_accel <= 0.0
            || design.max_lon_decel <= 0.0
            || design.mu_x <= 0.1
            || design.mu_y <= 0.1) {
        throw std::runtime_error("design grid contains non-positive capability values");
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto cli = parse_args(argc, argv);
        const auto track = create_track(cli);
        const auto base_params = load_vehicle_params(cli);
        const auto external_caps = load_attack_speed_caps(cli.attack_speed_cap_file);
        const auto external_windows = load_attack_speed_windows(cli.attack_speed_cap_file);

        std::vector<IdealSpeedMetrics> rows;
        if (cli.optimize_grid) {
            for (double mass : cli.mass_list) {
                for (double cl_a : cli.cl_a_list) {
                    for (double mu_y : cli.mu_y_list) {
                        for (double mu_x : cli.mu_x_list) {
                            for (double ax : cli.max_lon_accel_list) {
                                for (double bx : cli.max_lon_decel_list) {
                                    for (double vmax : cli.max_speed_list) {
                                        const DesignPoint design{mass, cl_a, mu_y, mu_x, ax, bx, vmax};
                                        validate_design_constraints(design);
                                        rows.push_back(evaluate_design(
                                            track,
                                            base_params,
                                            cli,
                                            design,
                                            external_caps,
                                            external_windows));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } else {
            DesignPoint design{
                std::max(base_params.mass.m_total, 200.0),
                std::min(base_params.aero.CL_A, 5.0),
                base_params.tire.mu_y_max,
                base_params.tire.mu_x_max,
                cli.max_lon_accel,
                cli.max_lon_decel,
                cli.max_speed,
            };
            validate_design_constraints(design);
            rows.push_back(evaluate_design(
                track,
                base_params,
                cli,
                design,
                external_caps,
                external_windows));
        }

        auto best_it = std::min_element(
            rows.begin(),
            rows.end(),
            [](const auto& a, const auto& b) {
                return a.attack_final_target_time < b.attack_final_target_time;
            });
        if (best_it == rows.end()) {
            throw std::runtime_error("No design rows were evaluated");
        }
        write_csv(cli.csv_output, rows);

        json result = {
            {"track", cli.track_name},
            {"track_length", best_it->track_length},
            {"assumptions", {
                {"vehicle_state", "centerline_aligned_steering_reached"},
                {"optimization", cli.optimize_grid ? "finite_grid" : "single_design"},
                {"mass_min_kg", 200.0},
                {"CL_A_max", 5.0},
                {"note", "Unbounded non-mass/non-CL_A variables make the mathematical optimum approach zero time."},
            }},
            {"evaluated_count", rows.size()},
            {"best", metrics_to_json(*best_it)},
            {"baseline_or_single", metrics_to_json(rows.front())},
        };

        if (cli.output_json) {
            std::cout << "%%JSON_BEGIN%%\n" << result.dump(2) << "\n%%JSON_END%%\n";
        } else {
            std::cout << std::fixed << std::setprecision(3)
                      << "track_length=" << best_it->track_length
                      << " best_attack_final_target_time=" << best_it->attack_final_target_time
                      << " best_attack_target_time=" << best_it->attack_target_time
                      << " best_base_profile_time=" << best_it->base_profile_time
                      << " mass=" << best_it->design.mass
                      << " CL_A=" << best_it->design.cl_a
                      << " mu_y=" << best_it->design.mu_y
                      << " mu_x=" << best_it->design.mu_x
                      << " max_lon_accel=" << best_it->design.max_lon_accel
                      << " max_lon_decel=" << best_it->design.max_lon_decel
                      << " max_speed=" << best_it->design.max_speed
                      << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "ideal_speed_target failed: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
