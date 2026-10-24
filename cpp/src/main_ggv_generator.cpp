/// @file main_ggv_generator.cpp
/// @brief Offline GGV / steady-state table generator (Issues #23, #27).
///
/// Drives Vehicle14DOF_New (RK4, 1 ms) through quasi-steady manoeuvres:
///   1. constant-speed slow ramp-steer at each v  → ay_max(v), δ_ss(v, κ), β_max(v), r_max(v)
///   2. from steady cornering at r·ay_max, a throttle ramp (best gear by engine torque × rpm,
///      redline-limited) / brake ramp, holding the steer → peak ax_max(v, r) / ax_min(v, r)
/// Engine map, gear ratios, LSD, aero map at speed and the 4-wheel load transfer all come from
/// the plant itself; nothing here re-implements vehicle physics.
///
/// Usage: ggv_generator [--params vehicle.json] [--out ggv.json] [--tire pinn|tmeasy] [--quick]
///
/// The tire model is PINN by default. This target is compiled with
/// FSAE_TEST_TIRE_MODEL_TYPE so that, when the PINN artifact is missing, it falls back to the
/// same 14DOF plant with TMeasy; the fallback is logged and recorded in the JSON "source".

#include <fsae/driver/gear_selection.hpp>
#include <fsae/laptime/ggv_map.hpp>
#include <fsae/vehicle/vehicle_14dof_new.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace fsae;

namespace {

constexpr double kDt = 0.001;

struct Rig {
    vehicle::VehicleParams params;
    std::unique_ptr<vehicle::Vehicle14DOF_New> car;
    double redline_rpm = 14500.0;

    int best_gear() const {
        // Max wheel torque ∝ T_engine(rpm_g) · ratio_g ∝ T_engine(rpm_g) · rpm_g (same wheel ω).
        const int n_gears = std::max<int>(1, static_cast<int>(params.powertrain.gear_ratios.size()));
        int best = 1;
        double best_score = -1.0;
        for (int g = 1; g <= n_gears; ++g) {
            const double rpm = car->get_engine_rpm(g);
            if (rpm > redline_rpm && g < n_gears) continue;
            const double score = car->get_engine_torque(g, 1.0) * std::max(rpm, 1.0);
            if (score > best_score) {
                best_score = score;
                best = g;
            }
        }
        return best;
    }

    /// Start straight at v with speed hold for t_settle.
    void start(double v, double t_settle) {
        car->reset();
        car->set_initial_velocity(v);
        for (double t = 0.0; t < t_settle; t += kDt) hold(v, 0.0);
    }

    /// One step holding speed v_ref (P throttle/brake) at tire steer delta.
    void hold(double v_ref, double delta) {
        const auto& s = car->state();
        vehicle::DriverInput in;
        in.steering = delta;
        const double e = v_ref - s.vx;
        in.throttle = std::clamp(0.15 + 0.6 * e, 0.0, 1.0);
        in.brake = std::clamp(-0.3 * e, 0.0, 1.0);
        if (in.brake > 0.0) in.throttle = 0.0;
        in.gear = driver::select_gear_from_vehicle_params(std::max(s.vx, 0.0), params);
        car->step(kDt, in);
    }
};

struct RampResult {
    double ay_max = 0.0;
    double beta_at_max = 0.0;
    std::vector<double> kappa, delta, ay;  // monotone-in-κ samples up to the limit
    bool friction_limited = false;
};

/// Constant-speed slow ramp steer (quasi-steady), stops after the ay peak or on spin.
RampResult ramp_steer(Rig& rig, double v) {
    RampResult out;
    rig.start(v, 0.6);
    const double delta_end = std::abs(rig.params.steering.max_angle);
    const double rate = 0.04;  // [rad/s] slow enough that yaw/sideslip lag is negligible
    double delta = 0.0, ay_f = 0.0, vy_prev = rig.car->state().vy;
    int steps_after_peak = 0;
    for (int k = 0; delta < delta_end; ++k) {
        delta = rate * k * kDt;
        rig.hold(v, delta);
        const auto& s = rig.car->state();
        const double ay = s.vx * s.r + (s.vy - vy_prev) / kDt;  // body lateral accel
        vy_prev = s.vy;
        ay_f += 0.02 * (ay - ay_f);  // 50 ms low-pass
        const double beta = std::atan2(s.vy, std::max(s.vx, 0.1));
        if (!std::isfinite(ay) || std::abs(beta) > 0.30 || s.vx < 0.5 * v) {
            out.friction_limited = true;  // spun / lost speed: the limit was reached
            break;
        }
        if (std::abs(ay_f) > out.ay_max) {
            out.ay_max = std::abs(ay_f);
            out.beta_at_max = std::abs(beta);
            steps_after_peak = 0;
        } else if (++steps_after_peak > 1500 && std::abs(ay_f) < 0.97 * out.ay_max) {
            out.friction_limited = true;  // ay fell after a peak: past the friction limit
            break;
        }
        if (k % 20 == 0 && std::abs(ay_f) >= out.ay_max - 1e-9) {
            const double kap = std::abs(s.r) / std::max(s.vx, 0.1);
            if (out.kappa.empty() || kap > out.kappa.back() + 1e-5) {
                out.kappa.push_back(kap);
                out.delta.push_back(delta);
                out.ay.push_back(std::abs(ay_f));
            }
        }
    }
    return out;
}

/// From steady cornering at tire steer delta and speed v, ramp throttle (or brake) and
/// return the peak sustained longitudinal acceleration magnitude (100 ms average).
double longitudinal_peak(Rig& rig, double v, double delta, bool accel) {
    rig.start(v, 0.4);
    for (double t = 0.0; t < 1.2; t += kDt) rig.hold(v, delta);  // settle the corner
    const double t_ramp = 0.20, t_hold = 0.25;
    std::vector<double> vx_hist;
    double peak = 0.0;
    const int gear = accel ? rig.best_gear() : 1;
    for (double t = 0.0; t < t_ramp + t_hold; t += kDt) {
        vehicle::DriverInput in;
        in.steering = delta;
        const double u = std::min(1.0, t / t_ramp);
        if (accel) {
            in.throttle = u;
            in.gear = gear;
        } else {
            in.brake = u;
            in.gear = driver::select_gear_from_vehicle_params(rig.car->state().vx, rig.params);
        }
        rig.car->step(kDt, in);
        vx_hist.push_back(rig.car->state().vx);
        const std::size_t n = vx_hist.size();
        if (n > 100) {
            const double ax = (vx_hist[n - 1] - vx_hist[n - 101]) / (100 * kDt);
            if (std::isfinite(ax)) peak = std::max(peak, accel ? ax : -ax);
        }
    }
    return std::max(peak, 0.0);
}

}  // namespace

int main(int argc, char** argv) {
    std::string params_file, out_file = "output/ggv/ggv.json", tire = "pinn";
    bool quick = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--params" && i + 1 < argc) params_file = argv[++i];
        else if (a == "--out" && i + 1 < argc) out_file = argv[++i];
        else if (a == "--tire" && i + 1 < argc) tire = argv[++i];
        else if (a == "--quick") quick = true;
        else {
            std::cerr << "usage: ggv_generator [--params f.json] [--out ggv.json] "
                         "[--tire pinn|tmeasy] [--quick]" << std::endl;
            return 2;
        }
    }

    Rig rig;
    rig.params = params_file.empty() ? vehicle::VehicleParams::default_fsae()
                                     : vehicle::VehicleParams::from_json(params_file);
    rig.params.steering.torque_mode = false;  // angle-input sweep
    if (!rig.params.powertrain.engine_rpm_points.empty()) {
        rig.redline_rpm = rig.params.powertrain.engine_rpm_points.back();
    }
    std::string source = "Vehicle14DOF_New quasi-steady sweep, tire=";
    rig.params.tire.model_type = tire;
    try {
        rig.car = std::make_unique<vehicle::Vehicle14DOF_New>(rig.params);
        source += tire;
    } catch (const std::exception& e) {
        if (tire == "tmeasy") throw;
        std::cerr << "WARNING: PINN tire model unavailable (" << e.what()
                  << ").\nWARNING: falling back to the same 14DOF plant with TMeasy tires; "
                     "the GGV is NOT PINN-derived." << std::endl;
        rig.params.tire.model_type = "tmeasy";
        rig.car = std::make_unique<vehicle::Vehicle14DOF_New>(rig.params);
        source += "tmeasy (fallback: PINN artifact missing)";
    }
    if (!params_file.empty()) source += ", params=" + params_file;

    laptime::GGVMap g;
    g.source = source;
    g.v = quick ? std::vector<double>{4.0, 10.0, 18.0, 28.0}
                : std::vector<double>{3.0, 5.0, 7.0, 9.0, 11.0, 13.0, 15.0, 18.0, 21.0, 25.0, 30.0, 35.0};
    g.r = quick ? std::vector<double>{0.0, 0.6, 1.0}
                : std::vector<double>{0.0, 0.25, 0.5, 0.7, 0.85, 1.0};
    g.kappa = {0.0, 0.01, 0.02, 0.04, 0.06, 0.08, 0.10, 0.13, 0.16, 0.20, 0.25};

    const double m = rig.params.mass.m_total;
    const double L = rig.params.mass.wheelbase;
    std::vector<RampResult> ramps;
    for (double v : g.v) {
        ramps.push_back(ramp_steer(rig, v));
        const auto& rr = ramps.back();
        std::cout << "v=" << v << " ay_max=" << rr.ay_max << " beta=" << rr.beta_at_max
                  << (rr.friction_limited ? "" : " (steer-limited)") << std::endl;
    }
    // Low speeds where max steer is reached before the friction peak: scale the lowest
    // friction-limited ay_max by total vertical load (m g + downforce) ratio.
    // ponytail: load-ratio extrapolation, replace with a constant-radius speed sweep if needed.
    int ref = -1;
    for (std::size_t i = 0; i < ramps.size(); ++i) {
        if (ramps[i].friction_limited) { ref = static_cast<int>(i); break; }
    }
    if (ref < 0) {
        std::cerr << "ERROR: no friction-limited ramp found; cannot build GGV" << std::endl;
        return 1;
    }
    const double rho = rig.params.aero.rho > 0.1 ? rig.params.aero.rho : 1.225;
    auto vertical = [&](double v) { return m * 9.81 + 0.5 * rho * std::abs(rig.params.aero.CL_A) * v * v; };
    for (std::size_t i = 0; i < ramps.size(); ++i) {
        double ay = ramps[i].ay_max;
        if (!ramps[i].friction_limited && static_cast<int>(i) < ref) {
            ay = ramps[ref].ay_max * vertical(g.v[i]) / vertical(g.v[ref]);
        }
        g.ay_max.push_back(ay);
        g.beta_max.push_back(ramps[i].beta_at_max > 0.0 ? ramps[i].beta_at_max : ramps[ref].beta_at_max);
        g.r_max.push_back(ay / g.v[i]);

        // δ_ss(v, κ): ramp samples, kinematic slope beyond the reachable range
        std::vector<double> row;
        const auto& rr = ramps[i];
        for (double k : g.kappa) {
            if (rr.kappa.size() >= 2 && k <= rr.kappa.back()) {
                row.push_back(laptime::interp1(rr.kappa, rr.delta, k));
            } else if (rr.kappa.size() >= 2) {
                row.push_back(rr.delta.back() + L * (k - rr.kappa.back()));
            } else {
                row.push_back(L * k);
            }
        }
        // κ=0 → δ=0 exactly; keep monotone
        row[0] = 0.0;
        for (std::size_t j = 1; j < row.size(); ++j) row[j] = std::max(row[j], row[j - 1]);
        g.delta_ss.push_back(row);
    }

    for (std::size_t i = 0; i < g.v.size(); ++i) {
        const double v = g.v[i];
        const auto& rr = ramps[i];
        std::vector<double> acc_row, brk_row;
        for (double rk : g.r) {
            if (rk >= 1.0 - 1e-9) {  // at the steady limit there is no longitudinal reserve
                acc_row.push_back(0.0);
                brk_row.push_back(0.0);
                continue;
            }
            const double delta = (rk <= 1e-9 || rr.ay.size() < 2)
                ? 0.0
                : laptime::interp1(rr.ay, rr.delta, rk * g.ay_max[i]);
            acc_row.push_back(longitudinal_peak(rig, v, delta, true));
            brk_row.push_back(longitudinal_peak(rig, v, delta, false));
        }
        for (std::size_t j = 1; j < acc_row.size(); ++j) {  // non-increasing in r
            acc_row[j] = std::min(acc_row[j], acc_row[j - 1]);
            brk_row[j] = std::min(brk_row[j], brk_row[j - 1]);
        }
        std::cout << "v=" << v << " ax_max(r=0)=" << acc_row[0] << " ax_min(r=0)=" << brk_row[0] << std::endl;
        g.ax_max.push_back(acc_row);
        g.ax_min.push_back(brk_row);
    }

    g.validate();
    const auto dir = std::filesystem::path(out_file).parent_path();
    if (!dir.empty()) std::filesystem::create_directories(dir);
    g.save(out_file);
    std::cout << "GGV written: " << out_file << " (" << source << ")" << std::endl;
    return 0;
}
