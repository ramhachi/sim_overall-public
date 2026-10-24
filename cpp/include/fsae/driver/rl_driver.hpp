#pragma once
/// @file rl_driver.hpp
/// @brief 強化学習ドライバーモデル（C++推論クラス）
///
/// MATLABで学習したSAC Actorネットワークの重みをテキストファイルから読み込み、
/// Forward Passのみで制御入力を計算する。
/// NNTextTireModel (nn_lookup_tire.hpp) と同じエクスポートパターンを使用。
///
/// 行動空間: 2D [steer_command(-1~1), accel(-1~1)] or 3D [steer, throttle, brake]
///   - 2D: accel > 0 → throttle, accel < 0 → brake（推奨）
///   - 3D: 従来形式（後方互換）
/// ACTION_SCALE行のNから自動判定。

#include "rl_control_safety.hpp"
#include "gear_selection.hpp"
#include "../vehicle/vehicle_14dof_new.hpp"
#include "../vehicle/vehicle_params.hpp"
#include "../vehicle/vehicle_state.hpp"
#include "../track/track_model.hpp"
#include "../track/frenet.hpp"
#include <vector>
#include <array>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <algorithm>
#include <iostream>

namespace fsae::driver {

/// RL Actorネットワークの全結合層（LayerNorm対応）
struct RLFCLayer {
    std::string name;
    std::vector<std::vector<double>> weights;  // [n_out][n_in]
    std::vector<double> biases;                // [n_out]
    std::vector<double> ln_gamma;              // LayerNorm scale（空なら適用しない）
    std::vector<double> ln_beta;               // LayerNorm offset
    int n_in = 0;
    int n_out = 0;

    std::vector<double> forward(const std::vector<double>& input) const {
        std::vector<double> output(n_out, 0.0);
        for (int i = 0; i < n_out; ++i) {
            output[i] = biases[i];
            for (int j = 0; j < n_in; ++j) {
                output[i] += weights[i][j] * input[j];
            }
        }
        if (!ln_gamma.empty()) {
            double mean = 0.0;
            for (auto v : output) mean += v;
            mean /= output.size();
            double var = 0.0;
            for (auto v : output) var += (v - mean) * (v - mean);
            var /= output.size();
            const double inv_std = 1.0 / std::sqrt(var + 1e-5);
            for (int i = 0; i < n_out; ++i) {
                output[i] = ln_gamma[i] * (output[i] - mean) * inv_std + ln_beta[i];
            }
        }
        return output;
    }
};

/// 強化学習ドライバー（推論専用）
class RLDriver {
public:
    static constexpr int SERVER_OBS_DIM = 34;
    static constexpr int N_VEHICLE_PARAMS = 11;
    static constexpr int OBS_DIM = 45;  // 34 (server) + 11 (vehicle params)

    RLDriver() = default;

    /// 車両パラメータ設定（build_observationで使用）
    void set_vehicle_params(const vehicle::VehicleParams& params) {
        a_ = params.mass.a;
        b_ = params.mass.b;
        r_tire_ = params.tire.radius;
        expected_steer_scale_ = rl_steering_command_limit(params);
        vehicle_params_ = params;
        has_vehicle_params_ = true;

        // Store raw vehicle params appended after the server observations.
        vp_raw_[0] = params.mass.m_total;
        vp_raw_[1] = params.aero.CL_A;
        vp_raw_[2] = params.aero.CD_A;
        vp_raw_[3] = params.tire.mu_y_max;
        vp_raw_[4] = params.tire.mu_x_max;
        vp_raw_[5] = params.suspension.k_spring[0];  // front spring
        vp_raw_[6] = params.suspension.k_arb_f;
        vp_raw_[7] = params.steering.stiffness;
        vp_raw_[8] = params.suspension.K_chassis;
        vp_raw_[9] = params.suspension.C_chassis;
        vp_raw_[10] = params.suspension.I_twist;
        validate_steering_scale_contract();
    }

    void load(const std::string& dir) {
        load_weights(dir + "/rl_actor_weights.txt");
        load_normalization(dir + "/rl_normalization_params.txt");
        loaded_ = true;
        validate_steering_scale_contract();
        std::cout << "RLDriver: loaded " << layers_.size() << " mean-path layers"
                  << ", act_dim=" << act_dim_ << " from " << dir << std::endl;
    }

    bool is_loaded() const { return loaded_; }
    int act_dim() const { return act_dim_; }
    void update_prev_action(double steer, double throttle, double brake) {
        prev_steer_ = steer;
        prev_throttle_ = throttle;
        prev_brake_ = brake;
    }
    int obs_dim() const { return obs_input_dim_; }

    /// Override the torque-command slew limit used by the inference-side filter.
    /// Negative means "use vehicle_params.steering.T_driver_rate_max"; zero disables
    /// the torque-mode slew limiter, matching the plant option semantics.
    void set_steering_torque_rate_limit_override(double rate_limit) {
        steering_torque_rate_limit_override_ = rate_limit;
    }

    /// 正規化済み観測から行動を推論
    std::vector<double> infer_normalized_action(
        const std::vector<double>& obs_norm) const
    {
        if (!loaded_ || layers_.empty()) {
            throw std::runtime_error("RLDriver not loaded");
        }

        std::vector<double> x(obs_norm.begin(), obs_norm.end());
        for (size_t i = 0; i + 1 < layers_.size(); ++i) {
            x = layers_[i].forward(x);
            for (auto& v : x) v = std::max(0.0, v); // ReLU
        }
        x = layers_.back().forward(x);
        for (auto& v : x) v = std::tanh(v);
        return x;
    }

    vehicle::DriverInput compute_control(
        const vehicle::State14DOF& state,
        const track::CurvatureBasedTrack& track,
        track::FrenetConverter& converter,
        double /*time*/)
    {
        if (!loaded_) {
            return vehicle::DriverInput{};
        }

        // 1. 観測ベクトル構築 (server obs)
        std::vector<double> raw_obs(SERVER_OBS_DIM);
        build_observation(state, track, converter, raw_obs);

        // 2. 正規化 [-1, 1] — server obs
        std::vector<double> obs_norm(obs_input_dim_);
        for (int i = 0; i < SERVER_OBS_DIM; ++i) {
            double range = obs_max_[i] - obs_min_[i];
            if (range < 1e-10) range = 1.0;
            obs_norm[i] = 2.0 * (raw_obs[i] - obs_min_[i]) / range - 1.0;
            obs_norm[i] = std::clamp(obs_norm[i], -1.0, 1.0);
        }
        // Append normalized vehicle params when the exported actor expects them.
        for (int i = SERVER_OBS_DIM; i < obs_input_dim_; ++i) {
            int vehicle_idx = i - SERVER_OBS_DIM;
            double range = obs_max_[i] - obs_min_[i];
            if (range < 1e-10) range = 1.0;
            obs_norm[i] = 2.0 * (vp_raw_[vehicle_idx] - obs_min_[i]) / range - 1.0;
            obs_norm[i] = std::clamp(obs_norm[i], -1.0, 1.0);
        }

        // 3. Forward Pass
        const auto action = infer_normalized_action(obs_norm);

        // 4. 行動をデコード
        vehicle::DriverInput input;
        input.steering = action[0] * action_scale_[0];
        const double torque_rate_limit =
            (steering_torque_rate_limit_override_ >= 0.0)
                ? steering_torque_rate_limit_override_
                : vehicle_params_.steering.T_driver_rate_max;
        input.steering = smooth_rl_steering_command(
            input.steering,
            prev_steer_,
            action_scale_[0],
            vehicle_params_.steering.torque_mode,
            0.020,
            torque_rate_limit);

        if (act_dim_ == 2) {
            // 2D: [steer, accel] — accel>0→throttle, accel<0→brake
            double accel = std::clamp(action[1], -1.0, 1.0);
            if (accel >= 0.0) {
                input.throttle = accel * action_scale_[1];
                input.brake = 0.0;
            } else {
                input.throttle = 0.0;
                input.brake = -accel * action_scale_[1];
            }
        } else {
            // 3D: [steer, throttle, brake]
            input.throttle = std::clamp(action[1] * action_scale_[1], 0.0, 1.0);
            input.brake = std::clamp(action[2] * action_scale_[2], 0.0, 1.0);
        }
        input.gear = select_gear(state.vx);

        // prev_action_ は物理値で保存（学習時と一致させる）
        prev_steer_ = input.steering;
        prev_throttle_ = input.throttle;
        prev_brake_ = input.brake;

        return input;
    }

private:
    std::vector<RLFCLayer> layers_;  // mean-path のみ（fcStdは除外）
    std::vector<double> obs_min_;
    std::vector<double> obs_max_;
    std::vector<double> action_scale_;
    int act_dim_ = 2;
    int obs_input_dim_ = OBS_DIM;

    // prev_action は物理値で保存（学習サーバーと同じ）
    double prev_steer_ = 0.0;
    double prev_throttle_ = 0.0;
    double prev_brake_ = 0.0;

    // 車両パラメータ（デフォルトは VehicleParams と一致）
    double a_ = 0.73;
    double b_ = 0.82;
    double r_tire_ = 0.228;

    // Vehicle params appended after the server obs (raw values before normalization)
    std::array<double, N_VEHICLE_PARAMS> vp_raw_ = {
        282.0, 2.8, 0.95, 2.0, 1.6, 25000.0, 500.0, 5000.0, 2500.0, 5.0, 1.5
    };
    double expected_steer_scale_ = 0.0;
    bool has_vehicle_params_ = false;
    vehicle::VehicleParams vehicle_params_ = vehicle::VehicleParams::default_fsae();
    double steering_torque_rate_limit_override_ = -1.0;

    bool loaded_ = false;

    void build_observation(
        const vehicle::State14DOF& st,
        const track::CurvatureBasedTrack& track,
        track::FrenetConverter& converter,
        std::vector<double>& obs)
    {
        const double map_x = st.y;
        const double map_y = st.x;
        const double map_theta = M_PI / 2.0 - st.psi;
        auto frenet = converter.global_to_frenet(map_x, map_y, map_theta);
        auto tp = track.get_point(frenet.s);

        // ======== 車両状態（19次元）========
        obs[0] = st.vx;
        obs[1] = st.vy;
        obs[2] = st.r;
        // 加速度（車体フレーム）— 学習サーバーと一致
        obs[3] = -st.vy * st.r;  // 縦加速度近似 (ax_body)
        obs[4] = st.vx * st.r;   // 横加速度近似 (ay_body)
        obs[5] = st.phi;

        // 前回の制御入力（物理値）— 学習時は C++ サーバーがこの形式で送信
        obs[6] = prev_steer_;
        obs[7] = prev_throttle_;
        obs[8] = prev_brake_;

        for (int i = 0; i < 4; ++i) {
            double vw = st.omega[i] * r_tire_;
            double vref = std::max(std::abs(st.vx), 0.5);
            obs[9 + i] = (vw - st.vx) / vref;
        }

        double alpha_f = 0.0, alpha_r = 0.0;
        if (std::abs(st.vx) > 0.5) {
            alpha_f = std::atan2(st.vy + a_ * st.r, std::max(st.vx, 0.5));
            alpha_r = std::atan2(st.vy - b_ * st.r, std::max(st.vx, 0.5));
        }
        obs[13] = alpha_f;
        obs[14] = alpha_r;

        // ======== トラック情報（15次元）========
        obs[15] = frenet.n;
        obs[16] = frenet.d_heading;
        obs[17] = track.get_curvature(frenet.s);

        double v_ratio = std::clamp(st.vx / 30.0, 0.0, 1.0);
        double la_d[5] = {10, 20, 40, 60, 80};
        for (int i = 0; i < 5; ++i) {
            double d = la_d[i] * (0.3 + 0.7 * v_ratio);
            double s_ahead = frenet.s + d;
            if (s_ahead > track.length()) s_ahead -= track.length();
            obs[18 + i] = track.get_curvature(s_ahead);
        }

        obs[23] = tp.width_left - frenet.n;
        obs[24] = tp.width_right + frenet.n;
        obs[25] = std::fmod(std::max(frenet.s, 0.0), track.length()) / track.length();

        double w_d[4] = {10, 20, 40, 60};
        for (int i = 0; i < 4; ++i) {
            double s_ahead = frenet.s + w_d[i];
            if (s_ahead > track.length()) s_ahead -= track.length();
            auto tp_a = track.get_point(s_ahead);
            obs[26 + i] = tp_a.width_left + tp_a.width_right;
        }

        obs[30] = st.phi_twist;
        obs[31] = st.dphi_twist;
        obs[32] = st.delta_sw;
        obs[33] = st.ddelta_sw;
    }

    int select_gear(double speed) const {
        return select_gear_from_vehicle_params(speed, vehicle_params_);
    }

    void load_weights(const std::string& path) {
        std::ifstream ifs(path);
        if (!ifs.is_open()) throw std::runtime_error("Cannot open: " + path);

        std::vector<RLFCLayer> all_layers;
        std::string line;
        std::string current_name;

        while (std::getline(ifs, line)) {
            if (line.substr(0, 5) == "LAYER") {
                current_name = line.substr(6);
                // 末尾の空白を除去
                while (!current_name.empty() && (current_name.back() == ' ' || current_name.back() == '\r'))
                    current_name.pop_back();
            } else if (line.substr(0, 7) == "WEIGHTS") {
                all_layers.emplace_back();
                auto* layer = &all_layers.back();
                layer->name = current_name;
                std::istringstream iss(line.substr(8));
                iss >> layer->n_out >> layer->n_in;

                layer->weights.resize(layer->n_out, std::vector<double>(layer->n_in));
                for (int i = 0; i < layer->n_out; ++i)
                    for (int j = 0; j < layer->n_in; ++j)
                        ifs >> layer->weights[i][j];

                std::getline(ifs, line); // 余分な改行
                std::getline(ifs, line); // "BIAS N"
                int n_bias = 0;
                std::istringstream biss(line.substr(5));
                biss >> n_bias;
                layer->biases.resize(n_bias);
                for (int i = 0; i < n_bias; ++i) ifs >> layer->biases[i];
            } else if (line.substr(0, 8) == "LN_GAMMA" && !all_layers.empty()) {
                auto* layer = &all_layers.back();
                int n = 0;
                std::istringstream iss(line.substr(9));
                iss >> n;
                layer->ln_gamma.resize(n);
                for (int i = 0; i < n; ++i) ifs >> layer->ln_gamma[i];
            } else if (line.substr(0, 7) == "LN_BETA" && !all_layers.empty()) {
                auto* layer = &all_layers.back();
                int n = 0;
                std::istringstream iss(line.substr(8));
                iss >> n;
                layer->ln_beta.resize(n);
                for (int i = 0; i < n; ++i) ifs >> layer->ln_beta[i];
            }
        }

        // fcStd（標準偏差パス）を除外して mean-path のみ保持
        layers_.clear();
        for (auto& l : all_layers) {
            if (l.name.find("Std") != std::string::npos ||
                l.name.find("std") != std::string::npos) {
                std::cout << "RLDriver: skipping std layer '" << l.name << "'" << std::endl;
                continue;
            }
            layers_.push_back(std::move(l));
        }
    }

    void load_normalization(const std::string& path) {
        std::ifstream ifs(path);
        if (!ifs.is_open()) throw std::runtime_error("Cannot open: " + path);

        std::string line;
        std::getline(ifs, line); // "OBS_BOUNDS N"
        // Parse N from header (supports both 30 and 37)
        int n_obs = OBS_DIM;
        {
            std::istringstream hdr(line);
            std::string tag;
            hdr >> tag >> n_obs;
        }
        obs_input_dim_ = std::clamp(n_obs, SERVER_OBS_DIM, OBS_DIM);

        obs_min_.resize(OBS_DIM, 0.0);
        obs_max_.resize(OBS_DIM, 1.0);
        for (int i = 0; i < obs_input_dim_ && i < OBS_DIM; ++i) {
            ifs >> obs_min_[i] >> obs_max_[i];
        }
        // If file has fewer bounds than OBS_DIM, remaining use defaults above

        std::getline(ifs, line); // 余分な改行
        std::getline(ifs, line); // "ACTION_SCALE N"
        std::istringstream iss(line.substr(13));
        iss >> act_dim_;
        action_scale_.resize(act_dim_);
        for (int i = 0; i < act_dim_; ++i) {
            ifs >> action_scale_[i];
        }
    }

    void validate_steering_scale_contract() const {
        if (!loaded_ || !has_vehicle_params_ || action_scale_.empty()) {
            return;
        }

        const double tol = std::max(1.0, std::abs(expected_steer_scale_)) * 1e-6;
        if (std::abs(action_scale_[0] - expected_steer_scale_) > tol) {
            throw std::runtime_error(
                "RL actor steering scale does not match the current vehicle steering contract");
        }
    }
};

} // namespace fsae::driver
