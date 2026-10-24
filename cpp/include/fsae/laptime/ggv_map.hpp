#pragma once
/// @file ggv_map.hpp
/// @brief GGV(v, ay) 限界表と定常旋回表（Issue #23, #27）
///
/// オフライン生成器 `ggv_generator`（cpp/src/main_ggv_generator.cpp）が 14DOF プラントの
/// 準定常スイープから作る JSON を読み、runtime は 1D/2D の線形補間のみ行う。
///
/// 表の定義（全て絶対値、符号は呼び出し側で付ける）:
///   ay_max(v)            最大定常横加速度 [m/s^2]
///   ax_max(v, r)         最大前後加速度 [m/s^2]（駆動、エンジン/ギア/トラクション/空気抵抗込みの正味）
///   ax_min(v, r)         最大減速度 [m/s^2]（制動、空気抵抗込みの正味、正値）
///     r = |ay| / ay_max(v) ∈ [0, 1]（正規化横加速度）
///   delta_ss(v, |κ|)     定常旋回に必要なタイヤ舵角 [rad]（正値、κ の符号で反転）
///   beta_max(v)          限界旋回時の車体スリップ角 [rad]
///   r_max(v)             限界旋回時のヨーレート [rad/s]（= ay_max/v）

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fsae::laptime {

/// 単一の摩擦利用マージン（Issue #26）。横方向限界と制動限界に 1 回だけ掛ける。
/// 根拠: 旧 `safety_factor = 0.95`（速度に掛け）≈ 加速度で 0.90。
/// ponytail: 定数、ドライバーモデル追従誤差 RMS から m(v) を導出したら置き換える。
inline constexpr double kDefaultFrictionMargin = 0.90;

inline double interp1(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (xs.empty()) return 0.0;
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    const auto it = std::upper_bound(xs.begin(), xs.end(), x);
    const std::size_t i = static_cast<std::size_t>(it - xs.begin()) - 1;
    const double a = (x - xs[i]) / (xs[i + 1] - xs[i]);
    return ys[i] + a * (ys[i + 1] - ys[i]);
}

/// table[i][j] at (xs[i], ys[j]); 範囲外はクランプ
inline double interp2(const std::vector<double>& xs, const std::vector<double>& ys,
                      const std::vector<std::vector<double>>& table, double x, double y) {
    if (xs.empty() || ys.empty()) return 0.0;
    if (xs.size() == 1) return interp1(ys, table[0], y);
    x = std::clamp(x, xs.front(), xs.back());
    const std::size_t hi = std::min<std::size_t>(
        std::upper_bound(xs.begin(), xs.end(), x) - xs.begin(), xs.size() - 1);
    const std::size_t i = hi - 1;
    const double a = (x - xs[i]) / (xs[i + 1] - xs[i]);
    return (1.0 - a) * interp1(ys, table[i], y) + a * interp1(ys, table[i + 1], y);
}

struct GGVMap {
    std::vector<double> v;                      // [m/s] 昇順
    std::vector<double> ay_max;                 // [m/s^2] per v
    std::vector<double> r;                      // [-] 昇順 0..1
    std::vector<std::vector<double>> ax_max;    // [iv][ir] [m/s^2] ≥ 0
    std::vector<std::vector<double>> ax_min;    // [iv][ir] [m/s^2] ≥ 0（減速度の大きさ）

    std::vector<double> kappa;                  // [1/m] 昇順 ≥ 0（定常旋回表、任意）
    std::vector<std::vector<double>> delta_ss;  // [iv][ik] [rad]
    std::vector<double> beta_max;               // [rad] per v
    std::vector<double> r_max;                  // [rad/s] per v

    std::string source;                         // 生成元の説明（モデル、タイヤ種別）

    bool empty() const { return v.empty(); }
    bool has_steady_state() const { return !kappa.empty() && !delta_ss.empty(); }

    double ay_limit(double speed) const { return interp1(v, ay_max, speed); }

    double ax_accel(double speed, double ay) const {
        return interp2(v, r, ax_max, speed, lateral_ratio(speed, ay));
    }
    double ax_brake(double speed, double ay) const {
        return interp2(v, r, ax_min, speed, lateral_ratio(speed, ay));
    }
    double lateral_ratio(double speed, double ay) const {
        return std::clamp(std::abs(ay) / std::max(ay_limit(speed), 1e-6), 0.0, 1.0);
    }

    /// 定常旋回に必要なタイヤ舵角の大きさ [rad]（|κ|）
    double steer_angle(double abs_kappa, double speed) const {
        return interp2(v, kappa, delta_ss, speed, std::abs(abs_kappa));
    }
    double beta_limit(double speed) const { return interp1(v, beta_max, speed); }
    double yaw_rate_limit(double speed) const { return interp1(v, r_max, speed); }

    /// v^2|κ| ≤ scale·ay_max(v) を満たす最大速度（表上で二分法、プランナー前処理のみで使用）
    double corner_speed(double abs_kappa, double scale, double v_cap) const {
        if (abs_kappa <= 1e-9) return v_cap;
        auto feasible = [&](double s) { return s * s * abs_kappa <= scale * ay_limit(s); };
        if (feasible(v_cap)) return v_cap;
        double lo = 0.0, hi = v_cap;
        for (int it = 0; it < 50; ++it) {
            const double mid = 0.5 * (lo + hi);
            (feasible(mid) ? lo : hi) = mid;
        }
        return lo;
    }

    nlohmann::json to_json() const {
        return {
            {"schema", "fsae.ggv.v1"},
            {"source", source},
            {"v_mps", v},
            {"ay_max_mps2", ay_max},
            {"ay_ratio", r},
            {"ax_max_mps2", ax_max},
            {"ax_min_mps2", ax_min},
            {"kappa_1pm", kappa},
            {"delta_ss_rad", delta_ss},
            {"beta_max_rad", beta_max},
            {"r_max_radps", r_max},
        };
    }

    static GGVMap from_json(const nlohmann::json& j) {
        if (j.value("schema", std::string()) != "fsae.ggv.v1") {
            throw std::runtime_error("GGV JSON: schema must be 'fsae.ggv.v1'");
        }
        GGVMap g;
        g.source = j.value("source", std::string());
        g.v = j.at("v_mps").get<std::vector<double>>();
        g.ay_max = j.at("ay_max_mps2").get<std::vector<double>>();
        g.r = j.at("ay_ratio").get<std::vector<double>>();
        g.ax_max = j.at("ax_max_mps2").get<std::vector<std::vector<double>>>();
        g.ax_min = j.at("ax_min_mps2").get<std::vector<std::vector<double>>>();
        if (j.contains("kappa_1pm")) {
            g.kappa = j.at("kappa_1pm").get<std::vector<double>>();
            g.delta_ss = j.at("delta_ss_rad").get<std::vector<std::vector<double>>>();
            g.beta_max = j.at("beta_max_rad").get<std::vector<double>>();
            g.r_max = j.at("r_max_radps").get<std::vector<double>>();
        }
        g.validate();
        return g;
    }

    static GGVMap load(const std::string& path) {
        std::ifstream ifs(path);
        if (!ifs) throw std::runtime_error("GGV JSON: cannot open " + path);
        return from_json(nlohmann::json::parse(ifs));
    }

    void save(const std::string& path) const {
        std::ofstream ofs(path);
        if (!ofs) throw std::runtime_error("GGV JSON: cannot write " + path);
        ofs << to_json().dump(2) << "\n";
    }

    void validate() const {
        auto ascending = [](const std::vector<double>& xs) {
            return std::adjacent_find(xs.begin(), xs.end(),
                                      [](double a, double b) { return !(a < b); }) == xs.end();
        };
        auto grid_ok = [](const std::vector<std::vector<double>>& t, std::size_t n, std::size_t m) {
            return t.size() == n && std::all_of(t.begin(), t.end(),
                [m](const std::vector<double>& row) { return row.size() == m; });
        };
        if (v.size() < 2 || !ascending(v) || ay_max.size() != v.size() ||
            r.size() < 2 || !ascending(r) ||
            !grid_ok(ax_max, v.size(), r.size()) || !grid_ok(ax_min, v.size(), r.size())) {
            throw std::runtime_error("GGV JSON: inconsistent v / ay_ratio / ax tables");
        }
        if (!kappa.empty() &&
            (!ascending(kappa) || !grid_ok(delta_ss, v.size(), kappa.size()) ||
             beta_max.size() != v.size() || r_max.size() != v.size())) {
            throw std::runtime_error("GGV JSON: inconsistent steady-state tables");
        }
    }
};

} // namespace fsae::laptime
