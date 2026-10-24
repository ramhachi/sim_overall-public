#pragma once
/// @file aero_map_4d.hpp
/// @brief 4次元エアロマップ（RHf, RHr, roll, yaw）
///
/// 本番経路は「設計目標値（target aero）から作る解析マップ」（coefficient モード）。
/// CL·A, CD·A, 空力バランス, h_aero, RH/roll/yaw 感度, DRS ON 目標を AeroMapParams で明示し、
/// set_coefficient_defaults() で格子を再生成する。これらは設計目標であり実測ではない。
/// 実測（CFD/風洞）マップ JSON を load_from_json() で与えた場合はそれが目標値を上書きする。
///
/// 入力変数:
///   - RH_f: フロントライドハイト [mm]
///   - RH_r: リアライドハイト [mm]
///   - roll: ロール角 [rad]
///   - yaw:  エアロヨー角 [rad]
/// 域外入力は格子端にクランプし、out_of_domain ビット（bit0 RHf, bit1 RHr, bit2 roll, bit3 yaw）と
/// out_of_domain_count() で通知する。
/// 座標: 力 Fx, Fy は SAE 車体系（X 前, Y 右）、Fz は揚力正（負 = ダウンフォース）。
/// モーメントは CG 回り（SAE: Mx ロール, My ピッチ ノーズアップ正, Mz ヨー）。
///
/// ponytail: 速度軸（Re 依存）なし、純 V² スケール。必要になったら 5 軸目 V か dCL/dlnV 補正配列を追加。

#include <vector>
#include <array>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <fstream>
#include <limits>
#include <initializer_list>
#include <string>
#include <tuple>
#include <utility>

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

namespace fsae::physics::aero {

/// 空力係数出力
template<typename Scalar = double>
struct AeroOutput {
    Scalar CL = 0;      // リフト係数（負=ダウンフォース）
    Scalar CD = 0;      // ドラッグ係数
    Scalar CY = 0;      // サイドフォース係数
    Scalar Cm = 0;      // ピッチングモーメント係数（鉛直力分のみ）
    Scalar CoP_x = 0;   // 鉛直力の圧力中心 X [m from rear axle]（前輪配分 = CoP_x / L_ref）
    Scalar CoP_y_x = 0; // 横力の圧力中心 X [m from rear axle]
    std::uint8_t out_of_domain = 0;  // 域外ビット（bit0 RHf, bit1 RHr, bit2 roll, bit3 yaw）
};

/// 空力・モーメント出力
template<typename Scalar = double>
struct AeroForces {
    Scalar Fx = 0;      // ドラッグ [N] (負=後方)
    Scalar Fy = 0;      // サイドフォース [N]
    Scalar Fz = 0;      // リフト [N] (負=ダウンフォース)
    Scalar Mx = 0;      // ロールモーメント [Nm]（CG 回り）
    Scalar My = 0;      // ピッチモーメント [Nm]（CG 回り）
    Scalar Mz = 0;      // ヨーモーメント [Nm]（CG 回り）

    // 前後輪へのダウンフォース配分（鉛直 CoP から導出）
    Scalar Fz_front = 0;
    Scalar Fz_rear = 0;
    std::uint8_t out_of_domain = 0;
};

/// 4Dエアロマップパラメータ
template<typename Scalar = double>
struct AeroMapParams {
    Scalar rho = 1.225;             // 空気密度 [kg/m³]
    Scalar A_ref = 1.0;             // 参照面積 [m²]（1 のとき CL/CD は CL·A/CD·A）
    Scalar L_ref = 1.6;             // 参照長さ = ホイールベース [m]
    Scalar x_cg = 0.8;              // CG 位置 [m from rear axle]
    Scalar h_cg = 0.28;             // CG 高さ [m above ground]
    Scalar reference_speed = 15.0;  // 参照速度 [m/s]
    Scalar reference_density = 1.225; // 参照密度 [kg/m³]
    bool use_direct_load_map = false;

    // ---- 設計目標（解析マップ生成用、実測ではない）----
    Scalar CL_default = -2.5;       // CL（A_ref=1 なら −CL·A）@ 公称姿勢
    Scalar CD_default = 1.0;        // CD（A_ref=1 なら CD·A）@ 公称姿勢
    Scalar CoP_default = 0.45;      // 空力バランス（前輪配分）→ CoP_x = CoP_default·L_ref
    Scalar h_aero = 0.35;           // ドラッグ / 横力の作用高さ [m above ground]
    Scalar rh_f_nominal = 40.0;     // 感度の基準 RH [mm]
    Scalar rh_r_nominal = 55.0;
    Scalar dCL_drh_f = 0.015;       // 前 RH 1 mm 低下あたりの CL 相対増 [1/mm]
    Scalar dCL_drh_r = 0.010;       // 後 RH 1 mm 低下あたりの CL 相対増 [1/mm]
    Scalar CL_rh_min = 0.6;         // RH 効果の倍率下限 / 上限
    Scalar CL_rh_max = 1.4;
    Scalar roll_CL_loss = 3.0;      // ロール 1 rad あたりの CL 相対損失（下限 0.7 倍）[1/rad]
    Scalar yaw_CL_freq = 2.0;       // CL ∝ cos(k·yaw)（下限 0.6 倍）
    Scalar yaw_CD_quad = 2.0;       // CD ∝ 1 + k·yaw² [1/rad²]
    Scalar yaw_CY_gain = 0.5;       // CY = k·sin(2·yaw)
    Scalar dCoP_drh_f = -0.003;     // 前 RH 1 mm 増での CoP_x 変化 [m/mm]（前を下げるとバランス前進）
    Scalar dCoP_drh_r = 0.002;      // 後 RH 1 mm 増での CoP_x 変化 [m/mm]（後を下げるとバランス後退）

    // ---- DRS ON 目標（マップに drs_on ブロックが無い場合に使用）----
    Scalar drs_CL_ratio = 0.54;     // CL_on / CL_off
    Scalar drs_CD_ratio = 0.74;     // CD_on / CD_off
    /// CoP_x 変化 [m]。NaN = リアウイングのみの DRS 規則：除去 DF (1−r)·Fz をリア車軸位置に置くと
    /// フロント DF 不変 → CoP_on = CoP_off / drs_CL_ratio（≤ L_ref にクランプ）
    Scalar drs_CoP_x_shift = std::numeric_limits<Scalar>::quiet_NaN();
    Scalar drs_side_moment_ratio = 0.70; // direct-load の Mx/Mz 倍率

    // 公称既定（direct-load）
    Scalar Fx_default = -80.0;      // [N] at reference speed
    Scalar Fy_default = 0.0;        // [N] at reference speed
    Scalar Fz_default = -350.0;     // [N] at reference speed
    Scalar Mx_default = 0.0;        // [Nm] at reference speed
    Scalar My_default = 0.0;        // [Nm] at reference speed
    Scalar Mz_default = 0.0;        // [Nm] at reference speed
};

/// 4D補間用グリッド
template<typename Scalar = double>
struct AeroMapGrid {
    std::vector<Scalar> rh_f;    // フロントRH [mm]
    std::vector<Scalar> rh_r;    // リアRH [mm]
    std::vector<Scalar> roll;    // ロール角 [rad]
    std::vector<Scalar> yaw;     // ヨー角 [rad]
};

/// 4Dエアロマップクラス
template<typename Scalar = double>
class AeroMap4D {
public:
    AeroMap4D() {
        init_default_grid();
        init_sample_data();
    }

    explicit AeroMap4D(const AeroMapParams<Scalar>& params)
        : params_(params) {
        init_default_grid();
        init_sample_data();
    }

    /// 空力係数を計算（4D補間 + DRS対応）
    AeroOutput<Scalar> compute(
        Scalar rh_front_mm,         // フロントライドハイト [mm]
        Scalar rh_rear_mm,          // リアライドハイト [mm]
        Scalar roll_rad,            // ロール角 [rad]
        Scalar yaw_rad,             // エアロヨー角 [rad]
        bool drs_active = false) const  // DRS起動フラグ
    {
        const Cell c = locate(rh_front_mm, rh_rear_mm, roll_rad, yaw_rad);
        const Tables& t = (drs_active && has_drs_table_) ? drs_ : off_;

        AeroOutput<Scalar> out;
        out.out_of_domain = c.ood;
        out.CL = interp(t.CL, c);
        out.CD = interp(t.CD, c);
        out.CY = interp(t.CY, c);
        out.CoP_x = interp(t.CoP_x, c);
        out.CoP_y_x = interp(t.CoP_y_x, c);

        if (drs_active && !has_drs_table_) {
            out.CoP_x = drs_cop_on(out.CoP_x);
            out.CL *= params_.drs_CL_ratio;
            out.CD *= params_.drs_CD_ratio;
        }

        out.Cm = out.CL * (out.CoP_x - params_.x_cg) / params_.L_ref;
        return out;
    }

    /// 力とモーメント（CG 回り）を計算
    /// @param drs_frac DRS フラップ位置 [0-1]（bool 可。中間値は OFF/ON の線形ブレンド）
    AeroForces<Scalar> compute_forces(
        Scalar velocity,            // 車速 [m/s]
        Scalar rh_front_mm,         // フロントライドハイト [mm]
        Scalar rh_rear_mm,          // リアライドハイト [mm]
        Scalar roll_rad,            // ロール角 [rad]
        Scalar yaw_rad,             // エアロヨー角 [rad]
        Scalar drs_frac = Scalar(0)) const
    {
        if (drs_frac <= Scalar(0) || drs_frac >= Scalar(1)) {
            return compute_forces_binary(velocity, rh_front_mm, rh_rear_mm, roll_rad, yaw_rad,
                                         drs_frac >= Scalar(1));
        }
        auto a = compute_forces_binary(velocity, rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, false);
        const auto b = compute_forces_binary(velocity, rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, true);
        const Scalar w = drs_frac;
        for (auto [p, q] : std::initializer_list<std::pair<Scalar*, const Scalar*>>{
                 {&a.Fx, &b.Fx}, {&a.Fy, &b.Fy}, {&a.Fz, &b.Fz}, {&a.Mx, &b.Mx},
                 {&a.My, &b.My}, {&a.Mz, &b.Mz}, {&a.Fz_front, &b.Fz_front}, {&a.Fz_rear, &b.Fz_rear}}) {
            *p += w * (*q - *p);
        }
        return a;
    }

    /// 基準点まわりのモーメントを CG まわりへ移す：M_cg = M_ref + (r_ref − r_cg) × F
    /// @param dx,dy,dz r_ref − r_cg（SAE 車体系: X 前, Y 右, Z 下）[m]
    /// f.Fz は揚力正なので SAE の Fz = −f.Fz として扱う。
    static void transfer_moments(AeroForces<Scalar>& f, Scalar dx, Scalar dy, Scalar dz) {
        const Scalar fz = -f.Fz;
        f.Mx += dy * fz - dz * f.Fy;
        f.My += dz * f.Fx - dx * fz;
        f.Mz += dx * f.Fy - dy * f.Fx;
    }

    /// ライドハイトをサスペンション変位から計算
    static Scalar compute_ride_height(Scalar z_sprung, Scalar z_u,
                                      Scalar static_rh_mm, Scalar wheel_radius) {
        (void)wheel_radius;
        // z は下向き正なので、ばね上-ばね下の増加は ride height 低下を意味する
        return static_rh_mm - (z_sprung - z_u) * Scalar(1000.0);
    }

    /// パラメータアクセス（目標値を変えたら set_coefficient_defaults() で再生成すること）
    AeroMapParams<Scalar>& params() { return params_; }
    const AeroMapParams<Scalar>& params() const { return params_; }
    const AeroMapGrid<Scalar>& grid() const { return grid_; }
    bool uses_direct_load_map() const { return params_.use_direct_load_map; }
    bool has_drs_table() const { return has_drs_table_; }
    /// 直近の load_from_json 失敗理由（成功時は空）
    const std::string& last_error() const { return last_error_; }
    /// 域外（クランプ）評価の累計回数
    std::uint64_t out_of_domain_count() const { return ood_count_; }

    void set_reference_conditions(Scalar reference_speed, Scalar reference_density) {
        params_.reference_speed = std::max(reference_speed, Scalar(0.1));
        params_.reference_density = std::max(reference_density, Scalar(0.1));
    }

    void set_direct_load_defaults(Scalar fx, Scalar fy, Scalar fz, Scalar mx, Scalar my, Scalar mz) {
        params_.Fx_default = fx;
        params_.Fy_default = fy;
        params_.Fz_default = fz;
        params_.Mx_default = mx;
        params_.My_default = my;
        params_.Mz_default = mz;
        params_.use_direct_load_map = true;
        init_direct_load_data();
    }

    /// 目標 CL/CD/バランスを設定し、params() の感度目標から解析マップを再生成
    void set_coefficient_defaults(Scalar cl, Scalar cd, Scalar balance_front = Scalar(0.45)) {
        params_.CL_default = cl;
        params_.CD_default = cd;
        params_.CoP_default = balance_front;
        params_.use_direct_load_map = false;
        init_sample_data();
    }

    /// 実測マップ JSON を読み込み、目標値マップを上書きする。
    /// 失敗時は false を返し、理由を last_error() と info ログに残す（自身は変更しない）。
    /// 書式: {aero_map: {V_ref_mps, reference_density, axes{rh_front_mm, rh_rear_mm, roll_rad|roll_deg,
    ///   yaw_rad|yaw_deg}, reference_point{x_from_rear_axle_m, y_m, height_m}?, data{...}, drs_on{...}?}}
    ///   data: direct-load {Fx_N,Fy_N,Fz_N,Mx_Nm,My_Nm,Mz_Nm} または係数 {CL,CD,CY,CoP_x, CoP_y_x?}
    ///   reference_point: direct-load モーメントの基準点（無ければ CG 回りとみなす）
    ///   drs_on: data と同じキーの任意部分集合（DRS ON 時に置換）
    bool load_from_json(const std::string& filepath) {
        last_error_ = load_impl(filepath);
        if (!last_error_.empty()) {
            spdlog::info("aero map '{}' not loaded ({}); using target-aero map", filepath, last_error_);
            return false;
        }
        spdlog::info("aero map '{}' loaded ({} mode{}), overrides target-aero coefficients", filepath,
                     params_.use_direct_load_map ? "direct-load" : "coefficient",
                     has_drs_table_ ? ", with drs_on" : "");
        return true;
    }

private:
    struct Tables {
        std::vector<Scalar> CL, CD, CY, CoP_x, CoP_y_x;
        std::vector<Scalar> Fx, Fy, Fz, Mx, My, Mz;
    };
    struct Cell {
        std::array<size_t, 4> i{};
        std::array<Scalar, 4> w{};
        std::uint8_t ood = 0;
    };

    AeroMapParams<Scalar> params_;
    AeroMapGrid<Scalar> grid_;
    Tables off_;
    Tables drs_;
    bool has_drs_table_ = false;
    bool has_ref_point_ = false;
    std::array<Scalar, 3> ref_point_{};   // {x from rear axle, y, height above ground} [m]
    std::string last_error_;
    // ponytail: 非アトミックな診断カウンタ（1 インスタンス 1 スレッド前提）、共有するなら std::atomic へ
    mutable std::uint64_t ood_count_ = 0;

    AeroForces<Scalar> compute_forces_binary(Scalar velocity, Scalar rh_front_mm, Scalar rh_rear_mm,
                                             Scalar roll_rad, Scalar yaw_rad, bool drs_active) const {
        AeroForces<Scalar> f;
        const Scalar dh = params_.h_aero - params_.h_cg;   // ドラッグ / 横力の CG 上方高さ
        if (params_.use_direct_load_map) {
            const Cell c = locate(rh_front_mm, rh_rear_mm, roll_rad, yaw_rad);
            const Tables& t = (drs_active && has_drs_table_) ? drs_ : off_;
            f.out_of_domain = c.ood;
            f.Fx = interp(t.Fx, c);
            f.Fy = interp(t.Fy, c);
            f.Fz = interp(t.Fz, c);
            f.Mx = interp(t.Mx, c);
            f.My = interp(t.My, c);
            f.Mz = interp(t.Mz, c);

            const Scalar reference_speed = std::max(params_.reference_speed, Scalar(0.1));
            const Scalar reference_density = std::max(params_.reference_density, Scalar(0.1));
            const Scalar scale = (params_.rho / reference_density) *
                                 (velocity * velocity) / (reference_speed * reference_speed);
            for (Scalar* v : {&f.Fx, &f.Fy, &f.Fz, &f.Mx, &f.My, &f.Mz}) *v *= scale;

            if (has_ref_point_) {
                transfer_moments(f, ref_point_[0] - params_.x_cg, ref_point_[1],
                                 -(ref_point_[2] - params_.h_cg));
            }

            // 鉛直 CoP（CG 相対, My ≈ Δx·Fz とみなす）
            Scalar x_rel = std::abs(f.Fz) > Scalar(1e-9) ? f.My / f.Fz : Scalar(0);
            if (drs_active && !has_drs_table_) {
                const Scalar fz_on = f.Fz * params_.drs_CL_ratio;
                const Scalar x_rel_on = drs_cop_on(params_.x_cg + x_rel) - params_.x_cg;
                f.My += x_rel_on * fz_on - x_rel * f.Fz;
                f.Fz = fz_on;
                f.Fx *= params_.drs_CD_ratio;
                f.Mx *= params_.drs_side_moment_ratio;
                f.Mz *= params_.drs_side_moment_ratio;
                x_rel = x_rel_on;
            }
            set_axle_split(f, params_.x_cg + x_rel);
            return f;
        }

        const Scalar q = Scalar(0.5) * params_.rho * velocity * velocity;
        const auto coef = compute(rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, drs_active);
        f.out_of_domain = coef.out_of_domain;
        f.Fx = -q * params_.A_ref * coef.CD;  // ドラッグ（後方）
        f.Fy = q * params_.A_ref * coef.CY;   // サイドフォース
        f.Fz = q * params_.A_ref * coef.CL;   // リフト（ダウンフォースは負）

        // 鉛直力は (CoP_x, h_aero 不問)、ドラッグは h_aero、横力は (CoP_y_x, h_aero) に作用
        f.Mx = dh * f.Fy;
        f.My = (coef.CoP_x - params_.x_cg) * f.Fz - dh * f.Fx;
        f.Mz = (coef.CoP_y_x - params_.x_cg) * f.Fy;
        set_axle_split(f, coef.CoP_x);
        return f;
    }

    void set_axle_split(AeroForces<Scalar>& f, Scalar cop_x_from_rear) const {
        const Scalar front = std::clamp(cop_x_from_rear / params_.L_ref, Scalar(0), Scalar(1));
        f.Fz_front = f.Fz * front;
        f.Fz_rear = f.Fz - f.Fz_front;
    }

    /// DRS ON 時の鉛直 CoP [m from rear axle]（drs_on データが無い場合の目標/規則）
    Scalar drs_cop_on(Scalar cop_off) const {
        if (!std::isnan(params_.drs_CoP_x_shift)) {
            return cop_off + params_.drs_CoP_x_shift;
        }
        static bool logged = false;
        if (!logged) {
            logged = true;
            spdlog::info("DRS: no drs_on data / CoP target; rear-wing-only rule CoP_on = CoP_off / {:.2f}",
                         static_cast<double>(params_.drs_CL_ratio));
        }
        return std::min(cop_off / std::max(params_.drs_CL_ratio, Scalar(1e-3)), params_.L_ref);
    }

    std::string load_impl(const std::string& filepath) {
        try {
            std::ifstream file(filepath);
            if (!file.is_open()) {
                return "cannot open file";
            }

            nlohmann::json root;
            file >> root;
            const auto& config = root.contains("aero_map") ? root["aero_map"] : root;

            AeroMap4D<Scalar> loaded(*this);
            loaded.has_drs_table_ = false;
            loaded.has_ref_point_ = false;

            if (config.contains("V_ref_mps")) {
                loaded.params_.reference_speed = config["V_ref_mps"];
            }
            if (config.contains("reference_density")) {
                loaded.params_.reference_density = config["reference_density"];
            }
            if (config.contains("reference_point")) {
                const auto& rp = config["reference_point"];
                for (const char* k : {"x_from_rear_axle_m", "y_m", "height_m"}) {
                    if (!rp.contains(k)) return std::string("reference_point missing '") + k + "'";
                }
                loaded.ref_point_ = {rp["x_from_rear_axle_m"].get<Scalar>(), rp["y_m"].get<Scalar>(),
                                     rp["height_m"].get<Scalar>()};
                loaded.has_ref_point_ = true;
            }

            if (config.contains("axes")) {
                const auto& axes = config["axes"];
                auto deg_axis = [&](const char* rad, const char* deg, std::vector<Scalar>& dst) {
                    if (axes.contains(rad)) {
                        dst = axes[rad].template get<std::vector<Scalar>>();
                    } else if (axes.contains(deg)) {
                        dst.clear();
                        for (auto v : axes[deg].template get<std::vector<Scalar>>()) dst.push_back(v * Scalar(M_PI / 180.0));
                    }
                };
                if (axes.contains("rh_front_mm")) loaded.grid_.rh_f = axes["rh_front_mm"].get<std::vector<Scalar>>();
                if (axes.contains("rh_rear_mm")) loaded.grid_.rh_r = axes["rh_rear_mm"].get<std::vector<Scalar>>();
                deg_axis("roll_rad", "roll_deg", loaded.grid_.roll);
                deg_axis("yaw_rad", "yaw_deg", loaded.grid_.yaw);
            }

            const std::pair<const char*, const std::vector<Scalar>*> axes_list[] = {
                {"rh_front_mm", &loaded.grid_.rh_f}, {"rh_rear_mm", &loaded.grid_.rh_r},
                {"roll", &loaded.grid_.roll}, {"yaw", &loaded.grid_.yaw}};
            for (const auto& [name, ax] : axes_list) {
                if (ax->size() < 2) return std::string("axis '") + name + "' needs >= 2 points";
                for (size_t i = 1; i < ax->size(); ++i) {
                    if ((*ax)[i] <= (*ax)[i - 1]) return std::string("axis '") + name + "' not strictly ascending";
                }
            }

            loaded.allocate_data();
            const size_t expected_size = loaded.expected_data_size();
            const auto& data = config.contains("data") ? config["data"] : config;

            // 戻り値: "" = 成功, "absent" = キー無し, それ以外 = 形状エラー
            auto load_vector = [&](const nlohmann::json& src, const char* key,
                                   std::vector<Scalar>& dst) -> std::string {
                if (!src.contains(key)) return "absent";
                if (!src[key].is_array()) return std::string("'") + key + "' is not an array";
                auto values = src[key].template get<std::vector<Scalar>>();
                if (values.size() != expected_size) {
                    return std::string("'") + key + "' has " + std::to_string(values.size()) +
                           " values, expected " + std::to_string(expected_size) + " (= nRHf*nRHr*nRoll*nYaw)";
                }
                dst = std::move(values);
                return "";
            };
            auto load_set = [&](const nlohmann::json& src, Tables& t, bool direct, bool optional) -> std::string {
                using Keys = std::vector<std::pair<const char*, std::vector<Scalar>*>>;
                const Keys keys = direct
                    ? Keys{{"Fx_N", &t.Fx}, {"Fy_N", &t.Fy}, {"Fz_N", &t.Fz},
                           {"Mx_Nm", &t.Mx}, {"My_Nm", &t.My}, {"Mz_Nm", &t.Mz}}
                    : Keys{{"CL", &t.CL}, {"CD", &t.CD}, {"CY", &t.CY}, {"CoP_x", &t.CoP_x}};
                for (const auto& [key, dst] : keys) {
                    const std::string err = load_vector(src, key, *dst);
                    if (err == "absent" && optional) continue;
                    if (!err.empty()) return err == "absent" ? std::string("missing '") + key + "'" : err;
                }
                return "";
            };

            const bool direct = data.contains("Fx_N");
            if (const auto err = load_set(data, loaded.off_, direct, false); !err.empty()) {
                return (direct ? "direct-load data: " : "coefficient data: ") + err;
            }
            if (!direct) {
                const auto err = load_vector(data, "CoP_y_x", loaded.off_.CoP_y_x);
                if (err == "absent") loaded.off_.CoP_y_x = loaded.off_.CoP_x;
                else if (!err.empty()) return "coefficient data: " + err;
            }
            if (data.contains("CoP_dist")) {
                spdlog::warn("aero map: 'CoP_dist' is ignored (front balance = CoP_x / L_ref)");
            }

            if (config.contains("drs_on")) {
                loaded.drs_ = loaded.off_;
                const auto& drs = config["drs_on"];
                if (const auto err = load_set(drs, loaded.drs_, direct, true); !err.empty()) {
                    return "drs_on: " + err;
                }
                if (!direct) {
                    const auto err = load_vector(drs, "CoP_y_x", loaded.drs_.CoP_y_x);
                    if (!err.empty() && err != "absent") return "drs_on: " + err;
                }
                loaded.has_drs_table_ = true;
            }

            loaded.params_.use_direct_load_map = direct;
            *this = std::move(loaded);
            return "";
        } catch (const nlohmann::json::exception& e) {
            return std::string("json error: ") + e.what();
        }
    }

    // 4D配列インデックス
    size_t idx(size_t i, size_t j, size_t k, size_t l) const {
        size_t n1 = grid_.rh_r.size();
        size_t n2 = grid_.roll.size();
        size_t n3 = grid_.yaw.size();
        return ((i * n1 + j) * n2 + k) * n3 + l;
    }

    // データ配列確保
    void allocate_data() {
        const size_t total = expected_data_size();
        for (auto* v : {&off_.CL, &off_.CD, &off_.CY, &off_.CoP_x, &off_.CoP_y_x}) v->assign(total, Scalar(0));
        init_direct_load_data();
        has_drs_table_ = false;
    }

    // デフォルトグリッド初期化
    void init_default_grid() {
        // フロントRH: 20-60mm (5点)
        grid_.rh_f = {20, 30, 40, 50, 60};
        // リアRH: 30-80mm (6点)
        grid_.rh_r = {30, 40, 50, 60, 70, 80};
        // ロール: -5deg〜+5deg (5点)
        constexpr Scalar deg2rad = Scalar(3.14159265359 / 180.0);
        grid_.roll = {Scalar(-5)*deg2rad, Scalar(-2.5)*deg2rad, Scalar(0),
                      Scalar(2.5)*deg2rad, Scalar(5)*deg2rad};
        // ヨー: -15deg〜+15deg (7点)
        grid_.yaw = {Scalar(-15)*deg2rad, Scalar(-10)*deg2rad, Scalar(-5)*deg2rad,
                     Scalar(0), Scalar(5)*deg2rad, Scalar(10)*deg2rad, Scalar(15)*deg2rad};

        allocate_data();
    }

    // 設計目標から解析マップを生成（実測ではない）
    void init_sample_data() {
        const auto& p = params_;
        has_drs_table_ = false;
        has_ref_point_ = false;
        for (size_t i = 0; i < grid_.rh_f.size(); ++i) {
            const Scalar rh_f = grid_.rh_f[i];
            for (size_t j = 0; j < grid_.rh_r.size(); ++j) {
                const Scalar rh_r = grid_.rh_r[j];
                for (size_t k = 0; k < grid_.roll.size(); ++k) {
                    const Scalar roll = grid_.roll[k];
                    for (size_t l = 0; l < grid_.yaw.size(); ++l) {
                        const Scalar yaw = grid_.yaw[l];

                        // ライドハイト効果（地面効果: 低いほどダウンフォース増加）
                        const Scalar rh_effect = std::clamp(
                            Scalar(1) + p.dCL_drh_f * (p.rh_f_nominal - rh_f) + p.dCL_drh_r * (p.rh_r_nominal - rh_r),
                            p.CL_rh_min, p.CL_rh_max);
                        // ロール効果（フロアシール損失）
                        const Scalar roll_loss = std::clamp(Scalar(1) - p.roll_CL_loss * std::abs(roll), Scalar(0.7), Scalar(1));
                        // ヨー効果
                        const Scalar yaw_effect = std::clamp(Scalar(std::cos(yaw * p.yaw_CL_freq)), Scalar(0.6), Scalar(1));

                        const size_t index = idx(i, j, k, l);
                        off_.CL[index] = p.CL_default * rh_effect * roll_loss * yaw_effect;
                        off_.CD[index] = p.CD_default * (Scalar(1) + p.yaw_CD_quad * yaw * yaw);
                        off_.CY[index] = std::sin(yaw * Scalar(2)) * p.yaw_CY_gain;
                        off_.CoP_x[index] = p.CoP_default * p.L_ref
                                          + p.dCoP_drh_f * (rh_f - p.rh_f_nominal)
                                          + p.dCoP_drh_r * (rh_r - p.rh_r_nominal);
                        off_.CoP_y_x[index] = off_.CoP_x[index];
                    }
                }
            }
        }
    }

    void init_direct_load_data() {
        const size_t total = expected_data_size();
        off_.Fx.assign(total, params_.Fx_default);
        off_.Fy.assign(total, params_.Fy_default);
        off_.Fz.assign(total, params_.Fz_default);
        off_.Mx.assign(total, params_.Mx_default);
        off_.My.assign(total, params_.My_default);
        off_.Mz.assign(total, params_.Mz_default);
    }

    size_t expected_data_size() const {
        return grid_.rh_f.size() * grid_.rh_r.size() * grid_.roll.size() * grid_.yaw.size();
    }

    // インデックスと補間重み（範囲外は端にクランプ）
    static std::pair<size_t, Scalar> find_index_weight(
        const std::vector<Scalar>& grid, Scalar val) {
        if (grid.size() < 2) return {0, Scalar(0)};
        if (val <= grid.front()) return {0, Scalar(0)};
        if (val >= grid.back()) return {grid.size() - 2, Scalar(1)};
        const size_t i = static_cast<size_t>(std::upper_bound(grid.begin(), grid.end(), val) - grid.begin()) - 1;
        return {i, (val - grid[i]) / (grid[i + 1] - grid[i])};
    }

    Cell locate(Scalar rh_f, Scalar rh_r, Scalar roll, Scalar yaw) const {
        Cell c;
        const std::array<const std::vector<Scalar>*, 4> axes{&grid_.rh_f, &grid_.rh_r, &grid_.roll, &grid_.yaw};
        const std::array<Scalar, 4> v{rh_f, rh_r, roll, yaw};
        for (size_t a = 0; a < 4; ++a) {
            if (v[a] < axes[a]->front() || v[a] > axes[a]->back()) c.ood |= std::uint8_t(1u << a);
            std::tie(c.i[a], c.w[a]) = find_index_weight(*axes[a], v[a]);
        }
        if (c.ood) ++ood_count_;
        return c;
    }

    // 4D線形補間
    Scalar interp(const std::vector<Scalar>& data, const Cell& c) const {
        Scalar sum = Scalar(0);
        for (int di = 0; di <= 1; ++di) {
            const Scalar wi = di ? c.w[0] : Scalar(1) - c.w[0];
            for (int dj = 0; dj <= 1; ++dj) {
                const Scalar wj = dj ? c.w[1] : Scalar(1) - c.w[1];
                for (int dk = 0; dk <= 1; ++dk) {
                    const Scalar wk = dk ? c.w[2] : Scalar(1) - c.w[2];
                    for (int dl = 0; dl <= 1; ++dl) {
                        const Scalar wl = dl ? c.w[3] : Scalar(1) - c.w[3];
                        sum += wi * wj * wk * wl * data[idx(c.i[0] + di, c.i[1] + dj, c.i[2] + dk, c.i[3] + dl)];
                    }
                }
            }
        }
        return sum;
    }
};

} // namespace fsae::physics::aero
