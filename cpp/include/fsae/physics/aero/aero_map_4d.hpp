#pragma once
/// @file aero_map_4d.hpp
/// @brief 4次元エアロマップ（RHf, RHr, roll, yaw）
/// 
/// 設計書 Section 6 準拠
/// 入力変数:
///   - RH_f: フロントライドハイト [mm]
///   - RH_r: リアライドハイト [mm]
///   - roll: ロール角 [rad]
///   - yaw:  エアロヨー角 [rad]

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace fsae::physics::aero {

/// 空力係数出力
template<typename Scalar = double>
struct AeroOutput {
    Scalar CL = 0;      // リフト係数（負=ダウンフォース）
    Scalar CD = 0;      // ドラッグ係数
    Scalar CY = 0;      // サイドフォース係数
    Scalar Cm = 0;      // ピッチングモーメント係数
    Scalar CoP_x = 0;   // 空力中心X位置 [m from rear axle]
    Scalar CoP_dist = 0.5; // 前後DF配分 (0=リア, 1=フロント)
};

/// 空力・モーメント出力
template<typename Scalar = double>
struct AeroForces {
    Scalar Fx = 0;      // ドラッグ [N] (負=後方)
    Scalar Fy = 0;      // サイドフォース [N]
    Scalar Fz = 0;      // リフト [N] (負=ダウンフォース)
    Scalar Mx = 0;      // ロールモーメント [Nm]
    Scalar My = 0;      // ピッチモーメント [Nm]
    Scalar Mz = 0;      // ヨーモーメント [Nm]
    
    // 前後輪へのダウンフォース配分
    Scalar Fz_front = 0;
    Scalar Fz_rear = 0;
};

/// 4Dエアロマップパラメータ
template<typename Scalar = double>
struct AeroMapParams {
    Scalar rho = 1.225;             // 空気密度 [kg/m³]
    Scalar A_ref = 1.0;             // 参照面積 [m²]
    Scalar L_ref = 1.6;             // 参照長さ（ホイールベース）[m]
    Scalar x_cg = 0.8;              // CG位置 [m from rear axle]
    Scalar reference_speed = 15.0;  // 参照速度 [m/s]
    Scalar reference_density = 1.225; // 参照密度 [kg/m³]
    bool use_direct_load_map = false;
     
    // デフォルト係数（マップがない場合）
    Scalar CL_default = -2.5;
    Scalar CD_default = 1.0;
    Scalar CoP_default = 0.45;      // 基準圧力中心
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
        // グリッド範囲にクランプ
        rh_front_mm = clamp(rh_front_mm, grid_.rh_f.front(), grid_.rh_f.back());
        rh_rear_mm = clamp(rh_rear_mm, grid_.rh_r.front(), grid_.rh_r.back());
        roll_rad = clamp(roll_rad, grid_.roll.front(), grid_.roll.back());
        yaw_rad = clamp(yaw_rad, grid_.yaw.front(), grid_.yaw.back());

        // インデックスと重みを計算
        auto [i_f, w_f] = find_index_weight(grid_.rh_f, rh_front_mm);
        auto [i_r, w_r] = find_index_weight(grid_.rh_r, rh_rear_mm);
        auto [i_roll, w_roll] = find_index_weight(grid_.roll, roll_rad);
        auto [i_yaw, w_yaw] = find_index_weight(grid_.yaw, yaw_rad);

        // 4D線形補間
        AeroOutput<Scalar> out;
        out.CL = interpolate_4d(CL_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
        out.CD = interpolate_4d(CD_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
        out.CY = interpolate_4d(CY_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
        out.CoP_x = interpolate_4d(CoP_x_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
        out.CoP_dist = interpolate_4d(CoP_dist_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);

        // ピッチモーメント係数
        out.Cm = out.CL * (out.CoP_x - params_.x_cg) / params_.L_ref;

        // DRS起動時は空力係数を減少（4D補間後に乗算）
        if (drs_active) {
            out.CL *= Scalar(0.54);  // -46% ダウンフォース（リアウイング開放）
            out.CD *= Scalar(0.74);  // -26% 空気抵抗
            // CY, CoP_x, CoP_dist は変化なし
        }

        return out;
    }
    
    /// 力とモーメントを計算（DRS対応）
    AeroForces<Scalar> compute_forces(
        Scalar velocity,            // 車速 [m/s]
        Scalar rh_front_mm,         // フロントライドハイト [mm]
        Scalar rh_rear_mm,          // リアライドハイト [mm]
        Scalar roll_rad,            // ロール角 [rad]
        Scalar yaw_rad,             // エアロヨー角 [rad]
        bool drs_active = false) const  // DRS起動フラグ
    {
        if (params_.use_direct_load_map) {
            rh_front_mm = clamp(rh_front_mm, grid_.rh_f.front(), grid_.rh_f.back());
            rh_rear_mm = clamp(rh_rear_mm, grid_.rh_r.front(), grid_.rh_r.back());
            roll_rad = clamp(roll_rad, grid_.roll.front(), grid_.roll.back());
            yaw_rad = clamp(yaw_rad, grid_.yaw.front(), grid_.yaw.back());

            auto [i_f, w_f] = find_index_weight(grid_.rh_f, rh_front_mm);
            auto [i_r, w_r] = find_index_weight(grid_.rh_r, rh_rear_mm);
            auto [i_roll, w_roll] = find_index_weight(grid_.roll, roll_rad);
            auto [i_yaw, w_yaw] = find_index_weight(grid_.yaw, yaw_rad);

            AeroForces<Scalar> f;
            f.Fx = interpolate_4d(Fx_ref_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
            f.Fy = interpolate_4d(Fy_ref_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
            f.Fz = interpolate_4d(Fz_ref_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
            f.Mx = interpolate_4d(Mx_ref_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
            f.My = interpolate_4d(My_ref_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);
            f.Mz = interpolate_4d(Mz_ref_data_, i_f, i_r, i_roll, i_yaw, w_f, w_r, w_roll, w_yaw);

            const Scalar reference_speed = std::max(params_.reference_speed, Scalar(0.1));
            const Scalar reference_density = std::max(params_.reference_density, Scalar(0.1));
            const Scalar density_ratio = params_.rho / reference_density;
            const Scalar speed_ratio_sq = (velocity * velocity) / (reference_speed * reference_speed);
            const Scalar scale = density_ratio * speed_ratio_sq;

            f.Fx *= scale;
            f.Fy *= scale;
            f.Fz *= scale;
            f.Mx *= scale;
            f.My *= scale;
            f.Mz *= scale;

            if (drs_active) {
                f.Fx *= Scalar(0.74);
                f.Fz *= Scalar(0.54);
                f.My *= Scalar(0.54);
                f.Mx *= Scalar(0.70);
                f.Mz *= Scalar(0.70);
            }

            // direct-load モードでは前後配分は現時点では単純配分に留める
            const Scalar front_ratio = clamp(Scalar(0.5), Scalar(0.0), Scalar(1.0));
            f.Fz_front = f.Fz * front_ratio;
            f.Fz_rear = f.Fz * (Scalar(1.0) - front_ratio);
            return f;
        }

        // 動圧
        Scalar q = Scalar(0.5) * params_.rho * velocity * velocity;

        // 係数取得（DRSフラグを渡す）
        auto coef = compute(rh_front_mm, rh_rear_mm, roll_rad, yaw_rad, drs_active);
        
        // 力計算
        AeroForces<Scalar> f;
        f.Fx = -q * params_.A_ref * coef.CD;  // ドラッグ（後方）
        f.Fy = q * params_.A_ref * coef.CY;   // サイドフォース
        f.Fz = q * params_.A_ref * coef.CL;   // リフト（ダウンフォースは負）
        
        // モーメント計算
        Scalar lever_arm = coef.CoP_x - params_.x_cg;
        f.My = f.Fz * lever_arm;  // ピッチモーメント
        
        // ヨー角によるモーメント
        f.Mz = f.Fy * lever_arm;
        
        // 前後荷重配分
        f.Fz_front = f.Fz * coef.CoP_dist;
        f.Fz_rear = f.Fz * (Scalar(1.0) - coef.CoP_dist);
        
        return f;
    }
    
    /// ライドハイトをサスペンション変位から計算
    static Scalar compute_ride_height(Scalar z_sprung, Scalar z_u, 
                                      Scalar static_rh_mm, Scalar wheel_radius) {
        (void)wheel_radius;
        // z は下向き正なので、ばね上-ばね下の増加は ride height 低下を意味する
        return static_rh_mm - (z_sprung - z_u) * Scalar(1000.0);
    }
    
    /// パラメータアクセス
    AeroMapParams<Scalar>& params() { return params_; }
    const AeroMapParams<Scalar>& params() const { return params_; }
    const AeroMapGrid<Scalar>& grid() const { return grid_; }
    bool uses_direct_load_map() const { return params_.use_direct_load_map; }

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

    void set_coefficient_defaults(Scalar cl, Scalar cd, Scalar cop_dist = Scalar(0.45)) {
        params_.CL_default = cl;
        params_.CD_default = cd;
        params_.CoP_default = cop_dist;
        params_.use_direct_load_map = false;
        init_sample_data();
    }

    bool load_from_json(const std::string& filepath) {
        try {
            std::ifstream file(filepath);
            if (!file.is_open()) {
                return false;
            }

            nlohmann::json root;
            file >> root;
            const auto& config = root.contains("aero_map") ? root["aero_map"] : root;

            AeroMap4D<Scalar> loaded(*this);

            if (config.contains("V_ref_mps")) {
                loaded.params_.reference_speed = config["V_ref_mps"];
            }
            if (config.contains("reference_density")) {
                loaded.params_.reference_density = config["reference_density"];
            }

            if (config.contains("axes")) {
                const auto& axes = config["axes"];
                if (axes.contains("rh_front_mm")) loaded.grid_.rh_f = axes["rh_front_mm"].get<std::vector<Scalar>>();
                if (axes.contains("rh_rear_mm")) loaded.grid_.rh_r = axes["rh_rear_mm"].get<std::vector<Scalar>>();
                if (axes.contains("roll_rad")) loaded.grid_.roll = axes["roll_rad"].get<std::vector<Scalar>>();
                else if (axes.contains("roll_deg")) {
                    auto roll_deg = axes["roll_deg"].get<std::vector<Scalar>>();
                    loaded.grid_.roll.clear();
                    for (auto value : roll_deg) loaded.grid_.roll.push_back(value * Scalar(M_PI / 180.0));
                }
                if (axes.contains("yaw_rad")) loaded.grid_.yaw = axes["yaw_rad"].get<std::vector<Scalar>>();
                else if (axes.contains("yaw_deg")) {
                    auto yaw_deg = axes["yaw_deg"].get<std::vector<Scalar>>();
                    loaded.grid_.yaw.clear();
                    for (auto value : yaw_deg) loaded.grid_.yaw.push_back(value * Scalar(M_PI / 180.0));
                }
            }

            if (!loaded.has_valid_interpolation_grid()) {
                return false;
            }

            loaded.allocate_data();

            const auto& data = config.contains("data") ? config["data"] : config;
            const size_t expected_size = loaded.expected_data_size();
            auto load_vector = [&](const char* key, std::vector<Scalar>& dst) -> bool {
                if (!data.contains(key) || !data[key].is_array()) {
                    return false;
                }
                auto values = data[key].get<std::vector<Scalar>>();
                if (values.size() != expected_size) {
                    return false;
                }
                dst = std::move(values);
                return true;
            };

            const bool has_direct_data =
                load_vector("Fx_N", loaded.Fx_ref_data_) &&
                load_vector("Fy_N", loaded.Fy_ref_data_) &&
                load_vector("Fz_N", loaded.Fz_ref_data_) &&
                load_vector("Mx_Nm", loaded.Mx_ref_data_) &&
                load_vector("My_Nm", loaded.My_ref_data_) &&
                load_vector("Mz_Nm", loaded.Mz_ref_data_);

            if (has_direct_data) {
                loaded.params_.use_direct_load_map = true;
                *this = std::move(loaded);
                return true;
            }

            const bool has_coeff_data =
                load_vector("CL", loaded.CL_data_) &&
                load_vector("CD", loaded.CD_data_) &&
                load_vector("CY", loaded.CY_data_) &&
                load_vector("CoP_x", loaded.CoP_x_data_) &&
                load_vector("CoP_dist", loaded.CoP_dist_data_);

            if (!has_coeff_data) {
                return false;
            }

            loaded.params_.use_direct_load_map = false;
            *this = std::move(loaded);
            return true;
        } catch (const nlohmann::json::exception&) {
            return false;
        }
    }

private:
    AeroMapParams<Scalar> params_;
    AeroMapGrid<Scalar> grid_;
    
    // データ配列（4D）
    std::vector<Scalar> CL_data_;
    std::vector<Scalar> CD_data_;
    std::vector<Scalar> CY_data_;
    std::vector<Scalar> CoP_x_data_;
    std::vector<Scalar> CoP_dist_data_;
    std::vector<Scalar> Fx_ref_data_;
    std::vector<Scalar> Fy_ref_data_;
    std::vector<Scalar> Fz_ref_data_;
    std::vector<Scalar> Mx_ref_data_;
    std::vector<Scalar> My_ref_data_;
    std::vector<Scalar> Mz_ref_data_;
    
    // 4D配列インデックス
    size_t idx(size_t i, size_t j, size_t k, size_t l) const {
        size_t n1 = grid_.rh_r.size();
        size_t n2 = grid_.roll.size();
        size_t n3 = grid_.yaw.size();
        return ((i * n1 + j) * n2 + k) * n3 + l;
    }
    
    // データ配列確保
    void allocate_data() {
        size_t total = grid_.rh_f.size() * grid_.rh_r.size() * 
                       grid_.roll.size() * grid_.yaw.size();
        CL_data_.resize(total, Scalar(0));
        CD_data_.resize(total, Scalar(0));
        CY_data_.resize(total, Scalar(0));
        CoP_x_data_.resize(total, Scalar(0));
        CoP_dist_data_.resize(total, Scalar(0.5));
        Fx_ref_data_.resize(total, params_.Fx_default);
        Fy_ref_data_.resize(total, params_.Fy_default);
        Fz_ref_data_.resize(total, params_.Fz_default);
        Mx_ref_data_.resize(total, params_.Mx_default);
        My_ref_data_.resize(total, params_.My_default);
        Mz_ref_data_.resize(total, params_.Mz_default);
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
    
    // サンプルデータ初期化（Formula Student典型値）
    void init_sample_data() {
        // ベース空力係数
        Scalar CL_base = params_.CL_default;  // -2.5（ダウンフォース）
        Scalar CD_base = params_.CD_default;  // 1.0
        Scalar CoP_base = params_.CoP_default; // 0.45
        
        for (size_t i = 0; i < grid_.rh_f.size(); ++i) {
            Scalar rh_f = grid_.rh_f[i];
            for (size_t j = 0; j < grid_.rh_r.size(); ++j) {
                Scalar rh_r = grid_.rh_r[j];
                for (size_t k = 0; k < grid_.roll.size(); ++k) {
                    Scalar roll = grid_.roll[k];
                    for (size_t l = 0; l < grid_.yaw.size(); ++l) {
                        Scalar yaw = grid_.yaw[l];
                        
                        // ライドハイト効果（地面効果）
                        // 低いほどダウンフォース増加
                        Scalar rh_effect = Scalar(1.0) + Scalar(0.015) * (Scalar(40) - rh_f) 
                                         + Scalar(0.010) * (Scalar(55) - rh_r);
                        rh_effect = clamp(rh_effect, Scalar(0.6), Scalar(1.4));
                        
                        // ロール効果（フロアシール損失）
                        Scalar roll_loss = Scalar(1.0) - Scalar(3.0) * std::abs(roll);
                        roll_loss = clamp(roll_loss, Scalar(0.7), Scalar(1.0));
                        
                        // ヨー効果
                        Scalar yaw_effect = std::cos(yaw * Scalar(2));
                        yaw_effect = clamp(yaw_effect, Scalar(0.6), Scalar(1.0));
                        Scalar CY_val = std::sin(yaw * Scalar(2)) * Scalar(0.5);
                        
                        // 最終係数
                        size_t index = idx(i, j, k, l);
                        CL_data_[index] = CL_base * rh_effect * roll_loss * yaw_effect;
                        CD_data_[index] = CD_base * (Scalar(1.0) + Scalar(2.0) * yaw * yaw);
                        CY_data_[index] = CY_val;
                        
                        // 圧力中心
                        Scalar CoP = CoP_base;
                        CoP += (rh_f - Scalar(40)) * Scalar(0.003);  // フロント下がるとCoP前進
                        CoP -= (rh_r - Scalar(55)) * Scalar(0.002);  // リア下がるとCoP後退
                        CoP_x_data_[index] = CoP;
                        
                        // 前後配分
                        Scalar dist = Scalar(0.5) + (rh_f - rh_r + Scalar(15)) * Scalar(0.003);
                        CoP_dist_data_[index] = clamp(dist, Scalar(0.35), Scalar(0.65));
                    }
                }
            }
        }
    }

    void init_direct_load_data() {
        const size_t total = grid_.rh_f.size() * grid_.rh_r.size() *
                             grid_.roll.size() * grid_.yaw.size();
        Fx_ref_data_.assign(total, params_.Fx_default);
        Fy_ref_data_.assign(total, params_.Fy_default);
        Fz_ref_data_.assign(total, params_.Fz_default);
        Mx_ref_data_.assign(total, params_.Mx_default);
        My_ref_data_.assign(total, params_.My_default);
        Mz_ref_data_.assign(total, params_.Mz_default);
    }

    size_t expected_data_size() const {
        return grid_.rh_f.size() * grid_.rh_r.size() * grid_.roll.size() * grid_.yaw.size();
    }

    bool has_valid_interpolation_grid() const {
        return grid_.rh_f.size() >= 2
            && grid_.rh_r.size() >= 2
            && grid_.roll.size() >= 2
            && grid_.yaw.size() >= 2;
    }
    
    // クランプ
    static Scalar clamp(Scalar val, Scalar lo, Scalar hi) {
        return std::max(lo, std::min(val, hi));
    }
    
    // インデックスと補間重みを計算
    static std::pair<size_t, Scalar> find_index_weight(
        const std::vector<Scalar>& grid, Scalar val) {
        if (grid.size() < 2) return {0, Scalar(0)};
        if (val <= grid.front()) return {0, Scalar(0)};
        if (val >= grid.back()) return {grid.size() - 2, Scalar(1)};
        
        // 線形探索（小さいグリッドなので）
        for (size_t i = 0; i < grid.size() - 1; ++i) {
            if (val >= grid[i] && val <= grid[i + 1]) {
                Scalar w = (val - grid[i]) / (grid[i + 1] - grid[i]);
                return {i, w};
            }
        }
        return {grid.size() - 2, Scalar(1)};
    }
    
    // 4D線形補間
    Scalar interpolate_4d(const std::vector<Scalar>& data,
                          size_t i0, size_t j0, size_t k0, size_t l0,
                          Scalar w0, Scalar w1, Scalar w2, Scalar w3) const {
        Scalar sum = Scalar(0);
        for (int di = 0; di <= 1; ++di) {
            Scalar wi = (di == 0) ? (Scalar(1) - w0) : w0;
            for (int dj = 0; dj <= 1; ++dj) {
                Scalar wj = (dj == 0) ? (Scalar(1) - w1) : w1;
                for (int dk = 0; dk <= 1; ++dk) {
                    Scalar wk = (dk == 0) ? (Scalar(1) - w2) : w2;
                    for (int dl = 0; dl <= 1; ++dl) {
                        Scalar wl = (dl == 0) ? (Scalar(1) - w3) : w3;
                        sum += wi * wj * wk * wl * 
                               data[idx(i0 + di, j0 + dj, k0 + dk, l0 + dl)];
                    }
                }
            }
        }
        return sum;
    }
};

} // namespace fsae::physics::aero
