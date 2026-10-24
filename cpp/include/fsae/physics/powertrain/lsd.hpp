#pragma once

#include <cmath>
#include <algorithm>

namespace fsae {
namespace physics {
namespace powertrain {

/**
 * @brief Salisbury式LSD（Limited Slip Differential）モデル
 * 
 * クラッチプレート式差動制限装置のモデル化。
 * 加速/減速時に異なるランプ角を使用し、左右輪間のトルク配分を制御。
 * 
 * 制限トルク計算式:
 *   T_lock = T_pre + (T_input / tan(θ_ramp)) × R_eff × μ_clutch × N_plates
 * 
 * 参考: 学生フォーミュラ車両運動シミュレーション総合ガイド セクション8.3
 */
class SalisburyLSD {
public:
    /**
     * @brief LSDパラメータ
     */
    struct Params {
        double ramp_angle_accel;  ///< 加速側ランプ角 [deg] (30-45°: 角度小→ロック強→トラクション重視)
        double ramp_angle_decel;  ///< 減速側ランプ角 [deg] (45-60°: 進入安定性を制御)
        double preload_torque;    ///< プリロードトルク [Nm] (20-80 Nm: 初期差動制限トルク)
        double effective_radius;  ///< クラッチ有効半径 [m] (0.03-0.05 m)
        double clutch_friction;   ///< クラッチ摩擦係数 [-] (0.08-0.12)
        int num_plates;           ///< クラッチプレート数 [-] (4-12枚)
        /// tanh 平滑化の差速スケール [rad/s]。RK4 @1 ms 安定条件 T_lock·dt/(Iw·ω_s) < 2.78 より
        /// Iw = 0.5 kg m², T_lock ≤ 130 Nm で ω_s ≥ 0.094 → 0.1（車輪周速差 ≈ 2 cm/s, 実用上ロック）
        double omega_smooth = 0.1;

        /**
         * @brief FSAE標準的なLSDパラメータ（トラクション重視）
         */
        static Params fsae_traction_focused() {
            Params p;
            p.ramp_angle_accel = 35.0;   // 強めのロック→加速トラクション確保
            p.ramp_angle_decel = 50.0;   // 進入時の安定性
            p.preload_torque = 40.0;     // 中程度のプリロード
            p.effective_radius = 0.04;   // 40mm
            p.clutch_friction = 0.10;    // 標準的な値
            p.num_plates = 8;            // 一般的な枚数
            return p;
        }
        
        /**
         * @brief バランス型LSDパラメータ
         */
        static Params fsae_balanced() {
            Params p;
            p.ramp_angle_accel = 40.0;   // 中程度のロック
            p.ramp_angle_decel = 55.0;   // やや緩め
            p.preload_torque = 30.0;     // 低めのプリロード
            p.effective_radius = 0.04;
            p.clutch_friction = 0.10;
            p.num_plates = 8;
            return p;
        }
        
        /**
         * @brief ハンドリング重視LSDパラメータ（オートクロス向け）
         */
        static Params fsae_handling_focused() {
            Params p;
            p.ramp_angle_accel = 45.0;   // 緩めのロック→回頭性重視
            p.ramp_angle_decel = 60.0;   // さらに緩め
            p.preload_torque = 20.0;     // 低プリロード
            p.effective_radius = 0.04;
            p.clutch_friction = 0.10;
            p.num_plates = 6;            // 少なめ
            return p;
        }
    };
    
    /**
     * @brief コンストラクタ
     * @param params LSDパラメータ
     */
    explicit SalisburyLSD(const Params& params = Params::fsae_traction_focused())
        : params_(params) {}
    
    /**
     * @brief パラメータ設定
     */
    void set_params(const Params& params) {
        params_ = params;
    }
    
    /**
     * @brief パラメータ取得
     */
    const Params& get_params() const {
        return params_;
    }
    
    /**
     * @brief 差動制限トルクを計算
     * 
     * @param input_torque 入力トルク（エンジントルク×ギア比） [Nm]
     * @param omega_left 左輪角速度 [rad/s]
     * @param omega_right 右輪角速度 [rad/s]
     * @return locking_torque 差動制限トルク [Nm]
     * 
     * 正のトルク = 差動を制限する方向
     * 制限トルクは左右輪間のトルク差の最大値を制限
     */
    double compute_locking_torque(double input_torque, 
                                   double omega_left, 
                                   double omega_right) const {
        // 加速/減速判定（入力トルクの符号で判断）
        bool is_accelerating = input_torque > 0.0;
        
        // 使用するランプ角を選択
        double ramp_angle_deg = is_accelerating ? 
            params_.ramp_angle_accel : params_.ramp_angle_decel;
        double ramp_angle_rad = ramp_angle_deg * M_PI / 180.0;
        
        // Salisbury式による制限トルク計算
        // T_lock = T_pre + (T_input / tan(θ)) × R_eff × μ × N
        double torque_dependent_term = 
            (std::abs(input_torque) / std::tan(ramp_angle_rad)) *
            params_.effective_radius *
            params_.clutch_friction *
            static_cast<double>(params_.num_plates);
        
        // プリロード + トルク依存成分（速度差に依存しない）
        return params_.preload_torque + torque_dependent_term;
    }
    
    /**
     * @brief 左右トルク配分を計算
     * 
     * @param total_torque 総駆動トルク [Nm]
     * @param omega_left 左輪角速度 [rad/s]
     * @param omega_right 右輪角速度 [rad/s]
     * @param torque_left 左輪配分トルク [Nm] (出力)
     * @param torque_right 右輪配分トルク [Nm] (出力)
     * 
     * オープンデフの場合: 遅い方の車輪に全トルク集中
     * LSDの場合: 差動制限により高速側にもトルク配分
     */
    void distribute_torque(double total_torque,
                          double omega_left,
                          double omega_right,
                          double& torque_left,
                          double& torque_right) const {
        // 差動制限トルク（プリロード + ランプ項。ランプは入力トルクの符号で選択：
        // エンジンブレーキ時は減速ランプ）
        const double locking_torque = compute_locking_torque(
            total_torque, omega_left, omega_right);

        // オープンデフ基本成分: 均等配分
        const double base_torque = total_torque * 0.5;

        // 連続化したクラッチ摩擦: T_bias = T_lock·tanh(Δω/ω_s)
        // |base_torque| キャップは撤廃（実 Salisbury のプリロードは入力トルクと無関係に効く → コーストでも有効）。
        // 摩擦は速い輪からトルクを奪い遅い輪に渡す。
        // ponytail: 静摩擦ロック拘束は無し（Δω≈0 で T_bias→0）。実測ロッキング面 T_lock(T_in,Δω) が
        // 得られたら compute_locking_torque を 2D LUT に置換する。
        const double omega_s = std::max(params_.omega_smooth, 1e-6);
        const double torque_bias = locking_torque * std::tanh((omega_left - omega_right) / omega_s);
        torque_left = base_torque - torque_bias * 0.5;
        torque_right = base_torque + torque_bias * 0.5;
    }
    
    /**
     * @brief LSD効率（トルク配分比）を計算
     * 
     * @return bias_ratio トルクバイアス比 [-] (1.0=オープンデフ, >1.0=LSD効果)
     * 
     * トルクバイアス比 = (高速側トルク / 低速側トルク)
     * 典型的なFSAE LSD: 1.5-3.0倍程度
     */
    double compute_bias_ratio(double input_torque,
                             double omega_left,
                             double omega_right) const {
        double torque_left, torque_right;
        distribute_torque(input_torque, omega_left, omega_right,
                         torque_left, torque_right);
        
        double slow_torque = std::min(std::abs(torque_left), std::abs(torque_right));
        double fast_torque = std::max(std::abs(torque_left), std::abs(torque_right));
        
        if (slow_torque < 1e-6) {
            return 1.0;  // トルクがない場合
        }
        
        return fast_torque / slow_torque;
    }

private:
    Params params_;  ///< LSDパラメータ
};

}  // namespace powertrain
}  // namespace physics
}  // namespace fsae
