#pragma once
/// @file vehicle_params.hpp
/// @brief 14DOF車両パラメータ構造体

#include <algorithm>
#include <array>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace fsae::vehicle {

/// Piecewise-linear lookup with constant extrapolation. Empty table -> fallback.
inline double interp1_clamped(const std::vector<double>& x, const std::vector<double>& y,
                              double q, double fallback) {
    if (x.empty() || x.size() != y.size()) return fallback;
    if (q <= x.front()) return y.front();
    if (q >= x.back()) return y.back();
    const auto it = std::upper_bound(x.begin(), x.end(), q);
    const size_t k = static_cast<size_t>(it - x.begin());
    const double t = (q - x[k - 1]) / (x[k] - x[k - 1]);
    return y[k - 1] + t * (y[k] - y[k - 1]);
}

/// 車輪インデックス
enum WheelIndex : int {
    FL = 0,  // Front Left
    FR = 1,  // Front Right
    RL = 2,  // Rear Left
    RR = 3   // Rear Right
};

/// 質量・慣性パラメータ
struct MassParams {
    double m_total = 282.0;     // 全装備重量 [kg] = m_sprung + Σm_unsprung
    double m_sprung = 230.0;    // ばね上質量 [kg]
    std::array<double, 4> m_unsprung = {12.0, 12.0, 14.0, 14.0};  // ばね下質量 [kg]
    
    // 重心位置
    double wheelbase = 1.55;    // ホイールベース [m]
    double a = 0.73;            // 前軸からCG [m]
    double b = 0.82;            // 後軸からCG [m]
    double track_f = 1.20;      // フロントトレッド [m]
    double track_r = 1.18;      // リアトレッド [m]
    double h_cg = 0.28;         // 重心高 [m]
    
    // 慣性モーメント
    double Ixx = 32.0;          // ロール慣性 [kg·m²]
    double Iyy = 95.0;          // ピッチ慣性 [kg·m²]
    double Izz = 85.0;          // ヨー慣性 [kg·m²]
    double Ixz = -3.0;          // 慣性乗積 [kg·m²]
    
    // 車輪慣性
    double Iw = 0.5;            // 車輪回転慣性 [kg·m²]（全輪共通、旧モデル互換）
    // 輪別慣性 [FL,FR,RL,RR]（0 の輪は Iw を使用）。JSON の mass.Iw が配列ならここへ入る
    std::array<double, 4> Iw_corner = {0.0, 0.0, 0.0, 0.0};
};

/// サスペンションパラメータ
struct SuspensionParams {
    // ばね
    std::array<double, 4> k_spring = {25000, 25000, 30000, 30000};  // [N/m]
    std::array<double, 4> motion_ratio = {0.85, 0.85, 0.90, 0.90};
    // MR(travel) テーブル（任意, #10）。travel = 平衡からのホイールストローク [m]（+ 圧縮）。
    // 空なら一定 MR。ばね力は F_wheel = MR(x)*(F_static/MR(0) + k*∫_0^x MR)。
    std::vector<double> motion_ratio_table_travel = {};
    std::array<std::vector<double>, 4> motion_ratio_table = {};
    // true: ダンパー(Bouc-Wen)/LuGre/バンプストップも素子座標 (MR*x, MR*v) で評価し MR 倍で
    // ホイールへ戻す（c0, bump_stop_* は素子側の値）。false: 旧挙動（ホイール座標のまま）
    bool mr_applies_to_dampers = true;
    
    // アンチロールバー
    // ARB ロール剛性 [Nm/rad]。ホイールには kc_*.arb_motion_ratio^2 倍で効く（#10）。
    // arb_motion_ratio = 1（既定）ならアクスル等価値として従来どおり
    double k_arb_f = 500.0;     // フロントARB [Nm/rad]
    double k_arb_r = 300.0;     // リアARB [Nm/rad]
    
    // Bouc-Wenダンパー（設計書セクション5.3 / 13.3準拠）
    // F_damper = c0 * v + k0 * x + alpha * z
    // dz/dt = A * v - beta * |v| * z * |z|^(n-1) - gamma * v * |z|^n
    double c0 = 1500.0;         // 粘性減衰係数 [Ns/m]
    double k0 = 0.0;            // 剛性係数 [N/m]（通常0、スプリング別計算）
    double bw_alpha = 1000.0;   // ヒステリシス強度 [N] (500-2000)
    double bw_A = 100.0;        // 形状パラメータ (典型値: 100)
    double bw_beta = 300.0;     // 形状パラメータ (典型値: 300)
    double bw_gamma = 0.0;      // 形状パラメータ (典型値: 0)
    double bw_n = 1.0;          // 形状指数 (典型値: 1)

    // Optional measured damper dyno LUT (#11). Velocity is at the damper
    // element [m/s, + compression], force is element force [N, + upward on
    // body]; one table per corner. Empty = use the Bouc-Wen model above.
    std::vector<double> damper_lut_velocity;
    std::array<std::vector<double>, 4> damper_lut_force;
    
    // LuGre摩擦モデル（設計書セクション5.4準拠）
    // ⚠️ 安定性制約（ガイド §7.5, §12.6）:
    //    σ_0 < 2/Δt = 2000 N/m (Δt=1ms の場合)
    //    超過する場合はサブステップ積分を使用
    double lugre_sigma0 = 1000.0;   // 毛剛性 [N/m] (安定性考慮済み: < 2000)
    double lugre_sigma1 = 10.0;     // 毛減衰 [Ns/m]
    double lugre_sigma2 = 0.0;      // 粘性摩擦 [Ns/m]
    double lugre_Fc = 20.0;         // クーロン摩擦 [N]
    double lugre_Fs = 30.0;         // 静止摩擦 [N]
    double lugre_vs = 0.01;         // ストライベック速度 [m/s]
    int lugre_substeps = 1;         // サブステップ数（1=通常、10=0.1ms刻み）
    
    // バンプストップ
    double bump_stop_gap = 0.03;    // 圧縮側ギャップ [m]
    double bump_stop_k = 100000.0;  // 圧縮側剛性 [N/m]
    double bump_stop_gap_rebound = -1.0;  // 伸び側ギャップ [m]（<0 → bump_stop_gap と同値）
    double bump_stop_k_rebound = -1.0;    // 伸び側剛性 [N/m]（<0 → bump_stop_k と同値）

    // ジオメトリ（ガイド §2.4, §7）
    // キャンバーゲイン: サスペンションストローク1mあたりのキャンバー角変化 [rad/m]
    // 負の値: バンプ（縮み）でネガティブキャンバー増加
    std::array<double, 4> camber_gain = {-0.4, -0.4, -0.35, -0.35};  // [rad/m] (約-23deg/m)

    // 静的キャンバー角（設計値） [rad]
    std::array<double, 4> static_camber = {-0.017, -0.017, -0.015, -0.015};  // [rad] (約-1deg)

    // 静的トー角 [rad] — 各輪の幾何学的ヨー角
    // 正: 右向き（FL:トーイン, FR:トーアウト）、負: 左向き
    // 対称トーでは左右の符号が逆になる
    std::array<double, 4> static_toe = {-0.002, 0.002, 0.001, -0.001};  // [rad] F:0.1deg toe-out, R:0.05deg toe-in

    // POU（Pushrod on Upright）パラメータ
    // 操舵角δに対するジャッキアップ変位 [m]: Δz = c1·δ + c2·δ²
    // 正値: スプリング圧縮方向（車体上昇）
    std::array<double, 4> pou_jack_c1 = {0, 0, 0, 0};  // [m/rad]
    std::array<double, 4> pou_jack_c2 = {0, 0, 0, 0};  // [m/rad²]

    // 操舵角δに対するキャンバー変化 [rad]: Δγ = c1·δ + c2·δ²
    // per-wheel係数（ミラー対称の場合、FL=-FR: 例 [-0.05, 0.05, 0, 0]）
    std::array<double, 4> pou_camber_c1 = {0, 0, 0, 0};  // [rad/rad]
    std::array<double, 4> pou_camber_c2 = {0, 0, 0, 0};  // [rad/rad²]

    // POU物理ハードポイント座標（参照/デバッグ用、シミュレーション計算には使用しない）
    // MATLAB最適化からのエクスポートデータ。全ゼロ=未設定
    struct POUHardpoint {
        std::array<double, 3> M_pivot_chassis = {0, 0, 0};   // M-pivot (chassis frame) [m]
        std::array<double, 3> M_pivot_upright = {0, 0, 0};   // M-pivot (upright frame) [m]
        std::array<double, 3> bellcrank_pivot = {0, 0, 0};    // Bellcrank pivot [m]
        double pushrod_length = 0.0;                          // Nominal pushrod length [m]
        double installation_ratio = 0.0;                      // Mechanical advantage [-]
        bool valid = false;                                   // True if data was loaded
    };
    std::array<POUHardpoint, 4> pou_hardpoints;  // Per-wheel [FL, FR, RL, RR]

    // Kinematics & Compliance schema
    struct KCCornerParams {
        struct Hardpoints {
            std::array<double, 3> upper_arm_chassis_fore = {0, 0, 0};
            std::array<double, 3> upper_arm_chassis_aft = {0, 0, 0};
            std::array<double, 3> lower_arm_chassis_fore = {0, 0, 0};
            std::array<double, 3> lower_arm_chassis_aft = {0, 0, 0};
            std::array<double, 3> upper_ball_joint = {0, 0, 0};
            std::array<double, 3> lower_ball_joint = {0, 0, 0};
            std::array<double, 3> wheel_center = {0, 0, 0};
            std::array<double, 3> contact_patch = {0, 0, 0};
            std::array<double, 3> steering_arm_outer = {0, 0, 0};
            std::array<double, 3> tie_rod_inner = {0, 0, 0};
            std::array<double, 3> tie_rod_outer = {0, 0, 0};
            std::array<double, 3> pushrod_chassis = {0, 0, 0};
            std::array<double, 3> pushrod_upright = {0, 0, 0};
            std::array<double, 3> rocker_pivot = {0, 0, 0};
            std::array<double, 3> damper_upper = {0, 0, 0};
            std::array<double, 3> damper_lower = {0, 0, 0};
            std::array<double, 3> arb_link_chassis = {0, 0, 0};
            std::array<double, 3> arb_link_wheel = {0, 0, 0};
        };

        struct KinematicSurrogate {
            std::array<double, 3> camber_vs_travel = {0, 0, 0}; // c0 + c1*x + c2*x^2 [rad]
            std::array<double, 3> toe_vs_travel = {0, 0, 0};    // c0 + c1*x + c2*x^2 [rad]
            std::array<double, 3> camber_vs_steer = {0, 0, 0};  // c0 + c1*d + c2*d^2 [rad]
            std::array<double, 3> toe_vs_steer = {0, 0, 0};     // c0 + c1*d + c2*d^2 [rad]
            double roll_camber_gain = 0.0;   // [rad/rad]
            double roll_steer_gain = 0.0;    // [rad/rad]
            double jacking_force_per_fy = 0.0; // [N/N]
        };

        struct ComplianceSurrogate {
            std::array<double, 6> toe_per_load = {0, 0, 0, 0, 0, 0};      // Fx,Fy,Fz,Mx,My,Mz -> toe [rad/unit]
            std::array<double, 6> camber_per_load = {0, 0, 0, 0, 0, 0};   // Fx,Fy,Fz,Mx,My,Mz -> camber [rad/unit]
            std::array<double, 6> caster_per_load = {0, 0, 0, 0, 0, 0};   // Fx,Fy,Fz,Mx,My,Mz -> caster [rad/unit]
            std::array<double, 6> track_per_load = {0, 0, 0, 0, 0, 0};    // Fx,Fy,Fz,Mx,My,Mz -> track change [m/unit]
            std::array<double, 6> wheelbase_per_load = {0, 0, 0, 0, 0, 0}; // Fx,Fy,Fz,Mx,My,Mz -> wheelbase change [m/unit]
        };

        /// Dense K&C sweep tables (#8). All lengths are SI; empty = use the
        /// quadratic surrogate (legacy artifacts).
        struct KinematicSamples {
            std::vector<double> travel_m;
            std::vector<double> camber_rad, toe_rad, caster_rad, kpi_rad;
            std::vector<double> track_change_m, rc_height_m, jacking_force_per_fy;
            std::vector<double> anti_brake, anti_drive;
            std::vector<double> roll_rad, roll_camber_residual_rad, roll_toe_residual_rad,
                roll_center_height_m;
            std::vector<double> steer_rad, steer_rack_m, steer_camber_rad, steer_toe_rad;
        };

        Hardpoints hardpoints;
        KinematicSurrogate kinematics;
        ComplianceSurrogate compliance;
        KinematicSamples samples;
        double caster = 0.0;           // [rad]
        double scrub_radius = 0.0;     // [m]
        double mechanical_trail = 0.0; // [m]
        double caster_trail = 0.0;     // [m]
        bool valid = false;
    };

    struct KCAxleParams {
        double roll_center_height = 0.0;         // [m]
        double roll_center_height_per_travel = 0.0; // [m/m]
        double roll_center_height_per_roll = 0.0;   // [m/rad]
        double anti_brake_pct = 0.0;             // [0-1]
        double anti_drive_pct = 0.0;             // [0-1]
        double arb_motion_ratio = 1.0;           // [-]
    };

    std::array<KCCornerParams, 4> kc_corners;
    KCAxleParams kc_front;
    KCAxleParams kc_rear;

    // フレーム柔軟性パラメータ（Phase A1）
    // デフォルト値は剛体モード（K_chassis = 1e6で後方互換性を確保）
    double K_chassis = 1e6;      // フレームねじり剛性 [Nm/deg]（典型値: 1000-3500）
    double C_chassis = 5.0;      // フレーム構造減衰 [Nm·s/rad]（典型値: 3-10）
    double I_twist = 1.5;        // フレームねじり慣性 [kg·m²]（典型値: 0.5-3.0）
};

/// タイヤパラメータ
struct TireParams {
    struct RelaxationParams {
        double Lx_base = 0.15;          // [m]
        double Ly_base = 0.25;          // [m]
        double Fz_exponent = 0.4;       // [-]
        double temp_coeff_per_c = -0.003; // [1/degC]
        double temp_ref_c = 40.0;       // [degC]
        double min_Lx = 0.02;           // [m]
        double max_Lx = 0.50;           // [m]
        double min_Ly = 0.05;           // [m]
        double max_Ly = 1.00;           // [m]
        bool enabled = true;
    };

    double radius = 0.228;      // タイヤ半径 [m]
    double mu_x_max = 1.6;
    double mu_y_max = 2.0;  // レーシングタイヤ（実測ベース）
    double Fz_nom = 1500.0;
    double Lx = 0.15;           // 縦緩和長 [m]
    double Ly = 0.25;           // 横緩和長 [m]
    double Cx = 80000.0;        // 縦剛性 [N/m]
    double Cy = 22000.0;        // 横剛性 [N/rad]
    double v_N = 0.5;           // 仮想速度 [m/s]
    double k_tire = 200000.0;   // タイヤ垂直剛性 [N/m]
    double c_tire = 500.0;      // タイヤ垂直減衰 [Ns/m]
    double dmu_dFz = -0.0001;   // 荷重感度 [1/N]
    double pneumatic_trail = 0.03; // SAT近似用空気トレール [m]
    // 転がり抵抗係数 [-]（#12）。既定 0.015 は乗用/レーシングタイヤの舗装路典型値
    // 0.010-0.020（Gillespie, Fundamentals of Vehicle Dynamics, ch.4）。実測コーストダウンで置換すること
    double C_rr = 0.015;
#ifdef FSAE_TEST_TIRE_MODEL_TYPE
    std::string model_type = FSAE_TEST_TIRE_MODEL_TYPE; // test-only TMeasy fallback
#else
    std::string model_type = "pinn";       // production runtime requires pinn / nn
#endif
    std::string model_dir = "output/tire_models/Hoosier_18p0X6p0-10_R20_Rim6"; // モデルディレクトリ（PINN）
    std::string model_manifest = "output/tire_models/Hoosier_18p0X6p0-10_R20_Rim6/manifest.json"; // manifest.json パス（PINN）
    std::string model_name = "Hoosier_18p0X6p0-10_R20_Rim6"; // EasyStart 表示用
    bool pinn_use_camber_input = false;    // true only for PINN exports with measured camber coverage
    bool pinn_ttc_lateral_sign = true;     // convert TTC Fy/Mz signs to the vehicle/TMeasy convention
    double pinn_eval_rate_hz = 500.0;      // simulation-time ZOH; 0 restores every RK4/K&C evaluation
    double ambient_temperature_c = 25.0;   // 外気温 [°C]
    RelaxationParams relaxation;
};

/// ステアリングパラメータ
struct SteeringParams {
    double ackermann = 0.7;          // アッカーマン率 (0-1)
    double max_angle = 0.52;         // 最大舵角 [rad] (~30deg)
    double stiffness = 5000.0;       // ステアリング系回転剛性 K_steer [Nm/rad]
                                     // (ガイド §10.3: コンプライアンスステア用)
                                     // 典型値: 3000-8000 Nm/rad
                                     // 高剛性 → 応答性向上、低剛性 → セルフセンタリング強化

    // ステアリングトルクモデル（Phase A2）
    double I_steer = 0.05;           // ステアリング慣性 [kg·m²]（典型値: 0.02-0.08）
    double R_steer = 4.5;            // 総ステアリング比 [-]（典型値: 3-6）
    double T_fric = 1.5;             // クーロン摩擦トルク [Nm]（典型値: 0.5-3.0）
    double C_steer_torque = 0.5;     // 粘性減衰係数 [Nm·s/rad]（典型値: 0.1-1.0）
    double K_return = 0.0;           // 復元ばね剛性 [Nm/rad]（通常0、セルフセンタリングはSATで実現）
    double T_driver_max = 25.0;      // 最大ドライバートルク [Nm]（片手: 15-25, 両手: 30-50）
    double T_driver_rate_max = 0.0;  // ドライバートルク指令レート上限 [Nm/s]（0=無制限）
    bool torque_mode = false;        // トルクモード有効化（false=角度モード、後方互換性）
};

/// パワートレインパラメータ
struct PowertrainParams {
    // エンジントルクカーブ（RPM→トルク）
    std::vector<double> engine_rpm_points = {1000, 5000, 9000, 12500, 14500};
    std::vector<double> engine_torque_curve = {35, 48, 60, 63, 55};  // [Nm]
    
    // ギア比（1速〜6速）
    std::vector<double> gear_ratios = {2.75, 2.00, 1.63, 1.43, 1.30, 1.21};
    double primary_ratio = 1.90;    // 一次減速比
    double final_ratio = 3.50;      // 最終減速比（スプロケット）
    double efficiency = 0.90;       // 伝達効率
    // 反映慣性（#4）。0 = 無効（実測/カタログ値が無い限り推定値を入れない）
    double I_engine = 0.0;          // クランク軸まわり慣性 [kg·m²]
    double I_gearbox = 0.0;         // ミッション出力軸まわり慣性 [kg·m²]
    double shift_time_s = 0.0;      // 変速中の駆動トルク遮断時間 [s]

    // Legacy 3/6/8DOF models (not the 14DOF path)
    double max_torque = 63.0;       // 最大エンジントルク [Nm]
    double gear_ratio = 3.0;        // 総減速比（旧形式）

    // LSD（Salisbury式）パラメータ
    double lsd_preload = 40.0;           // プリロードトルク [Nm] (20-80)
    double lsd_ramp_accel = 35.0;        // 加速側ランプ角 [deg] (30-45: 小→ロック強)
    double lsd_ramp_decel = 50.0;        // 減速側ランプ角 [deg] (45-60: 進入安定性)
    double lsd_effective_radius = 0.04;  // クラッチ有効半径 [m] (40mm標準)
    double lsd_clutch_friction = 0.10;   // クラッチ摩擦係数 [-] (0.08-0.12)
    int lsd_num_plates = 8;              // クラッチプレート数 [-] (4-12枚)
};

/// ブレーキパラメータ
struct BrakeParams {
    double max_pressure = 8e6;  // 最大油圧 [Pa]（バランス中立時のライン圧 @ pedal=1）
    // フロント配分 = 前後マスターシリンダ圧比 (#36): p_front = 2*balance*p,
    // p_rear = 2*(1-balance)*p。0.5 で前後同圧（中立）、幾何学的な前後差は
    // caliper_area*effective_radius が受け持つ（バランスバー + 同径MCの物理）。
    double balance = 0.55;
    // ペダル→ライン圧テーブル [0-1] -> [Pa]（空なら pedal*max_pressure）
    std::vector<double> pedal_table = {};
    std::vector<double> pressure_table = {};
    double pressure_lag_s = 0.0;       // 1次遅れ時定数 τ [s]（0=遅れ無し）
    double omega_smooth = 0.5;         // T = -T_max*tanh(ω/ω_s) の ω_s [rad/s]
    double mu_pad = 0.45;       // パッド摩擦係数
    std::array<double, 4> caliper_area = {0.002, 0.002, 0.0015, 0.0015};  // [m²]
    std::array<double, 4> effective_radius = {0.10, 0.10, 0.09, 0.09};    // [m]
};

/// 空力パラメータ
struct AeroParams {
    double CL_A = 2.8;          // ダウンフォース係数×面積 [m²]
    double CD_A = 0.95;         // ドラッグ係数×面積 [m²]
    double aero_balance = 0.45; // フロント空力配分
    double h_aero = 0.35;       // 空力中心高さ [m]
    double rho = 1.225;         // 空気密度 [kg/m³]
    std::string map_mode = "coefficient"; // coefficient / direct_load
    std::string aero_map_file = "";
    double reference_speed = 15.0;          // [m/s]
    double reference_density = 1.225;       // [kg/m³]
    double nominal_front_ride_height_mm = 40.0;
    double nominal_rear_ride_height_mm = 55.0;
    double nominal_roll_rad = 0.0;
    double nominal_yaw_rad = 0.0;
    double Fx_ref_total = -80.0;            // [N] at Vref
    double Fy_ref_total = 0.0;              // [N] at Vref
    double Fz_ref_total = -350.0;           // [N] at Vref
    double Mx_ref_total = 0.0;              // [Nm] at Vref
    double My_ref_total = 0.0;              // [Nm] at Vref
    double Mz_ref_total = 0.0;              // [Nm] at Vref
    bool planner_uses_aero_map = true;
};

/// 14DOF車両パラメータ（統合）
struct VehicleParams {
    MassParams mass;
    SuspensionParams suspension;
    TireParams tire;
    SteeringParams steering;
    PowertrainParams powertrain;
    BrakeParams brake;
    AeroParams aero;
    
    /// JSONオブジェクトからパース
    static void parse_json(const nlohmann::json& j, VehicleParams& p) {
        const nlohmann::json& config =
            (j.contains("params") && j["params"].is_object()) ? j["params"] : j;

        auto load_vec3 = [](const nlohmann::json& src, std::array<double, 3>& dst) {
            if (src.is_array() && src.size() >= 3) {
                for (int i = 0; i < 3; ++i) dst[i] = src[i];
            }
        };

        auto load_arr3 = [](const nlohmann::json& src, std::array<double, 3>& dst) {
            if (src.is_array() && src.size() >= 3) {
                for (int i = 0; i < 3; ++i) dst[i] = src[i];
            }
        };

        auto load_arr6 = [](const nlohmann::json& src, std::array<double, 6>& dst) {
            if (src.is_array() && src.size() >= 6) {
                for (int i = 0; i < 6; ++i) dst[i] = src[i];
            }
        };

        // 質量
        if (config.contains("mass")) {
            auto& m = config["mass"];
            if (m.contains("total_kg")) p.mass.m_total = m["total_kg"];
            if (m.contains("m_total")) p.mass.m_total = m["m_total"];
            if (m.contains("sprung_kg")) p.mass.m_sprung = m["sprung_kg"];
            if (m.contains("m_sprung")) p.mass.m_sprung = m["m_sprung"];
            if (m.contains("unsprung_kg") && m["unsprung_kg"].is_array()) {
                for(int i=0; i<4; ++i) p.mass.m_unsprung[i] = m["unsprung_kg"][i];
            }
            if (m.contains("m_unsprung") && m["m_unsprung"].is_array()) {
                for(int i=0; i<4; ++i) p.mass.m_unsprung[i] = m["m_unsprung"][i];
            }
            if (m.contains("wheelbase_m")) p.mass.wheelbase = m["wheelbase_m"];
            if (m.contains("wheelbase")) p.mass.wheelbase = m["wheelbase"];
            if (m.contains("track_front_m")) p.mass.track_f = m["track_front_m"];
            if (m.contains("track_f")) p.mass.track_f = m["track_f"];
            if (m.contains("track_rear_m")) p.mass.track_r = m["track_rear_m"];
            if (m.contains("track_r")) p.mass.track_r = m["track_r"];
            if (m.contains("cg_height_m")) p.mass.h_cg = m["cg_height_m"];
            if (m.contains("h_cg")) p.mass.h_cg = m["h_cg"];
            if (m.contains("a_m")) p.mass.a = m["a_m"];
            if (m.contains("a")) p.mass.a = m["a"];
            if (m.contains("b_m")) p.mass.b = m["b_m"];
            if (m.contains("b")) p.mass.b = m["b"];
            if (m.contains("Ixx_kgm2")) p.mass.Ixx = m["Ixx_kgm2"];
            if (m.contains("Ixx")) p.mass.Ixx = m["Ixx"];
            if (m.contains("Iyy_kgm2")) p.mass.Iyy = m["Iyy_kgm2"];
            if (m.contains("Iyy")) p.mass.Iyy = m["Iyy"];
            if (m.contains("Izz_kgm2")) p.mass.Izz = m["Izz_kgm2"];
            if (m.contains("Izz")) p.mass.Izz = m["Izz"];
            if (m.contains("Ixz_kgm2")) p.mass.Ixz = m["Ixz_kgm2"];
            if (m.contains("Ixz")) p.mass.Ixz = m["Ixz"];
            for (const char* key : {"Iw_kgm2", "Iw"}) {
                if (!m.contains(key)) continue;
                if (m[key].is_array()) {
                    double sum = 0.0;
                    for (int i = 0; i < 4; ++i) sum += p.mass.Iw_corner[i] = m[key][i];
                    p.mass.Iw = 0.25 * sum;  // legacy scalar consumers
                } else {
                    p.mass.Iw = m[key];
                }
            }
        }

        // サスペンション
        if (config.contains("suspension")) {
            auto& s = config["suspension"];
            if (s.contains("k_spring") && s["k_spring"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.k_spring[i] = s["k_spring"][i];
            }
            if (s.contains("motion_ratio") && s["motion_ratio"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.motion_ratio[i] = s["motion_ratio"][i];
            }
            if (s.contains("k_arb_f")) p.suspension.k_arb_f = s["k_arb_f"];
            if (s.contains("k_arb_r")) p.suspension.k_arb_r = s["k_arb_r"];
            
            // ダンパー (Bouc-Wen)
            if (s.contains("c0")) p.suspension.c0 = s["c0"];
            if (s.contains("k0")) p.suspension.k0 = s["k0"];
            if (s.contains("bw_alpha")) p.suspension.bw_alpha = s["bw_alpha"];
            if (s.contains("bw_A")) p.suspension.bw_A = s["bw_A"];
            if (s.contains("bw_beta")) p.suspension.bw_beta = s["bw_beta"];
            if (s.contains("bw_gamma")) p.suspension.bw_gamma = s["bw_gamma"];
            if (s.contains("bw_n")) p.suspension.bw_n = s["bw_n"];
            if (s.contains("damper_lut")) {
                auto& dl = s["damper_lut"];
                if (dl.contains("velocity_mps") && dl["velocity_mps"].is_array()) {
                    p.suspension.damper_lut_velocity.clear();
                    for (const auto& v : dl["velocity_mps"]) {
                        p.suspension.damper_lut_velocity.push_back(v.get<double>());
                    }
                }
                if (dl.contains("force_N") && dl["force_N"].is_object()) {
                    const std::array<std::string, 4> keys = {"FL", "FR", "RL", "RR"};
                    for (int i = 0; i < 4; ++i) {
                        if (!dl["force_N"].contains(keys[i])) continue;
                        auto& dst = p.suspension.damper_lut_force[i];
                        dst.clear();
                        for (const auto& f : dl["force_N"][keys[i]]) {
                            dst.push_back(f.get<double>());
                        }
                    }
                }
            }
            if (s.contains("lugre_sigma0")) p.suspension.lugre_sigma0 = s["lugre_sigma0"];
            if (s.contains("lugre_sigma1")) p.suspension.lugre_sigma1 = s["lugre_sigma1"];
            if (s.contains("lugre_sigma2")) p.suspension.lugre_sigma2 = s["lugre_sigma2"];
            if (s.contains("lugre_Fc")) p.suspension.lugre_Fc = s["lugre_Fc"];
            if (s.contains("lugre_Fs")) p.suspension.lugre_Fs = s["lugre_Fs"];
            if (s.contains("lugre_vs")) p.suspension.lugre_vs = s["lugre_vs"];
            if (s.contains("lugre_substeps")) p.suspension.lugre_substeps = s["lugre_substeps"];
             
            // バンプストップ & ジオメトリ
            if (s.contains("bump_stop_gap")) p.suspension.bump_stop_gap = s["bump_stop_gap"];
            if (s.contains("bump_stop_k")) p.suspension.bump_stop_k = s["bump_stop_k"];
            if (s.contains("bump_stop_gap_rebound")) p.suspension.bump_stop_gap_rebound = s["bump_stop_gap_rebound"];
            if (s.contains("bump_stop_k_rebound")) p.suspension.bump_stop_k_rebound = s["bump_stop_k_rebound"];
            if (s.contains("mr_applies_to_dampers")) p.suspension.mr_applies_to_dampers = s["mr_applies_to_dampers"];
            if (s.contains("motion_ratio_table")) {
                auto& mt = s["motion_ratio_table"];
                p.suspension.motion_ratio_table_travel = mt.at("travel").get<std::vector<double>>();
                const std::array<std::string, 4> keys = {"FL", "FR", "RL", "RR"};
                for (int i = 0; i < 4; ++i) {
                    p.suspension.motion_ratio_table[i] = mt.at(keys[i]).get<std::vector<double>>();
                    if (p.suspension.motion_ratio_table[i].size() != p.suspension.motion_ratio_table_travel.size()) {
                        throw std::runtime_error("suspension.motion_ratio_table." + keys[i] + " size != travel size");
                    }
                }
            }
            if (s.contains("camber_gain") && s["camber_gain"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.camber_gain[i] = s["camber_gain"][i];
            }
            if (s.contains("static_camber") && s["static_camber"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.static_camber[i] = s["static_camber"][i];
            }
            if (s.contains("static_toe") && s["static_toe"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.static_toe[i] = s["static_toe"][i];
            }

            // POU（Pushrod on Upright）
            if (s.contains("pou_jack_c1") && s["pou_jack_c1"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.pou_jack_c1[i] = s["pou_jack_c1"][i];
            }
            if (s.contains("pou_jack_c2") && s["pou_jack_c2"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.pou_jack_c2[i] = s["pou_jack_c2"][i];
            }
            if (s.contains("pou_camber_c1") && s["pou_camber_c1"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.pou_camber_c1[i] = s["pou_camber_c1"][i];
            }
            if (s.contains("pou_camber_c2") && s["pou_camber_c2"].is_array()) {
                for(int i=0; i<4; ++i) p.suspension.pou_camber_c2[i] = s["pou_camber_c2"][i];
            }

            // フレーム柔軟性（Phase A1）
            if (s.contains("K_chassis")) p.suspension.K_chassis = s["K_chassis"];
            if (s.contains("C_chassis")) p.suspension.C_chassis = s["C_chassis"];
            if (s.contains("I_twist")) p.suspension.I_twist = s["I_twist"];

            if (s.contains("kc")) {
                auto& kc = s["kc"];

                auto parse_axle = [](const nlohmann::json& src, SuspensionParams::KCAxleParams& dst) {
                    if (src.contains("roll_center_height")) dst.roll_center_height = src["roll_center_height"];
                    if (src.contains("roll_center_height_per_travel")) {
                        dst.roll_center_height_per_travel = src["roll_center_height_per_travel"];
                    }
                    if (src.contains("roll_center_height_per_roll")) {
                        dst.roll_center_height_per_roll = src["roll_center_height_per_roll"];
                    }
                    if (src.contains("anti_brake_pct")) dst.anti_brake_pct = src["anti_brake_pct"];
                    if (src.contains("anti_drive_pct")) dst.anti_drive_pct = src["anti_drive_pct"];
                    if (src.contains("arb_motion_ratio")) dst.arb_motion_ratio = src["arb_motion_ratio"];
                };

                if (kc.contains("front")) parse_axle(kc["front"], p.suspension.kc_front);
                if (kc.contains("rear")) parse_axle(kc["rear"], p.suspension.kc_rear);

                if (kc.contains("corners")) {
                    auto& corners = kc["corners"];
                    const std::array<std::string, 4> wheel_keys = {"FL", "FR", "RL", "RR"};
                    for (int i = 0; i < 4; ++i) {
                        if (!corners.contains(wheel_keys[i])) continue;
                        auto& src = corners[wheel_keys[i]];
                        auto& dst = p.suspension.kc_corners[i];

                        if (src.contains("caster")) dst.caster = src["caster"];
                        if (src.contains("scrub_radius")) dst.scrub_radius = src["scrub_radius"];
                        if (src.contains("mechanical_trail")) dst.mechanical_trail = src["mechanical_trail"];
                        if (src.contains("caster_trail")) dst.caster_trail = src["caster_trail"];

                        if (src.contains("kinematics")) {
                            auto& kin = src["kinematics"];
                            if (kin.contains("camber_vs_travel")) load_arr3(kin["camber_vs_travel"], dst.kinematics.camber_vs_travel);
                            if (kin.contains("toe_vs_travel")) load_arr3(kin["toe_vs_travel"], dst.kinematics.toe_vs_travel);
                            if (kin.contains("camber_vs_steer")) load_arr3(kin["camber_vs_steer"], dst.kinematics.camber_vs_steer);
                            if (kin.contains("toe_vs_steer")) load_arr3(kin["toe_vs_steer"], dst.kinematics.toe_vs_steer);
                            if (kin.contains("roll_camber_gain")) dst.kinematics.roll_camber_gain = kin["roll_camber_gain"];
                            if (kin.contains("roll_steer_gain")) dst.kinematics.roll_steer_gain = kin["roll_steer_gain"];
                            if (kin.contains("jacking_force_per_fy")) dst.kinematics.jacking_force_per_fy = kin["jacking_force_per_fy"];
                        }

                        if (src.contains("compliance")) {
                            auto& comp = src["compliance"];
                            if (comp.contains("toe_per_load")) load_arr6(comp["toe_per_load"], dst.compliance.toe_per_load);
                            if (comp.contains("camber_per_load")) load_arr6(comp["camber_per_load"], dst.compliance.camber_per_load);
                            if (comp.contains("caster_per_load")) load_arr6(comp["caster_per_load"], dst.compliance.caster_per_load);
                            if (comp.contains("track_per_load")) load_arr6(comp["track_per_load"], dst.compliance.track_per_load);
                            if (comp.contains("wheelbase_per_load")) load_arr6(comp["wheelbase_per_load"], dst.compliance.wheelbase_per_load);
                        }

                        if (src.contains("sample_values")) {
                            auto& sv = src["sample_values"];
                            auto load_vec = [](const nlohmann::json& node, std::vector<double>& dst) {
                                dst.clear();
                                if (!node.is_array()) return;
                                dst.reserve(node.size());
                                for (const auto& v : node) dst.push_back(v.get<double>());
                            };
                            if (sv.contains("travel_m")) load_vec(sv["travel_m"], dst.samples.travel_m);
                            if (sv.contains("camber_rad")) load_vec(sv["camber_rad"], dst.samples.camber_rad);
                            if (sv.contains("toe_rad")) load_vec(sv["toe_rad"], dst.samples.toe_rad);
                            if (sv.contains("caster_rad")) load_vec(sv["caster_rad"], dst.samples.caster_rad);
                            if (sv.contains("kpi_rad")) load_vec(sv["kpi_rad"], dst.samples.kpi_rad);
                            if (sv.contains("track_change_m")) load_vec(sv["track_change_m"], dst.samples.track_change_m);
                            if (sv.contains("rc_height_m")) load_vec(sv["rc_height_m"], dst.samples.rc_height_m);
                            if (sv.contains("jacking_force_per_fy")) load_vec(sv["jacking_force_per_fy"], dst.samples.jacking_force_per_fy);
                            if (sv.contains("anti_brake")) load_vec(sv["anti_brake"], dst.samples.anti_brake);
                            if (sv.contains("anti_drive")) load_vec(sv["anti_drive"], dst.samples.anti_drive);
                            if (sv.contains("roll_rad")) load_vec(sv["roll_rad"], dst.samples.roll_rad);
                            if (sv.contains("roll_camber_residual_rad")) load_vec(sv["roll_camber_residual_rad"], dst.samples.roll_camber_residual_rad);
                            if (sv.contains("roll_toe_residual_rad")) load_vec(sv["roll_toe_residual_rad"], dst.samples.roll_toe_residual_rad);
                            if (sv.contains("roll_center_height_m")) load_vec(sv["roll_center_height_m"], dst.samples.roll_center_height_m);
                            if (sv.contains("steer_rad")) load_vec(sv["steer_rad"], dst.samples.steer_rad);
                            if (sv.contains("steer_rack_m")) load_vec(sv["steer_rack_m"], dst.samples.steer_rack_m);
                            if (sv.contains("steer_camber_rad")) load_vec(sv["steer_camber_rad"], dst.samples.steer_camber_rad);
                            if (sv.contains("steer_toe_rad")) load_vec(sv["steer_toe_rad"], dst.samples.steer_toe_rad);
                        }

                        if (src.contains("hardpoints")) {
                            auto& hp = src["hardpoints"];
                            if (hp.contains("upper_arm_chassis_fore")) load_vec3(hp["upper_arm_chassis_fore"], dst.hardpoints.upper_arm_chassis_fore);
                            if (hp.contains("upper_arm_chassis_aft")) load_vec3(hp["upper_arm_chassis_aft"], dst.hardpoints.upper_arm_chassis_aft);
                            if (hp.contains("lower_arm_chassis_fore")) load_vec3(hp["lower_arm_chassis_fore"], dst.hardpoints.lower_arm_chassis_fore);
                            if (hp.contains("lower_arm_chassis_aft")) load_vec3(hp["lower_arm_chassis_aft"], dst.hardpoints.lower_arm_chassis_aft);
                            if (hp.contains("upper_ball_joint")) load_vec3(hp["upper_ball_joint"], dst.hardpoints.upper_ball_joint);
                            if (hp.contains("lower_ball_joint")) load_vec3(hp["lower_ball_joint"], dst.hardpoints.lower_ball_joint);
                            if (hp.contains("wheel_center")) load_vec3(hp["wheel_center"], dst.hardpoints.wheel_center);
                            if (hp.contains("contact_patch")) load_vec3(hp["contact_patch"], dst.hardpoints.contact_patch);
                            if (hp.contains("steering_arm_outer")) load_vec3(hp["steering_arm_outer"], dst.hardpoints.steering_arm_outer);
                            if (hp.contains("tie_rod_inner")) load_vec3(hp["tie_rod_inner"], dst.hardpoints.tie_rod_inner);
                            if (hp.contains("tie_rod_outer")) load_vec3(hp["tie_rod_outer"], dst.hardpoints.tie_rod_outer);
                            if (hp.contains("pushrod_chassis")) load_vec3(hp["pushrod_chassis"], dst.hardpoints.pushrod_chassis);
                            if (hp.contains("pushrod_upright")) load_vec3(hp["pushrod_upright"], dst.hardpoints.pushrod_upright);
                            if (hp.contains("rocker_pivot")) load_vec3(hp["rocker_pivot"], dst.hardpoints.rocker_pivot);
                            if (hp.contains("damper_upper")) load_vec3(hp["damper_upper"], dst.hardpoints.damper_upper);
                            if (hp.contains("damper_lower")) load_vec3(hp["damper_lower"], dst.hardpoints.damper_lower);
                            if (hp.contains("arb_link_chassis")) load_vec3(hp["arb_link_chassis"], dst.hardpoints.arb_link_chassis);
                            if (hp.contains("arb_link_wheel")) load_vec3(hp["arb_link_wheel"], dst.hardpoints.arb_link_wheel);
                        }

                        dst.valid = true;
                    }
                }
            }
        }

        // POU ハードポイント座標（参照データ）
        if (config.contains("pou_hardpoints")) {
            auto& ph = config["pou_hardpoints"];
            const std::array<std::string, 4> wheel_keys = {"FL", "FR", "RL", "RR"};
            for (int i = 0; i < 4; ++i) {
                if (ph.contains(wheel_keys[i])) {
                    auto& w = ph[wheel_keys[i]];
                    auto& hp = p.suspension.pou_hardpoints[i];
                    auto load3 = [](const nlohmann::json& arr, std::array<double,3>& dst) {
                        if (arr.is_array() && arr.size() >= 3) {
                            for (int k = 0; k < 3; ++k) dst[k] = arr[k];
                        }
                    };
                    if (w.contains("M_pivot_chassis")) load3(w["M_pivot_chassis"], hp.M_pivot_chassis);
                    if (w.contains("M_pivot_upright")) load3(w["M_pivot_upright"], hp.M_pivot_upright);
                    if (w.contains("bellcrank_pivot")) load3(w["bellcrank_pivot"], hp.bellcrank_pivot);
                    if (w.contains("pushrod_length_m")) hp.pushrod_length = w["pushrod_length_m"];
                    if (w.contains("installation_ratio")) hp.installation_ratio = w["installation_ratio"];
                    hp.valid = true;
                }
            }
        }

        // タイヤ
        if (config.contains("tire")) {
            auto& t = config["tire"];
            if (t.contains("radius")) p.tire.radius = t["radius"];
            if (t.contains("mu_x_max")) p.tire.mu_x_max = t["mu_x_max"];
            if (t.contains("mu_y_max")) p.tire.mu_y_max = t["mu_y_max"];
            if (t.contains("Fz_nom")) p.tire.Fz_nom = t["Fz_nom"];
            if (t.contains("Lx")) p.tire.Lx = t["Lx"];
            if (t.contains("Ly")) p.tire.Ly = t["Ly"];
            if (t.contains("Cx")) p.tire.Cx = t["Cx"];
            if (t.contains("Cy")) p.tire.Cy = t["Cy"];
            if (t.contains("v_N")) p.tire.v_N = t["v_N"];
            if (t.contains("k_tire")) p.tire.k_tire = t["k_tire"];
            if (t.contains("c_tire")) p.tire.c_tire = t["c_tire"];
            if (t.contains("dmu_dFz")) p.tire.dmu_dFz = t["dmu_dFz"];
            if (t.contains("pneumatic_trail")) p.tire.pneumatic_trail = t["pneumatic_trail"];
            if (t.contains("C_rr")) p.tire.C_rr = t["C_rr"];
            if (t.contains("model_type")) p.tire.model_type = t["model_type"];
            if (t.contains("model_dir")) p.tire.model_dir = t["model_dir"];
            if (t.contains("model_manifest")) p.tire.model_manifest = t["model_manifest"];
            if (t.contains("model_name")) p.tire.model_name = t["model_name"];
            if (t.contains("pinn_use_camber_input")) {
                p.tire.pinn_use_camber_input = t["pinn_use_camber_input"];
            }
            if (t.contains("pinn_ttc_lateral_sign")) {
                p.tire.pinn_ttc_lateral_sign = t["pinn_ttc_lateral_sign"];
            }
            if (t.contains("pinn_eval_rate_hz")) {
                p.tire.pinn_eval_rate_hz = t["pinn_eval_rate_hz"];
            }
            if (t.contains("ambient_temperature_c")) {
                p.tire.ambient_temperature_c = t["ambient_temperature_c"];
            }
            if (t.contains("relaxation")) {
                auto& rlx = t["relaxation"];
                if (rlx.contains("Lx_base")) p.tire.relaxation.Lx_base = rlx["Lx_base"];
                if (rlx.contains("Lx_base_m")) p.tire.relaxation.Lx_base = rlx["Lx_base_m"];
                if (rlx.contains("Ly_base")) p.tire.relaxation.Ly_base = rlx["Ly_base"];
                if (rlx.contains("Ly_base_m")) p.tire.relaxation.Ly_base = rlx["Ly_base_m"];
                if (rlx.contains("Fz_exponent")) p.tire.relaxation.Fz_exponent = rlx["Fz_exponent"];
                if (rlx.contains("temp_coeff_per_c")) p.tire.relaxation.temp_coeff_per_c = rlx["temp_coeff_per_c"];
                if (rlx.contains("temp_coeff_per_C")) p.tire.relaxation.temp_coeff_per_c = rlx["temp_coeff_per_C"];
                if (rlx.contains("temp_ref_c")) p.tire.relaxation.temp_ref_c = rlx["temp_ref_c"];
                if (rlx.contains("temp_ref_C")) p.tire.relaxation.temp_ref_c = rlx["temp_ref_C"];
                if (rlx.contains("min_Lx")) p.tire.relaxation.min_Lx = rlx["min_Lx"];
                if (rlx.contains("min_Lx_m")) p.tire.relaxation.min_Lx = rlx["min_Lx_m"];
                if (rlx.contains("max_Lx")) p.tire.relaxation.max_Lx = rlx["max_Lx"];
                if (rlx.contains("max_Lx_m")) p.tire.relaxation.max_Lx = rlx["max_Lx_m"];
                if (rlx.contains("min_Ly")) p.tire.relaxation.min_Ly = rlx["min_Ly"];
                if (rlx.contains("min_Ly_m")) p.tire.relaxation.min_Ly = rlx["min_Ly_m"];
                if (rlx.contains("max_Ly")) p.tire.relaxation.max_Ly = rlx["max_Ly"];
                if (rlx.contains("max_Ly_m")) p.tire.relaxation.max_Ly = rlx["max_Ly_m"];
                if (rlx.contains("enabled")) p.tire.relaxation.enabled = rlx["enabled"];
                p.tire.Lx = p.tire.relaxation.Lx_base;
                p.tire.Ly = p.tire.relaxation.Ly_base;
            }
        }

        // ステアリング
        if (config.contains("steering")) {
            auto& st = config["steering"];
            if (st.contains("ackermann")) p.steering.ackermann = st["ackermann"];
            if (st.contains("max_angle")) p.steering.max_angle = st["max_angle"];
            if (st.contains("stiffness")) p.steering.stiffness = st["stiffness"];

            // Phase A2: ステアリングトルクモデル
            if (st.contains("I_steer")) p.steering.I_steer = st["I_steer"];
            if (st.contains("R_steer")) p.steering.R_steer = st["R_steer"];
            if (st.contains("T_fric")) p.steering.T_fric = st["T_fric"];
            if (st.contains("C_steer_torque")) p.steering.C_steer_torque = st["C_steer_torque"];
            if (st.contains("K_return")) p.steering.K_return = st["K_return"];
            if (st.contains("T_driver_max")) p.steering.T_driver_max = st["T_driver_max"];
            if (st.contains("T_driver_rate_max")) p.steering.T_driver_rate_max = st["T_driver_rate_max"];
            if (st.contains("torque_mode")) p.steering.torque_mode = st["torque_mode"];
        }

        // パワートレイン
        if (config.contains("powertrain")) {
            auto& pt = config["powertrain"];
            if (pt.contains("gear_ratios") && pt["gear_ratios"].is_array()) {
                p.powertrain.gear_ratios.clear();
                for (const auto& val : pt["gear_ratios"]) {
                    p.powertrain.gear_ratios.push_back(val);
                }
            }
            if (pt.contains("primary_reduction")) p.powertrain.primary_ratio = pt["primary_reduction"];
            if (pt.contains("primary_ratio")) p.powertrain.primary_ratio = pt["primary_ratio"];
            if (pt.contains("sprocket_ratio")) p.powertrain.final_ratio = pt["sprocket_ratio"];
            if (pt.contains("final_ratio")) p.powertrain.final_ratio = pt["final_ratio"];
            if (pt.contains("efficiency")) p.powertrain.efficiency = pt["efficiency"];
            if (pt.contains("I_engine")) p.powertrain.I_engine = pt["I_engine"];
            if (pt.contains("I_gearbox")) p.powertrain.I_gearbox = pt["I_gearbox"];
            if (pt.contains("max_torque")) p.powertrain.max_torque = pt["max_torque"];
            if (pt.contains("gear_ratio")) p.powertrain.gear_ratio = pt["gear_ratio"];
            if (pt.contains("shift_time_s")) p.powertrain.shift_time_s = pt["shift_time_s"];
            if (pt.contains("lsd_preload")) p.powertrain.lsd_preload = pt["lsd_preload"];
            if (pt.contains("lsd_ramp_accel")) p.powertrain.lsd_ramp_accel = pt["lsd_ramp_accel"];
            if (pt.contains("lsd_ramp_decel")) p.powertrain.lsd_ramp_decel = pt["lsd_ramp_decel"];
            if (pt.contains("lsd_effective_radius")) p.powertrain.lsd_effective_radius = pt["lsd_effective_radius"];
            if (pt.contains("lsd_clutch_friction")) p.powertrain.lsd_clutch_friction = pt["lsd_clutch_friction"];
            if (pt.contains("lsd_num_plates")) p.powertrain.lsd_num_plates = pt["lsd_num_plates"];
             
            // トルクカーブ
            if (pt.contains("engine_torque_curve") && pt["engine_torque_curve"].is_array()) {
                p.powertrain.engine_torque_curve.clear();
                for (const auto& val : pt["engine_torque_curve"]) {
                    p.powertrain.engine_torque_curve.push_back(val);
                }
            }
            if (pt.contains("engine_rpm_points") && pt["engine_rpm_points"].is_array()) {
                p.powertrain.engine_rpm_points.clear();
                for (const auto& val : pt["engine_rpm_points"]) {
                    p.powertrain.engine_rpm_points.push_back(val);
                }
            }
        }

        // エアロ
        if (config.contains("aero")) {
            auto& a = config["aero"];
            if (a.contains("CL_A")) p.aero.CL_A = a["CL_A"];
            if (a.contains("CD_A")) p.aero.CD_A = a["CD_A"];
            if (a.contains("aero_balance")) p.aero.aero_balance = a["aero_balance"];
            if (a.contains("h_aero")) p.aero.h_aero = a["h_aero"];
            if (a.contains("rho")) p.aero.rho = a["rho"];
            if (a.contains("map_mode")) p.aero.map_mode = a["map_mode"];
            if (a.contains("aero_map_file")) p.aero.aero_map_file = a["aero_map_file"];
            if (a.contains("reference_speed")) p.aero.reference_speed = a["reference_speed"];
            if (a.contains("reference_density")) p.aero.reference_density = a["reference_density"];
            if (a.contains("nominal_front_ride_height_mm")) p.aero.nominal_front_ride_height_mm = a["nominal_front_ride_height_mm"];
            if (a.contains("nominal_rear_ride_height_mm")) p.aero.nominal_rear_ride_height_mm = a["nominal_rear_ride_height_mm"];
            if (a.contains("nominal_roll_rad")) p.aero.nominal_roll_rad = a["nominal_roll_rad"];
            if (a.contains("nominal_yaw_rad")) p.aero.nominal_yaw_rad = a["nominal_yaw_rad"];
            if (a.contains("Fx_ref_total")) p.aero.Fx_ref_total = a["Fx_ref_total"];
            if (a.contains("Fy_ref_total")) p.aero.Fy_ref_total = a["Fy_ref_total"];
            if (a.contains("Fz_ref_total")) p.aero.Fz_ref_total = a["Fz_ref_total"];
            if (a.contains("Mx_ref_total")) p.aero.Mx_ref_total = a["Mx_ref_total"];
            if (a.contains("My_ref_total")) p.aero.My_ref_total = a["My_ref_total"];
            if (a.contains("Mz_ref_total")) p.aero.Mz_ref_total = a["Mz_ref_total"];
            if (a.contains("planner_uses_aero_map")) p.aero.planner_uses_aero_map = a["planner_uses_aero_map"];
        }

        // ブレーキ
        if (config.contains("brake")) {
            auto& b = config["brake"];
            if (b.contains("max_pressure")) p.brake.max_pressure = b["max_pressure"];
            if (b.contains("balance")) p.brake.balance = b["balance"];
            if (b.contains("pedal_table")) p.brake.pedal_table = b["pedal_table"].get<std::vector<double>>();
            if (b.contains("pressure_table")) p.brake.pressure_table = b["pressure_table"].get<std::vector<double>>();
            if (b.contains("pressure_lag_s")) p.brake.pressure_lag_s = b["pressure_lag_s"];
            if (b.contains("omega_smooth")) p.brake.omega_smooth = b["omega_smooth"];
            if (b.contains("mu_pad")) p.brake.mu_pad = b["mu_pad"];
            if (b.contains("caliper_area") && b["caliper_area"].is_array()) {
                for (int i = 0; i < 4; ++i) p.brake.caliper_area[i] = b["caliper_area"][i];
            }
            if (b.contains("effective_radius") && b["effective_radius"].is_array()) {
                for (int i = 0; i < 4; ++i) p.brake.effective_radius[i] = b["effective_radius"][i];
            }
        }

        // ── フラットキー対応 (Python RL / MDO からの簡易 JSON) ──────────
        // Python 側は {"mass": 280, "CL_A": 2.0, "mu_y": 2.0, "k_spring_f": 30000, ...}
        // のようなフラット形式で送信する。ネストセクションが存在しない場合のみ適用。
        //
        // 判定: "mass" キーが無い or 数値 (=フラット形式) の場合に mass 関連を適用
        bool flat_mass = !config.contains("mass") || config["mass"].is_number();
        bool flat_susp = !config.contains("suspension");
        bool flat_tire = !config.contains("tire");
        bool flat_aero = !config.contains("aero");

        if (flat_mass) {
            if (config.contains("mass") && config["mass"].is_number()) {
                p.mass.m_total = config["mass"];
            }
            if (config.contains("m_total") && config["m_total"].is_number()) {
                p.mass.m_total = config["m_total"];
            }
            if (config.contains("wheelbase") && config["wheelbase"].is_number()) {
                p.mass.wheelbase = config["wheelbase"];
            }
            if (config.contains("cg_height") && config["cg_height"].is_number()) {
                p.mass.h_cg = config["cg_height"];
            }
            if (config.contains("h_cg") && config["h_cg"].is_number()) {
                p.mass.h_cg = config["h_cg"];
            }
        }
        if (flat_susp) {
            if (config.contains("k_spring_f") && config["k_spring_f"].is_number()) {
                double k = config["k_spring_f"];
                p.suspension.k_spring[0] = k;  // FL
                p.suspension.k_spring[1] = k;  // FR
            }
            if (config.contains("k_arb_f") && config["k_arb_f"].is_number()) {
                p.suspension.k_arb_f = config["k_arb_f"];
            }
        }
        if (flat_tire) {
            if (config.contains("mu_y") && config["mu_y"].is_number()) {
                p.tire.mu_y_max = config["mu_y"];
            }
            if (config.contains("mu_x") && config["mu_x"].is_number()) {
                p.tire.mu_x_max = config["mu_x"];
            }
            // PINN/NN タイヤモデル指定 (flat JSON — Python/MDO から送信可能)
            if (config.contains("tire_model_type") && config["tire_model_type"].is_string()) {
                p.tire.model_type = config["tire_model_type"];
            }
            if (config.contains("tire_model_dir") && config["tire_model_dir"].is_string()) {
                p.tire.model_dir = config["tire_model_dir"];
            }
            if (config.contains("tire_model_manifest") && config["tire_model_manifest"].is_string()) {
                p.tire.model_manifest = config["tire_model_manifest"];
            }
        }
        if (flat_aero) {
            if (config.contains("CL_A") && config["CL_A"].is_number()) {
                p.aero.CL_A = config["CL_A"];
            }
            if (config.contains("CD_A") && config["CD_A"].is_number()) {
                p.aero.CD_A = config["CD_A"];
            }
        }
    }

    /// JSONファイルから読み込み
    static VehicleParams from_json(const std::string& filepath) {
        VehicleParams p;
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("VehicleParams::from_json: cannot open " + filepath);
        }
        
        nlohmann::json j;
        file >> j;
        parse_json(j, p);
        return p;
    }

    /// JSON文字列から読み込み
    static VehicleParams from_json_string(const std::string& json_str) {
        VehicleParams p;
        auto j = nlohmann::json::parse(json_str);
        parse_json(j, p);
        return p;
    }
    
    /// デフォルト値で初期化
    static VehicleParams default_fsae() {
        return VehicleParams{};
    }
};

} // namespace fsae::vehicle
