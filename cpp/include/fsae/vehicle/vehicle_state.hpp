#pragma once
/// @file vehicle_state.hpp
/// @brief 14DOF車両状態ベクトル

#include <array>
#include <Eigen/Dense>

namespace fsae::vehicle {

/// 14DOF車両状態
struct VehicleState {
    // === ばね上 (Sprung Mass) ===
    // 位置
    double x = 0;       // 縦位置 [m]
    double y = 0;       // 横位置 [m]
    double z = 0;       // 垂直位置 [m]
    double phi = 0;     // ロール角 [rad]
    double theta = 0;   // ピッチ角 [rad]
    double psi = 0;     // ヨー角 [rad]
    
    // 速度（車両座標系）
    double vx = 0;      // 縦速度 [m/s]
    double vy = 0;      // 横速度 [m/s]
    double vz = 0;      // 垂直速度 [m/s]
    double p = 0;       // ロールレート [rad/s]
    double q = 0;       // ピッチレート [rad/s]
    double r = 0;       // ヨーレート [rad/s]
    
    // === ばね下 (Unsprung Mass) ===
    std::array<double, 4> z_u = {0, 0, 0, 0};     // 垂直位置 [m]
    std::array<double, 4> vz_u = {0, 0, 0, 0};    // 垂直速度 [m/s]
    
    // === 車輪回転 ===
    std::array<double, 4> omega = {0, 0, 0, 0};   // 回転速度 [rad/s]
    
    // === タイヤ状態（TMeasy） ===
    std::array<double, 4> tire_xi = {0, 0, 0, 0};   // 縦変形 [m]
    std::array<double, 4> tire_eta = {0, 0, 0, 0};  // 横変形 [m]
    
    // === ダンパー状態（Bouc-Wen） ===
    std::array<double, 4> damper_z = {0, 0, 0, 0};  // 履歴変数
    
    // === Eigenベクトル変換 ===
    static constexpr int STATE_SIZE = 40;
    
    Eigen::VectorXd to_vector() const {
        Eigen::VectorXd v(STATE_SIZE);
        int idx = 0;
        
        // ばね上位置
        v(idx++) = x; v(idx++) = y; v(idx++) = z;
        v(idx++) = phi; v(idx++) = theta; v(idx++) = psi;
        
        // ばね上速度
        v(idx++) = vx; v(idx++) = vy; v(idx++) = vz;
        v(idx++) = p; v(idx++) = q; v(idx++) = r;
        
        // ばね下
        for (int i = 0; i < 4; ++i) v(idx++) = z_u[i];
        for (int i = 0; i < 4; ++i) v(idx++) = vz_u[i];
        
        // 車輪回転
        for (int i = 0; i < 4; ++i) v(idx++) = omega[i];
        
        // タイヤ
        for (int i = 0; i < 4; ++i) v(idx++) = tire_xi[i];
        for (int i = 0; i < 4; ++i) v(idx++) = tire_eta[i];
        
        // ダンパー
        for (int i = 0; i < 4; ++i) v(idx++) = damper_z[i];
        
        return v;
    }
    
    static VehicleState from_vector(const Eigen::VectorXd& v) {
        VehicleState s;
        int idx = 0;
        
        // ばね上位置
        s.x = v(idx++); s.y = v(idx++); s.z = v(idx++);
        s.phi = v(idx++); s.theta = v(idx++); s.psi = v(idx++);
        
        // ばね上速度
        s.vx = v(idx++); s.vy = v(idx++); s.vz = v(idx++);
        s.p = v(idx++); s.q = v(idx++); s.r = v(idx++);
        
        // ばね下
        for (int i = 0; i < 4; ++i) s.z_u[i] = v(idx++);
        for (int i = 0; i < 4; ++i) s.vz_u[i] = v(idx++);
        
        // 車輪回転
        for (int i = 0; i < 4; ++i) s.omega[i] = v(idx++);
        
        // タイヤ
        for (int i = 0; i < 4; ++i) s.tire_xi[i] = v(idx++);
        for (int i = 0; i < 4; ++i) s.tire_eta[i] = v(idx++);
        
        // ダンパー
        for (int i = 0; i < 4; ++i) s.damper_z[i] = v(idx++);
        
        return s;
    }
    
    /// 車速を計算
    double speed() const {
        return std::sqrt(vx*vx + vy*vy);
    }
    
    /// スリップ角（車体）を計算
    double slip_angle_body() const {
        return (std::abs(vx) > 0.1) ? std::atan2(vy, vx) : 0.0;
    }
};

/// ドライバー入力
struct DriverInput {
    double steering = 0;    // ステアリング角 [rad]（ハンドル角×レシオ）
    double throttle = 0;    // スロットル [0-1]
    double brake = 0;       // ブレーキ [0-1]
    int gear = 3;           // ギア段
    bool drs_button = false; // DRSボタン [true=起動要求, false=通常]
};

/// 各輪の状態（中間計算用）
struct WheelState {
    // 位置（車両座標系）
    double x_pos, y_pos;    // 接地点位置
    
    // 速度
    double vx_wheel;        // 縦速度（ホイール座標系）
    double vy_wheel;        // 横速度（ホイール座標系）
    
    // スリップ
    double slip_angle;      // スリップ角 [rad]
    double slip_ratio;      // スリップ率 [-]
    
    // 力
    double Fx, Fy, Fz;      // タイヤ力 [N]
    double Mz;              // セルフアライニングトルク [Nm]
    
    // サスペンション
    double susp_travel;     // サス変位 [m]
    double susp_velocity;   // サス速度 [m/s]
    double F_spring;        // ばね力 [N]
    double F_damper;        // ダンパー力 [N]
    
    // 舵角
    double steer_angle;     // 舵角 [rad]
};

/// シミュレーション出力
struct SimOutput {
    double time;
    VehicleState state;
    std::array<WheelState, 4> wheels;
    
    // 派生量
    double ax, ay, az;      // 加速度 [m/s²]
    double Fz_aero;         // 空力ダウンフォース [N]
    double Fx_aero;         // 空力抗力 [N]
};

} // namespace fsae::vehicle
