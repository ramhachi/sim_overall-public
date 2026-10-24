#include <fsae/vehicle/vehicle_14dof_new.hpp>
#include <iostream>
#include <iomanip>
#include <fstream>

using namespace fsae::vehicle;

int main() {
    std::cout << "14DOF Vehicle with LSD - 旋回加速テスト\n";
    std::cout << "========================================\n\n";
    
    // 車両パラメータ設定
    VehicleParams params;
    params.mass.m_sprung = 200.0;
    params.mass.m_unsprung = {12.0, 12.0, 12.0, 12.0};
    params.mass.a = 0.8;
    params.mass.b = 0.7;
    params.mass.track_f = 1.2;
    params.mass.track_r = 1.15;
    params.mass.h_cg = 0.28;
    params.mass.Ixx = 60.0;
    params.mass.Iyy = 85.0;
    params.mass.Izz = 80.0;
    params.mass.Iw = 0.4;
    
    // LSDパラメータ（トラクション重視）
    params.powertrain.lsd_preload = 40.0;
    params.powertrain.lsd_ramp_accel = 35.0;
    params.powertrain.lsd_ramp_decel = 50.0;
    params.powertrain.lsd_effective_radius = 0.04;
    params.powertrain.lsd_clutch_friction = 0.10;
    params.powertrain.lsd_num_plates = 8;
    
    // 車両初期化
    Vehicle14DOF_New vehicle(params);
    
    std::cout << "車両パラメータ:\n";
    std::cout << "  質量: " << params.mass.m_sprung << " kg\n";
    std::cout << "  ホイールベース: " << (params.mass.a + params.mass.b) << " m\n";
    std::cout << "  LSDプリロード: " << params.powertrain.lsd_preload << " Nm\n";
    std::cout << "  加速側ランプ角: " << params.powertrain.lsd_ramp_accel << "°\n\n";
    
    // 初期状態設定
    vehicle.state().vx = 10.0;  // 10 m/s = 36 km/h (低速コーナー想定)
    vehicle.state().vy = 0.0;
    vehicle.state().r = 0.5;    // 0.5 rad/s のヨーレート（旋回中）
    
    // 車輪を初期速度に合わせる
    double R = params.tire.radius;
    for (int i = 0; i < 4; ++i) {
        vehicle.state().omega[i] = vehicle.state().vx / R;
    }
    
    // テストシナリオ
    std::cout << "テストシナリオ: 低速コーナー立ち上がり加速\n";
    std::cout << "  初速: 10 m/s (36 km/h)\n";
    std::cout << "  ギア: 2速\n";
    std::cout << "  ヨーレート: 0.5 rad/s (旋回中)\n";
    std::cout << "  スロットル: 0% → 100% (0.5秒かけて)\n\n";
    
    // CSVファイル出力
    std::ofstream csv("lsd_test_result.csv");
    csv << "time,throttle,vx,vy,r,omega_RL,omega_RR,omega_diff,engine_rpm,engine_torque\n";
    
    // シミュレーション
    double dt = 0.001;
    double t_end = 1.0;
    int gear = 2;
    
    std::cout << std::setw(8) << "時刻"
              << std::setw(12) << "スロットル"
              << std::setw(10) << "速度"
              << std::setw(12) << "左輪速度"
              << std::setw(12) << "右輪速度"
              << std::setw(12) << "速度差"
              << std::setw(12) << "エンジン\n";
    std::cout << std::string(76, '-') << "\n";
    
    for (double t = 0; t <= t_end; t += dt) {
        // スロットル入力（0.5秒かけて100%に）
        double throttle = std::min(t * 2.0, 1.0);
        
        DriverInput input;
        input.throttle = throttle;
        input.brake = 0.0;
        input.steering = 0.1;  // 軽い舵角
        input.gear = gear;
        
        // 状態更新
        vehicle.step(dt, input);
        const State14DOF& state = vehicle.state();
        
        // 0.1秒ごとに出力
        if (std::fmod(t, 0.1) < dt) {
            double omega_RL = state.omega[2];  // 後輪左
            double omega_RR = state.omega[3];  // 後輪右
            double omega_diff = omega_RL - omega_RR;
            double omega_avg = (omega_RL + omega_RR) / 2.0;
            double speed = omega_avg * R;
            
            // エンジン情報取得
            double engine_rpm = vehicle.get_engine_rpm(gear);
            double engine_torque = vehicle.get_engine_torque(gear, throttle);
            
            std::cout << std::fixed << std::setprecision(2)
                      << std::setw(6) << t << " s"
                      << std::setw(10) << throttle*100 << "%"
                      << std::setw(8) << speed << " m/s"
                      << std::setw(10) << omega_RL << " r/s"
                      << std::setw(10) << omega_RR << " r/s"
                      << std::setw(10) << omega_diff << " r/s"
                      << std::setw(10) << std::setprecision(0) << engine_rpm << " rpm\n";
            
            // CSV出力
            csv << t << ","
                << throttle << ","
                << state.vx << ","
                << state.vy << ","
                << state.r << ","
                << omega_RL << ","
                << omega_RR << ","
                << omega_diff << ","
                << engine_rpm << ","
                << engine_torque << "\n";
        }
    }
    
    csv.close();
    
    const State14DOF& final_state = vehicle.state();
    std::cout << "\n結果:\n";
    std::cout << "  最終速度: " << std::fixed << std::setprecision(1) 
              << final_state.vx << " m/s (" << final_state.vx * 3.6 << " km/h)\n";
    std::cout << "  左右輪速度差: " << std::setprecision(2) 
              << (final_state.omega[2] - final_state.omega[3]) << " rad/s\n";
    std::cout << "  ヨーレート: " << final_state.r << " rad/s\n";
    std::cout << "\nデータを lsd_test_result.csv に保存しました。\n";
    std::cout << "\nLSDが左右輪間のトルク配分を最適化し、\n";
    std::cout << "旋回加速時のトラクション向上に貢献しています。\n";
    
    return 0;
}
