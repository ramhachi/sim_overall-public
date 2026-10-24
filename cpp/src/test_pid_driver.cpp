/// @file test_pid_driver.cpp
/// @brief PID経路追従ドライバーのテストプログラム

#include <fsae/driver/pid_driver.hpp>
#include <fsae/vehicle/vehicle_14dof_new.hpp>
#include <fsae/track/track_model.hpp>
#include <fsae/laptime/velocity_profile.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace fsae;

void test_pid_circle_tracking() {
    std::cout << "=== PID Circle Tracking Test ===" << std::endl;

    // トラック生成
    auto track = track::create_circle_track(30.0, 3.0);

    // 速度プロファイル生成
    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;
    perf.max_speed = 25.0;

    laptime::VelocityProfileGenerator vel_gen(perf);
    auto velocity_profile = vel_gen.generate(track, 1.0);

    // PIDドライバー
    driver::PIDPathFollowingDriver pid_driver(track, velocity_profile);

    // 車両モデル
    vehicle::VehicleParams vehicle_params;
    vehicle::Vehicle14DOF_New vehicle(vehicle_params);
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);

    // 初期位置をトラック開始位置に設定
    // 円形トラック: 円の東端(R, 0)から北向き(π/2)でスタート
    vehicle.state().x = 30.0;  // 半径と同じ
    vehicle.state().y = 0.0;
    vehicle.state().psi = M_PI / 2.0;  // 北向き

    // シミュレーション
    double dt = 0.001;  // 1 ms
    double t_max = 5.0;  // 5秒間

    std::ofstream ofs("pid_circle_tracking.csv");
    ofs << "time,x,y,psi,vx,vy,r,steering,throttle,brake,s,n,d_heading,target_v,actual_v\n";

    for (double t = 0.0; t < t_max; t += dt) {
        // 制御入力計算
        auto input = pid_driver.compute_control(vehicle.state(), t);

        // 車両ステップ
        vehicle.step(dt, input);

        // Frenet座標取得
        auto frenet = pid_driver.get_current_frenet(vehicle.state());

        // 10ms毎に出力
        if (static_cast<int>(t * 1000) % 10 == 0) {
            auto s = vehicle.state();
            double target_v = 0.0;
            for (const auto& vp : velocity_profile) {
                if (std::abs(vp.s - frenet.s) < 1.0) {
                    target_v = vp.v;
                    break;
                }
            }

            ofs << std::fixed << std::setprecision(6)
                << t << "," << s.x << "," << s.y << "," << s.psi << ","
                << s.vx << "," << s.vy << "," << s.r << ","
                << input.steering << "," << input.throttle << "," << input.brake << ","
                << frenet.s << "," << frenet.n << "," << frenet.d_heading << ","
                << target_v << "," << s.speed() << "\n";
        }
    }

    // 統計
    auto final_frenet = pid_driver.get_current_frenet(vehicle.state());
    std::cout << "Final lateral error: " << final_frenet.n << " m" << std::endl;
    std::cout << "Final heading error: " << (final_frenet.d_heading * 180.0 / M_PI) << " deg" << std::endl;
    std::cout << "Distance traveled: " << final_frenet.s << " m" << std::endl;
    std::cout << "Results saved to pid_circle_tracking.csv" << std::endl;
}

void test_pid_figure_eight_tracking() {
    std::cout << "\n=== PID Figure Eight Tracking Test ===" << std::endl;

    auto track = track::create_figure_eight_track(25.0, 3.0);

    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;
    perf.max_speed = 25.0;

    laptime::VelocityProfileGenerator vel_gen(perf);
    auto velocity_profile = vel_gen.generate(track, 1.0);

    driver::PIDPathFollowingDriver pid_driver(track, velocity_profile);

    vehicle::VehicleParams vehicle_params;
    vehicle::Vehicle14DOF_New vehicle(vehicle_params);
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);

    // 初期位置（トラック開始位置：原点、東向き）
    vehicle.state().x = 0.0;
    vehicle.state().y = 0.0;
    vehicle.state().psi = 0.0;  // 東向き

    double dt = 0.001;
    double t_max = 10.0;

    std::ofstream ofs("pid_figure_eight_tracking.csv");
    ofs << "time,x,y,psi,s,n,d_heading,steering,speed,target_speed\n";

    for (double t = 0.0; t < t_max; t += dt) {
        auto input = pid_driver.compute_control(vehicle.state(), t);
        vehicle.step(dt, input);

        if (static_cast<int>(t * 1000) % 10 == 0) {
            auto s = vehicle.state();
            auto frenet = pid_driver.get_current_frenet(s);

            double target_v = 0.0;
            for (const auto& vp : velocity_profile) {
                if (std::abs(vp.s - frenet.s) < 1.0) {
                    target_v = vp.v;
                    break;
                }
            }

            ofs << std::fixed << std::setprecision(6)
                << t << "," << s.x << "," << s.y << "," << s.psi << ","
                << frenet.s << "," << frenet.n << "," << frenet.d_heading << ","
                << input.steering << "," << s.speed() << "," << target_v << "\n";
        }
    }

    auto final_frenet = pid_driver.get_current_frenet(vehicle.state());
    std::cout << "Final lateral error: " << final_frenet.n << " m" << std::endl;
    std::cout << "Distance traveled: " << final_frenet.s << " m" << std::endl;
    std::cout << "Results saved to pid_figure_eight_tracking.csv" << std::endl;
}

void test_pid_gain_tuning() {
    std::cout << "\n=== PID Gain Tuning Comparison ===" << std::endl;

    auto track = track::create_circle_track(30.0, 3.0);

    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;
    perf.max_speed = 25.0;

    laptime::VelocityProfileGenerator vel_gen(perf);
    auto velocity_profile = vel_gen.generate(track, 1.0);

    // 3つのゲインセットで比較
    std::vector<std::pair<std::string, driver::PIDGains>> gain_sets = {
        {"Conservative", driver::conservative_gains()},
        {"Default", driver::PIDGains()},
        {"Aggressive", driver::aggressive_gains()}
    };

    for (const auto& [name, gains] : gain_sets) {
        driver::PIDPathFollowingDriver pid_driver(track, velocity_profile, gains);

        vehicle::VehicleParams vehicle_params;
        vehicle::Vehicle14DOF_New vehicle(vehicle_params);
        vehicle.reset();
        vehicle.set_initial_velocity(15.0);
        vehicle.state().x = 0.0;
        vehicle.state().y = 0.0;
        vehicle.state().psi = 0.0;  // 東向き

        double dt = 0.001;
        double t_max = 3.0;

        double max_lateral_error = 0.0;
        double avg_lateral_error = 0.0;
        int count = 0;

        for (double t = 0.0; t < t_max; t += dt) {
            auto input = pid_driver.compute_control(vehicle.state(), t);
            vehicle.step(dt, input);

            if (t > 1.0) {  // 最初の1秒は除外（過渡応答）
                auto frenet = pid_driver.get_current_frenet(vehicle.state());
                double lat_error = std::abs(frenet.n);
                max_lateral_error = std::max(max_lateral_error, lat_error);
                avg_lateral_error += lat_error;
                count++;
            }
        }

        avg_lateral_error /= count;

        std::cout << name << " gains:" << std::endl;
        std::cout << "  Max lateral error: " << max_lateral_error << " m" << std::endl;
        std::cout << "  Avg lateral error: " << avg_lateral_error << " m" << std::endl;
    }
}

void test_pid_speed_control() {
    std::cout << "\n=== PID Speed Control Test ===" << std::endl;

    // 直線+コーナーで速度制御をテスト
    auto track = track::create_straight_corner_track(50.0, 15.0, 3.0);

    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;
    perf.max_speed = 30.0;

    laptime::VelocityProfileGenerator vel_gen(perf);
    auto velocity_profile = vel_gen.generate(track, 0.5);

    driver::PIDPathFollowingDriver pid_driver(track, velocity_profile);

    vehicle::VehicleParams vehicle_params;
    vehicle::Vehicle14DOF_New vehicle(vehicle_params);
    vehicle.reset();
    vehicle.set_initial_velocity(10.0);

    double dt = 0.001;
    double t_max = 10.0;

    std::ofstream ofs("pid_speed_control.csv");
    ofs << "time,s,speed,target_speed,throttle,brake,speed_error\n";

    for (double t = 0.0; t < t_max; t += dt) {
        auto input = pid_driver.compute_control(vehicle.state(), t);
        vehicle.step(dt, input);

        if (static_cast<int>(t * 1000) % 10 == 0) {
            auto s = vehicle.state();
            auto frenet = pid_driver.get_current_frenet(s);

            double target_v = 0.0;
            for (const auto& vp : velocity_profile) {
                if (std::abs(vp.s - frenet.s) < 1.0) {
                    target_v = vp.v;
                    break;
                }
            }

            double speed_error = target_v - s.speed();

            ofs << std::fixed << std::setprecision(6)
                << t << "," << frenet.s << "," << s.speed() << "," << target_v << ","
                << input.throttle << "," << input.brake << "," << speed_error << "\n";
        }
    }

    std::cout << "Speed control test completed" << std::endl;
    std::cout << "Results saved to pid_speed_control.csv" << std::endl;
}

int main() {
    std::cout << "PID Path-Following Driver Tests\n" << std::endl;

    test_pid_circle_tracking();
    test_pid_figure_eight_tracking();
    test_pid_gain_tuning();
    test_pid_speed_control();

    std::cout << "\nAll PID driver tests completed!" << std::endl;
    return 0;
}
