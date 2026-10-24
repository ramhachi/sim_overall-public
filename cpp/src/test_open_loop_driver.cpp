/// @file test_open_loop_driver.cpp
/// @brief オープンループドライバーのテストプログラム

#include <fsae/driver/open_loop_driver.hpp>
#include <fsae/vehicle/vehicle_14dof_new.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace fsae;

void test_step_steer() {
    std::cout << "=== Step Steer Test ===" << std::endl;

    // ドライバーモデル
    driver::StepSteerDriver driver(
        1.0,   // step_time
        0.1,   // steer_angle [rad] (≈5.7deg)
        10.0   // duration
    );

    // 車両モデル
    vehicle::VehicleParams params;
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);  // 15 m/s ≈ 54 km/h

    // シミュレーション
    double dt = 0.001;  // 1 ms
    std::ofstream ofs("step_steer_result.csv");
    ofs << "time,x,y,psi,vx,vy,r,steering,lateral_accel\n";

    for (double t = 0.0; t < driver.duration(); t += dt) {
        auto input = driver.get_input(t);
        vehicle.step(dt, input);

        auto s = vehicle.state();

        // 横加速度
        double ay = s.vx * s.r + 9.81 * std::sin(s.phi);

        if (static_cast<int>(t * 1000) % 10 == 0) {  // 10ms毎に出力
            ofs << std::fixed << std::setprecision(6)
                << t << "," << s.x << "," << s.y << "," << s.psi << ","
                << s.vx << "," << s.vy << "," << s.r << ","
                << input.steering << "," << ay << "\n";
        }
    }

    std::cout << "Results saved to step_steer_result.csv" << std::endl;
}

void test_sine_steer() {
    std::cout << "\n=== Sine Steer (Weave) Test ===" << std::endl;

    driver::SineSteerDriver driver(
        0.5,   // frequency [Hz]
        0.08,  // amplitude [rad] (≈4.6deg)
        10.0   // duration
    );

    vehicle::VehicleParams params;
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(20.0);  // 20 m/s ≈ 72 km/h

    double dt = 0.001;
    std::ofstream ofs("sine_steer_result.csv");
    ofs << "time,x,y,psi,vx,vy,r,steering,lateral_accel\n";

    for (double t = 0.0; t < driver.duration(); t += dt) {
        auto input = driver.get_input(t);
        vehicle.step(dt, input);

        auto s = vehicle.state();
        double ay = s.vx * s.r + 9.81 * std::sin(s.phi);

        if (static_cast<int>(t * 1000) % 10 == 0) {
            ofs << std::fixed << std::setprecision(6)
                << t << "," << s.x << "," << s.y << "," << s.psi << ","
                << s.vx << "," << s.vy << "," << s.r << ","
                << input.steering << "," << ay << "\n";
        }
    }

    std::cout << "Results saved to sine_steer_result.csv" << std::endl;
}

void test_double_lane_change() {
    std::cout << "\n=== Double Lane Change Test ===" << std::endl;

    driver::DoubleLaneChangeDriver driver(
        2.0,   // lane_change_time
        0.12,  // steer_amplitude [rad] (≈6.9deg)
        12.0   // duration
    );

    vehicle::VehicleParams params;
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);  // 15 m/s

    double dt = 0.001;
    std::ofstream ofs("double_lane_change_result.csv");
    ofs << "time,x,y,psi,vx,vy,r,steering,lateral_accel\n";

    for (double t = 0.0; t < driver.duration(); t += dt) {
        auto input = driver.get_input(t);
        vehicle.step(dt, input);

        auto s = vehicle.state();
        double ay = s.vx * s.r + 9.81 * std::sin(s.phi);

        if (static_cast<int>(t * 1000) % 10 == 0) {
            ofs << std::fixed << std::setprecision(6)
                << t << "," << s.x << "," << s.y << "," << s.psi << ","
                << s.vx << "," << s.vy << "," << s.r << ","
                << input.steering << "," << ay << "\n";
        }
    }

    std::cout << "Results saved to double_lane_change_result.csv" << std::endl;
}

void test_accel_brake() {
    std::cout << "\n=== Acceleration/Braking Test ===" << std::endl;

    driver::AccelBrakeDriver driver(
        4.0,   // accel_duration
        5.0,   // brake_start
        10.0   // total_duration
    );

    vehicle::VehicleParams params;
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(5.0);  // 5 m/s 初速

    double dt = 0.001;
    std::ofstream ofs("accel_brake_result.csv");
    ofs << "time,x,vx,ax,throttle,brake\n";

    double prev_vx = vehicle.state().vx;

    for (double t = 0.0; t < driver.duration(); t += dt) {
        auto input = driver.get_input(t);
        vehicle.step(dt, input);

        auto s = vehicle.state();
        double ax = (s.vx - prev_vx) / dt;
        prev_vx = s.vx;

        if (static_cast<int>(t * 1000) % 10 == 0) {
            ofs << std::fixed << std::setprecision(6)
                << t << "," << s.x << "," << s.vx << "," << ax << ","
                << input.throttle << "," << input.brake << "\n";
        }
    }

    std::cout << "Results saved to accel_brake_result.csv" << std::endl;
}

void test_custom_timeseries() {
    std::cout << "\n=== Custom Time Series Test ===" << std::endl;

    // カスタムマニューバを定義
    std::vector<driver::TimeSeriesPoint> timeseries = {
        {0.0, 0.0, 0.3, 0.0, 3},       // t=0s: 直進
        {2.0, 0.0, 0.3, 0.0, 3},       // t=2s: 直進継続
        {3.0, 0.1, 0.3, 0.0, 3},       // t=3s: 左旋回開始
        {5.0, 0.1, 0.3, 0.0, 3},       // t=5s: 旋回保持
        {6.0, -0.1, 0.3, 0.0, 3},      // t=6s: 反対方向へ
        {8.0, -0.1, 0.3, 0.0, 3},      // t=8s: 旋回保持
        {9.0, 0.0, 0.3, 0.0, 3},       // t=9s: 直進復帰
        {10.0, 0.0, 0.0, 0.5, 3},      // t=10s: 減速
    };

    driver::TimeSeriesDriver driver(timeseries);

    vehicle::VehicleParams params;
    vehicle::Vehicle14DOF_New vehicle(params);
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);

    double dt = 0.001;
    std::ofstream ofs("custom_timeseries_result.csv");
    ofs << "time,x,y,psi,vx,vy,r,steering\n";

    for (double t = 0.0; t < driver.duration(); t += dt) {
        auto input = driver.get_input(t);
        vehicle.step(dt, input);

        auto s = vehicle.state();

        if (static_cast<int>(t * 1000) % 10 == 0) {
            ofs << std::fixed << std::setprecision(6)
                << t << "," << s.x << "," << s.y << "," << s.psi << ","
                << s.vx << "," << s.vy << "," << s.r << ","
                << input.steering << "\n";
        }
    }

    std::cout << "Results saved to custom_timeseries_result.csv" << std::endl;
}

int main() {
    std::cout << "Open Loop Driver Tests\n" << std::endl;

    test_step_steer();
    test_sine_steer();
    test_double_lane_change();
    test_accel_brake();
    test_custom_timeseries();

    std::cout << "\nAll tests completed successfully!" << std::endl;
    return 0;
}
