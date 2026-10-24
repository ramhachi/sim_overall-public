#include <fsae/physics/powertrain/lsd.hpp>
#include <fsae/physics/powertrain/engine.hpp>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace fsae::physics::powertrain;

void print_section(const std::string& title) {
    std::cout << "\n" << std::string(60, '=') << "\n";
    std::cout << title << "\n";
    std::cout << std::string(60, '=') << "\n";
}

void test_lsd_basic() {
    print_section("LSD基本機能テスト");
    
    // トラクション重視LSD
    auto lsd = SalisburyLSD(SalisburyLSD::Params::fsae_traction_focused());
    auto params = lsd.get_params();
    
    std::cout << "パラメータ:\n";
    std::cout << "  加速側ランプ角: " << params.ramp_angle_accel << "°\n";
    std::cout << "  減速側ランプ角: " << params.ramp_angle_decel << "°\n";
    std::cout << "  プリロード: " << params.preload_torque << " Nm\n";
    std::cout << "  有効半径: " << params.effective_radius * 1000 << " mm\n";
    std::cout << "  摩擦係数: " << params.clutch_friction << "\n";
    std::cout << "  プレート数: " << params.num_plates << "\n";
}

void test_torque_distribution() {
    print_section("トルク配分テスト");
    
    auto lsd = SalisburyLSD(SalisburyLSD::Params::fsae_traction_focused());
    
    // テストケース: 3速、10,000rpm、80%スロットル
    auto engine = create_cbr600rr_engine();
    auto gears = GearRatios::cbr600rr_default();
    
    double throttle = 0.8;
    double rpm = 10000;
    double engine_torque = engine.get_torque(rpm, throttle);
    double total_ratio = gears.get_total_ratio(3);  // 3速
    double total_torque = engine_torque * total_ratio * 0.90;  // 効率90%
    
    std::cout << "\n入力条件:\n";
    std::cout << "  エンジン: " << rpm << " rpm, " << throttle*100 << "% スロットル\n";
    std::cout << "  エンジントルク: " << std::fixed << std::setprecision(1) 
              << engine_torque << " Nm\n";
    std::cout << "  総減速比: " << std::setprecision(2) << total_ratio << "\n";
    std::cout << "  総駆動トルク: " << std::setprecision(1) << total_torque << " Nm\n";
    
    // シナリオ1: 直進（左右同速度）
    std::cout << "\n[シナリオ1] 直進加速（左右輪速度差なし）\n";
    double omega_straight = 50.0;  // 50 rad/s
    double torque_L1, torque_R1;
    lsd.distribute_torque(total_torque, omega_straight, omega_straight, 
                         torque_L1, torque_R1);
    std::cout << "  左輪回転数: " << omega_straight << " rad/s\n";
    std::cout << "  右輪回転数: " << omega_straight << " rad/s\n";
    std::cout << "  左輪トルク: " << torque_L1 << " Nm\n";
    std::cout << "  右輪トルク: " << torque_R1 << " Nm\n";
    std::cout << "  トルク配分比: 50:50\n";
    
    // シナリオ2: 旋回（内輪10%遅い）
    std::cout << "\n[シナリオ2] 旋回加速（内輪が遅い）\n";
    double omega_outer = 50.0;
    double omega_inner = 45.0;  // 内輪10%遅い
    double torque_L2, torque_R2;
    lsd.distribute_torque(total_torque, omega_inner, omega_outer, 
                         torque_L2, torque_R2);
    std::cout << "  内輪回転数: " << omega_inner << " rad/s\n";
    std::cout << "  外輪回転数: " << omega_outer << " rad/s\n";
    std::cout << "  内輪トルク: " << torque_L2 << " Nm\n";
    std::cout << "  外輪トルク: " << torque_R2 << " Nm\n";
    double ratio = torque_R2 / torque_L2;
    std::cout << "  トルク配分比（外/内）: " << std::setprecision(2) << ratio << "\n";
    std::cout << "  → 外輪にもトルクが配分されている（LSD効果）\n";
    
    // シナリオ3: 片輪スリップ（片輪が大幅に速い）
    std::cout << "\n[シナリオ3] 片輪スリップ（片輪が2倍速）\n";
    double omega_grip = 50.0;
    double omega_slip = 100.0;  // スリップして2倍回転
    double torque_L3, torque_R3;
    lsd.distribute_torque(total_torque, omega_grip, omega_slip, 
                         torque_L3, torque_R3);
    std::cout << "  グリップ輪: " << omega_grip << " rad/s\n";
    std::cout << "  スリップ輪: " << omega_slip << " rad/s\n";
    std::cout << "  グリップ輪トルク: " << torque_L3 << " Nm\n";
    std::cout << "  スリップ輪トルク: " << torque_R3 << " Nm\n";
    double ratio2 = torque_L3 / torque_R3;
    std::cout << "  トルク配分比（グリップ/スリップ）: " << std::setprecision(2) << ratio2 << "\n";
    std::cout << "  → グリップ輪に多くのトルクが配分（トラクション確保）\n";
}

void test_locking_torque() {
    print_section("差動制限トルクテスト");
    
    auto lsd = SalisburyLSD(SalisburyLSD::Params::fsae_traction_focused());
    
    std::cout << "\n入力トルクと制限トルクの関係:\n";
    std::cout << std::setw(15) << "入力トルク" 
              << std::setw(20) << "制限トルク(加速)"
              << std::setw(20) << "制限トルク(減速)\n";
    std::cout << std::string(55, '-') << "\n";
    
    double omega_diff = 5.0;  // 5 rad/s の差動
    
    for (double input_torque : {100.0, 300.0, 500.0, 700.0, 1000.0}) {
        // 加速時
        double lock_accel = lsd.compute_locking_torque(
            input_torque, 50.0 + omega_diff, 50.0);
        
        // 減速時（負のトルク）
        double lock_decel = lsd.compute_locking_torque(
            -input_torque, 50.0 + omega_diff, 50.0);
        
        std::cout << std::setw(12) << std::fixed << std::setprecision(0) 
                  << input_torque << " Nm"
                  << std::setw(17) << lock_accel << " Nm"
                  << std::setw(17) << lock_decel << " Nm\n";
    }
    
    std::cout << "\n→ 加速時の方が強く差動を制限（ランプ角が小さいため）\n";
}

void test_bias_ratio() {
    print_section("トルクバイアス比テスト");
    
    std::cout << "\n異なるLSDセッティングの比較:\n\n";
    
    struct TestSetup {
        std::string name;
        SalisburyLSD::Params params;
    };
    
    std::vector<TestSetup> setups = {
        {"トラクション重視", SalisburyLSD::Params::fsae_traction_focused()},
        {"バランス型", SalisburyLSD::Params::fsae_balanced()},
        {"ハンドリング重視", SalisburyLSD::Params::fsae_handling_focused()}
    };
    
    double input_torque = 500.0;  // 500 Nm
    double omega_inner = 45.0;
    double omega_outer = 50.0;
    
    for (const auto& setup : setups) {
        SalisburyLSD lsd(setup.params);
        double bias = lsd.compute_bias_ratio(input_torque, omega_inner, omega_outer);
        
        std::cout << setup.name << ":\n";
        std::cout << "  加速ランプ角: " << setup.params.ramp_angle_accel << "°\n";
        std::cout << "  減速ランプ角: " << setup.params.ramp_angle_decel << "°\n";
        std::cout << "  プリロード: " << setup.params.preload_torque << " Nm\n";
        std::cout << "  トルクバイアス比: " << std::fixed << std::setprecision(2) 
                  << bias << " : 1\n";
        std::cout << "\n";
    }
    
    std::cout << "※ トルクバイアス比 = (高速側トルク / 低速側トルク)\n";
    std::cout << "  オープンデフ: 1.0 (全トルクが遅い方に集中)\n";
    std::cout << "  FSAE LSD: 1.5-3.0 (速い方にもトルク配分)\n";
}

void test_real_world_scenario() {
    print_section("実走行シナリオシミュレーション");
    
    auto lsd = SalisburyLSD(SalisburyLSD::Params::fsae_traction_focused());
    auto engine = create_cbr600rr_engine();
    auto gears = GearRatios::cbr600rr_default();
    
    std::cout << "\nシナリオ: 低速コーナー立ち上がり加速\n";
    std::cout << "  ギア: 2速\n";
    std::cout << "  エンジン: 9000 rpm (パワーバンド入口)\n";
    std::cout << "  スロットル: 60% → 100% (段階的に開ける)\n";
    std::cout << "  旋回中: 内輪5%遅い状態\n\n";
    
    int gear = 2;
    double rpm = 9000;
    double total_ratio = gears.get_total_ratio(gear);
    
    std::cout << std::setw(12) << "スロットル"
              << std::setw(18) << "エンジントルク"
              << std::setw(15) << "総駆動力"
              << std::setw(12) << "内輪T"
              << std::setw(12) << "外輪T\n";
    std::cout << std::string(69, '-') << "\n";
    
    double omega_base = 40.0;  // 基準車輪速度 [rad/s]
    
    for (double throttle : {0.6, 0.7, 0.8, 0.9, 1.0}) {
        double omega_inner = omega_base * 0.95;  // 内輪5%遅い
        double omega_outer = omega_base * 1.05;  // 外輪5%速い
        
        double engine_torque = engine.get_torque(rpm, throttle);
        double total_torque = engine_torque * total_ratio * 0.90;
        
        double torque_inner, torque_outer;
        lsd.distribute_torque(total_torque, omega_inner, omega_outer,
                            torque_inner, torque_outer);
        
        std::cout << std::setw(10) << std::fixed << std::setprecision(0) 
                  << throttle*100 << "%"
                  << std::setw(15) << std::setprecision(1) << engine_torque << " Nm"
                  << std::setw(13) << total_torque << " Nm"
                  << std::setw(10) << torque_inner << " Nm"
                  << std::setw(10) << torque_outer << " Nm\n";
    }
    
    std::cout << "\n→ スロットル開度に応じて、LSDが適切にトルクを配分\n";
    std::cout << "→ 外輪（高速側）にもトルクが配分され、トラクション向上\n";
}

// #35: 連続 tanh バイアス・コースト時プリロード・減速ランプ選択の判定（失敗数を返す）
int check_lsd_physics() {
    int failures = 0;
    auto check = [&](bool ok, const char* what) {
        std::cout << (ok ? "  PASS " : "  FAIL ") << what << "\n";
        if (!ok) ++failures;
    };
    print_section("判定 (#35)");
    const auto p = SalisburyLSD::Params::fsae_traction_focused();
    SalisburyLSD lsd(p);
    double tl = 0.0, tr = 0.0;

    // コースト（入力 0）でもプリロードが差速に抗する
    lsd.distribute_torque(0.0, 51.0, 50.0, tl, tr);
    check(std::abs((tr - tl) - p.preload_torque * std::tanh(1.0 / p.omega_smooth)) < 1e-9,
          "coast: preload acts with zero input torque");
    check(std::abs(tl + tr) < 1e-12, "coast: torque sum conserved");

    // 連続性: Δω 0 を跨いで滑らか（旧 0.01 rad/s デッドバンドの不連続が無い）
    double tl_a, tr_a, tl_b, tr_b;
    lsd.distribute_torque(300.0, 50.0 + 0.009, 50.0, tl_a, tr_a);
    lsd.distribute_torque(300.0, 50.0 + 0.011, 50.0, tl_b, tr_b);
    check(std::abs((tr_b - tl_b) - (tr_a - tl_a)) < 0.05 * p.preload_torque, "bias continuous across old deadband");
    lsd.distribute_torque(300.0, 50.0, 50.0, tl, tr);
    check(std::abs(tl - tr) < 1e-12, "zero slip -> equal split");

    // 減速ランプは負トルクで選択される（エンジンブレーキで有効）
    const double lock_acc = lsd.compute_locking_torque(500.0, 51.0, 50.0);
    const double lock_dec = lsd.compute_locking_torque(-500.0, 51.0, 50.0);
    check(lock_dec < lock_acc, "decel ramp (larger angle) selected for negative input torque");
    lsd.distribute_torque(-500.0, 60.0, 50.0, tl, tr);
    check(std::abs((tr - tl) - lock_dec * std::tanh(10.0 / p.omega_smooth)) < 1e-9,
          "engine braking: bias uses decel-ramp locking torque, no |T_in| cap");
    return failures;
}

int main() {
    std::cout << "FSAE Salisbury LSD テストプログラム\n";
    std::cout << "======================================\n";

    if (check_lsd_physics() > 0) {
        std::cerr << "LSD checks FAILED\n";
        return 1;
    }

    try {
        test_lsd_basic();
        test_torque_distribution();
        test_locking_torque();
        test_bias_ratio();
        test_real_world_scenario();
        
        print_section("テスト完了");
        std::cout << "\nすべてのテストが正常に完了しました。\n";
        std::cout << "LSDモデルが正しく動作しています。\n";
        
    } catch (const std::exception& e) {
        std::cerr << "エラー: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
