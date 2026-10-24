/// @file test_nn_tire_integration.cpp
/// @brief テキストベースNNタイヤモデル統合テスト

#include "fsae/physics/tire/nn_lookup_tire.hpp"
#include "fsae/physics/tire/tire_thermal_integrator.hpp"
#include "fsae/vehicle/vehicle_14dof_new.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace fsae::physics::tire;

// テスト用のダミー重みファイルを生成
class NNTireModelTest : public ::testing::Test {
protected:
    void SetUp() override {
        create_test_weights();
        create_test_norm_params();
    }
    
    void TearDown() override {
        std::remove("test_weights.txt");
        std::remove("test_norm_params.txt");
    }
    
    void create_test_weights() {
        std::ofstream file("test_weights.txt");
        
        file << "# Test Neural Network Weights\n";
        file << "# 2-layer network: 5 -> 10 -> 4\n\n";
        
        // Layer 1: 5 inputs -> 10 outputs
        file << "# Layer 1: Fully Connected\n";
        file << "LAYER_1_WEIGHTS: 10 5\n";
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 5; j++) {
                file << (0.1 * (i + j)) << " ";
            }
            file << "\n";
        }
        file << "\n";
        
        file << "LAYER_1_BIASES: 10\n";
        for (int i = 0; i < 10; i++) {
            file << (0.01 * i) << "\n";
        }
        file << "\n";
        
        // Layer 2: 10 inputs -> 4 outputs
        file << "# Layer 2: Fully Connected\n";
        file << "LAYER_2_WEIGHTS: 4 10\n";
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 10; j++) {
                file << (0.05 * (i - j)) << " ";
            }
            file << "\n";
        }
        file << "\n";
        
        file << "LAYER_2_BIASES: 4\n";
        for (int i = 0; i < 4; i++) {
            file << (0.1 * i) << "\n";
        }
        
        file.close();
    }
    
    void create_test_norm_params() {
        std::ofstream file("test_norm_params.txt");
        
        file << "# Normalization Parameters\n\n";
        
        file << "INPUT_MEAN:\n";
        file << "0.0 0.0 1000.0 0.0 50.0\n\n";
        
        file << "INPUT_STD:\n";
        file << "0.1 5.0 500.0 2.0 15.0\n\n";
        
        file << "OUTPUT_MEAN:\n";
        file << "0.0 0.0 0.0 0.0\n\n";
        
        file << "OUTPUT_STD:\n";
        file << "500.0 500.0 50.0 25.0\n";
        
        file.close();
    }
};

TEST_F(NNTireModelTest, LoadFromTextFiles) {
    // テスト用のダミーファイルではなく実際のcpp_exportを使う
    GTEST_SKIP() << "Using real cpp_export test instead (see RealModelInference test)";
}

TEST_F(NNTireModelTest, PredictionBasic) {
    GTEST_SKIP() << "Using real cpp_export test instead (see RealModelInference test)";
}

TEST_F(NNTireModelTest, PredictionDeterministic) {
    GTEST_SKIP() << "Using real cpp_export test instead (see RealModelInference test)";
}

// 実際のエクスポートファイルを使ったテスト
TEST(RealNNModel, LoadFromExport) {
    NNTextTireModel model;
    
    // cpp_exportディレクトリから読み込み（相対パス: build/ から見て ../../cpp_export）
    std::string export_dir = "../../cpp_export";
    
    // ファイルの存在確認
    std::ifstream weights_file(export_dir + "/network_weights.txt");
    if (!weights_file.good()) {
        GTEST_SKIP() << "cpp_export files not found at: " << export_dir;
    }
    
    // モデル読み込み
    ASSERT_NO_THROW(model.load_from_export(export_dir));
    EXPECT_TRUE(model.is_loaded());
    EXPECT_EQ(model.num_fc_layers(), 9);  // 9層のFC層
}

TEST(RealNNModel, InferenceBasic) {
    NNTextTireModel model;
    
    std::string export_dir = "../../cpp_export";
    std::ifstream weights_file(export_dir + "/network_weights.txt");
    if (!weights_file.good()) {
        GTEST_SKIP() << "cpp_export files not found";
    }
    
    model.load_from_export(export_dir);
    
    // 典型的な入力でテスト
    TireInput input;
    input.SR = 0.05;      // 5% スリップ率
    input.SA = 3.0;       // 3° スリップ角
    input.Fz = 1000.0;    // 1000N 垂直荷重
    input.IA = 0.0;       // 0° キャンバー角
    input.TSTC = 60.0;    // 60°C タイヤ温度
    
    TireForces forces;
    ASSERT_NO_THROW(forces = model.predict(input));
    
    // 出力が有限値であることを確認
    EXPECT_TRUE(std::isfinite(forces.Fx));
    EXPECT_TRUE(std::isfinite(forces.Fy));
    EXPECT_TRUE(std::isfinite(forces.Mx));
    EXPECT_TRUE(std::isfinite(forces.Mz));
    
    // 力が物理的に妥当な範囲にあるか確認（NN学習済みデータに基づく）
    EXPECT_GT(forces.Fx, -3000.0);
    EXPECT_LT(forces.Fx, 3000.0);
    EXPECT_GT(forces.Fy, -3000.0);
    EXPECT_LT(forces.Fy, 3000.0);
}

TEST(RealNNModel, InferenceDeterministic) {
    NNTextTireModel model;
    
    std::string export_dir = "../../cpp_export";
    std::ifstream weights_file(export_dir + "/network_weights.txt");
    if (!weights_file.good()) {
        GTEST_SKIP() << "cpp_export files not found";
    }
    
    model.load_from_export(export_dir);
    
    TireInput input = {0.05, 3.0, 1000.0, 0.0, 60.0};
    
    // 同じ入力で2回推論
    auto forces1 = model.predict(input);
    auto forces2 = model.predict(input);
    
    // 結果が同一であることを確認（決定的）
    EXPECT_DOUBLE_EQ(forces1.Fx, forces2.Fx);
    EXPECT_DOUBLE_EQ(forces1.Fy, forces2.Fy);
    EXPECT_DOUBLE_EQ(forces1.Mx, forces2.Mx);
    EXPECT_DOUBLE_EQ(forces1.Mz, forces2.Mz);
}

TEST(RealNNModel, InferenceContinuity) {
    NNTextTireModel model;
    
    std::string export_dir = "../../cpp_export";
    std::ifstream weights_file(export_dir + "/network_weights.txt");
    if (!weights_file.good()) {
        GTEST_SKIP() << "cpp_export files not found";
    }
    
    model.load_from_export(export_dir);
    
    // 連続性テスト：わずかに異なる入力で力が連続的に変化するか
    TireInput input1 = {0.05, 3.0, 1000.0, 0.0, 60.0};
    TireInput input2 = {0.051, 3.0, 1000.0, 0.0, 60.0};  // SR を1% 変化
    
    auto f1 = model.predict(input1);
    auto f2 = model.predict(input2);
    
    // 1%の入力変化で力の変化が連続的であることを確認
    double dFx = std::abs(f2.Fx - f1.Fx);
    double dFy = std::abs(f2.Fy - f1.Fy);
    
    // 急激な変化がないことを確認（NN推論の滑らかさ）
    EXPECT_LT(dFx, 100.0);  // 力の変化が100N以下
    EXPECT_LT(dFy, 100.0);
}

TEST(RealNNModel, RuntimePostprocessAppliesZeroSlipCorrectionAndFrictionCap) {
    namespace fs = std::filesystem;

    const fs::path runtime_dir = "test_nn_runtime_postprocess";
    fs::create_directories(runtime_dir);
    const fs::path runtime_path = runtime_dir / "pinn_runtime.json";

    {
        std::ofstream file(runtime_path);
        ASSERT_TRUE(file.is_open());
        file << R"JSON({
  "format": "pinn_runtime_v1",
  "input": {
    "names": ["SR", "SA", "Fz", "IA", "TSTC"],
    "mean": [0.0, 0.0, 0.0, 0.0, 0.0],
    "std": [1.0, 1.0, 1.0, 1.0, 1.0]
  },
  "output": {
    "names": ["Fx", "Fy", "Mx", "Mz"],
    "mean": [0.0, 0.0, 0.0, 0.0],
    "std": [1.0, 1.0, 1.0, 1.0]
  },
  "postprocess": {
    "enabled": true,
    "friction_cap_mu": 2.0,
    "zero_slip_sr_scale": 0.03,
    "zero_slip_sa_scale_deg": 0.5,
    "symmetry_sr_scale": 0.05,
    "symmetry_ia_scale_deg": 0.75,
    "sa_zero_fy_scale_deg": 0.3,
    "epsilon": 1.0e-9
  },
  "layers": [
    {
      "type": "linear",
      "weight": [
        [5000.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1000.0, 0.0, 100.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 50.0, 0.0, 0.0, 0.0]
      ],
      "bias": [3000.0, 1000.0, 10.0, 200.0]
    }
  ]
})JSON";
    }

    NNTextTireModel model;
    ASSERT_NO_THROW(model.load_from_export(runtime_dir.string()));

    const TireInput zero_input {0.0, 0.0, 1000.0, 0.0, 60.0};
    const auto diagnostics_disabled_forces = model.predict(zero_input);
    const auto disabled_diagnostics = model.runtime_diagnostics();
    EXPECT_EQ(disabled_diagnostics.prediction_calls, 0u);
    EXPECT_EQ(disabled_diagnostics.forward_physical_calls, 0u);
    EXPECT_EQ(disabled_diagnostics.postprocess_calls, 0u);

    model.set_runtime_diagnostics_enabled(true);
    model.reset_runtime_diagnostics();
    const auto zero_forces = model.predict(zero_input);
    EXPECT_DOUBLE_EQ(zero_forces.Fx, diagnostics_disabled_forces.Fx);
    EXPECT_DOUBLE_EQ(zero_forces.Fy, diagnostics_disabled_forces.Fy);
    EXPECT_DOUBLE_EQ(zero_forces.Mx, diagnostics_disabled_forces.Mx);
    EXPECT_DOUBLE_EQ(zero_forces.Mz, diagnostics_disabled_forces.Mz);
    EXPECT_NEAR(zero_forces.Fx, 0.0, 1e-9);
    EXPECT_NEAR(zero_forces.Fy, 0.0, 1e-9);
    EXPECT_NEAR(zero_forces.Mz, 0.0, 1e-9);
    EXPECT_NEAR(zero_forces.Mx, 10.0, 1e-9);

    const auto zero_diagnostics = model.runtime_diagnostics();
    EXPECT_EQ(zero_diagnostics.prediction_calls, 1u);
    EXPECT_EQ(zero_diagnostics.forward_physical_calls, 4u);
    EXPECT_EQ(zero_diagnostics.postprocess_calls, 1u);
    EXPECT_EQ(zero_diagnostics.zero_reference_forwards, 1u);
    EXPECT_EQ(zero_diagnostics.symmetry_forwards, 1u);
    EXPECT_EQ(zero_diagnostics.sa_zero_forwards, 1u);
    EXPECT_EQ(zero_diagnostics.zero_slip_projections, 3u);
    EXPECT_EQ(zero_diagnostics.symmetry_projections, 1u);
    EXPECT_EQ(zero_diagnostics.sa_zero_projections, 1u);
    EXPECT_EQ(zero_diagnostics.friction_cap_projections, 0u);

    model.reset_runtime_diagnostics();
    model.set_runtime_diagnostics_enabled(false);
    EXPECT_EQ(model.runtime_diagnostics().prediction_calls, 0u);

    const TireInput high_slip_input {0.2, 2.0, 1000.0, 0.0, 60.0};
    const auto high_slip_forces = model.predict(high_slip_input);
    const double combined_force =
        std::sqrt(high_slip_forces.Fx * high_slip_forces.Fx + high_slip_forces.Fy * high_slip_forces.Fy);
    EXPECT_NEAR(combined_force, 2000.0, 1e-6);
    EXPECT_NEAR(high_slip_forces.Mx, 10.0, 1e-9);
    EXPECT_GT(std::abs(high_slip_forces.Mz), 250.0);

    const TireInput sym_pos_input {0.0, 2.0, 1000.0, 0.0, 60.0};
    const TireInput sym_neg_input {0.0, -2.0, 1000.0, 0.0, 60.0};
    const auto sym_pos_forces = model.predict(sym_pos_input);
    const auto sym_neg_forces = model.predict(sym_neg_input);
    EXPECT_NEAR(sym_pos_forces.Fx, sym_neg_forces.Fx, 1e-9);
    EXPECT_NEAR(sym_pos_forces.Fy, -sym_neg_forces.Fy, 1e-9);
    EXPECT_NEAR(sym_pos_forces.Mz, -sym_neg_forces.Mz, 1e-9);

    const TireInput camber_pos_input {0.0, 0.0, 1000.0, 3.0, 60.0};
    const TireInput camber_neg_input {0.0, 0.0, 1000.0, -3.0, 60.0};
    const auto camber_pos_forces = model.predict(camber_pos_input);
    const auto camber_neg_forces = model.predict(camber_neg_input);
    EXPECT_GT(camber_pos_forces.Fy, 100.0);
    EXPECT_LT(camber_neg_forces.Fy, -100.0);
    EXPECT_NEAR(camber_pos_forces.Fy, -camber_neg_forces.Fy, 1e-6);

    fs::remove_all(runtime_dir);
}

TEST(RealNNModel, MultiHeadRuntimeJsonInference) {
    namespace fs = std::filesystem;

    const fs::path runtime_dir = "test_nn_runtime_multi_head";
    fs::create_directories(runtime_dir);
    const fs::path runtime_path = runtime_dir / "pinn_runtime.json";

    {
        std::ofstream file(runtime_path);
        ASSERT_TRUE(file.is_open());
        file << R"JSON({
  "format": "pinn_runtime_v2",
  "input": {
    "names": ["SR", "SA", "Fz", "IA", "TSTC"],
    "mean": [0.0, 0.0, 0.0, 0.0, 0.0],
    "std": [1.0, 1.0, 1.0, 1.0, 1.0]
  },
  "output": {
    "names": ["Fx", "Fy", "Mx", "Mz"],
    "mean": [0.0, 0.0, 0.0, 0.0],
    "std": [1.0, 1.0, 1.0, 1.0]
  },
  "architecture": {"type": "multi_head"},
  "trunk_layers": [
    {
      "type": "linear",
      "weight": [
        [1.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0, 0.0]
      ],
      "bias": [0.0, 0.0]
    }
  ],
  "heads": [
    {
      "name": "fx",
      "output_indices": [0],
      "layers": [
        {"type": "linear", "weight": [[1000.0, 0.0]], "bias": [10.0]}
      ]
    },
    {
      "name": "fy_mz",
      "output_indices": [1, 3],
      "layers": [
        {
          "type": "linear",
          "weight": [[0.0, 100.0], [0.0, 5.0]],
          "bias": [20.0, 30.0]
        }
      ]
    },
    {
      "name": "mx",
      "output_indices": [2],
      "layers": [
        {"type": "linear", "weight": [[0.0, 0.0]], "bias": [7.0]}
      ]
    }
  ]
})JSON";
    }

    NNTextTireModel model;
    ASSERT_NO_THROW(model.load_from_export(runtime_dir.string()));
    EXPECT_TRUE(model.is_loaded());
    EXPECT_EQ(model.num_fc_layers(), 4);
    EXPECT_TRUE(model.is_multi_head_runtime());
    EXPECT_EQ(model.runtime_description(), "pinn_runtime_v2_multi_head");

    const TireInput input {0.1, 2.0, 1000.0, 0.0, 60.0};
    const auto forces = model.predict(input);
    EXPECT_NEAR(forces.Fx, 110.0, 1e-12);
    EXPECT_NEAR(forces.Fy, 220.0, 1e-12);
    EXPECT_NEAR(forces.Mx, 7.0, 1e-12);
    EXPECT_NEAR(forces.Mz, 40.0, 1e-12);

    // Reused inference buffers must not leak state across heads or calls.
    const auto repeated_forces = model.predict(input);
    EXPECT_DOUBLE_EQ(repeated_forces.Fx, forces.Fx);
    EXPECT_DOUBLE_EQ(repeated_forces.Fy, forces.Fy);
    EXPECT_DOUBLE_EQ(repeated_forces.Mx, forces.Mx);
    EXPECT_DOUBLE_EQ(repeated_forces.Mz, forces.Mz);

    const auto forces_without_mx = model.predict(input, false);
    EXPECT_DOUBLE_EQ(forces_without_mx.Fx, forces.Fx);
    EXPECT_DOUBLE_EQ(forces_without_mx.Fy, forces.Fy);
    EXPECT_DOUBLE_EQ(forces_without_mx.Mx, 0.0);
    EXPECT_DOUBLE_EQ(forces_without_mx.Mz, forces.Mz);

    fs::remove_all(runtime_dir);
}

TEST(RealNNModel, RequestedPinnVehicleFailsWhenRuntimeMissing) {
    fsae::vehicle::VehicleParams params = fsae::vehicle::VehicleParams::default_fsae();
    params.tire.model_type = "pinn";
    params.tire.model_name = "missing_test_model";
    params.tire.model_manifest = "definitely_missing_pinn_manifest.json";

    EXPECT_THROW(
        {
            fsae::vehicle::Vehicle14DOF_New vehicle(params);
            (void)vehicle;
        },
        std::runtime_error);
}

TEST(RealNNModel, DefaultPinnVehicleUsesSafeRuntimeConventions) {
    const fsae::vehicle::VehicleParams params = fsae::vehicle::VehicleParams::default_fsae();
    const auto parsed_params = fsae::vehicle::VehicleParams::from_json_string(R"({
        "tire": {"pinn_eval_rate_hz": 1000.0}
    })");

    EXPECT_EQ(params.tire.model_type, "pinn");
    EXPECT_FALSE(params.tire.pinn_use_camber_input);
    EXPECT_TRUE(params.tire.pinn_ttc_lateral_sign);
    EXPECT_DOUBLE_EQ(params.tire.pinn_eval_rate_hz, 500.0);
    EXPECT_DOUBLE_EQ(parsed_params.tire.pinn_eval_rate_hz, 1000.0);
}

TEST(RealNNModel, MultirateZohControlsPredictionCount) {
    namespace fs = std::filesystem;
    const fs::path manifest = "output/tire_models/Hoosier_18p0X6p0-10_R20_Rim6/manifest.json";
    if (!fs::exists(manifest) && !fs::exists(fs::path("..") / manifest) &&
        !fs::exists(fs::path("../..") / manifest)) {
        GTEST_SKIP() << "requires generated PINN tire model: " << manifest;
    }

    constexpr int kSteps = 1000;
    auto count_predictions = [](double eval_rate_hz) {
        auto params = fsae::vehicle::VehicleParams::default_fsae();
        params.tire.pinn_eval_rate_hz = eval_rate_hz;
        fsae::vehicle::Vehicle14DOF_New vehicle(params);
        vehicle.reset();
        vehicle.set_initial_velocity(5.0);
        vehicle.set_nn_tire_runtime_diagnostics_enabled(true);
        vehicle.reset_nn_tire_runtime_diagnostics();

        fsae::vehicle::DriverInput input;
        input.throttle = 0.5;
        for (int i = 0; i < kSteps; ++i) {
            vehicle.step(0.001, input);
        }
        return vehicle.nn_tire_runtime_diagnostics().prediction_calls;
    };

    EXPECT_EQ(count_predictions(1000.0), 4004u);
    EXPECT_EQ(count_predictions(500.0), 2004u);
    EXPECT_EQ(count_predictions(20.0), 84u);

    auto reset_params = fsae::vehicle::VehicleParams::default_fsae();
    reset_params.tire.pinn_eval_rate_hz = 20.0;
    fsae::vehicle::Vehicle14DOF_New reset_vehicle(reset_params);
    reset_vehicle.set_nn_tire_runtime_diagnostics_enabled(true);
    reset_vehicle.reset_nn_tire_runtime_diagnostics();
    reset_vehicle.reset();
    EXPECT_EQ(reset_vehicle.nn_tire_runtime_diagnostics().prediction_calls, 4u);

    reset_vehicle.reset_nn_tire_runtime_diagnostics();
    reset_vehicle.set_initial_velocity(5.0);
    reset_vehicle.step(0.001, {});
    EXPECT_EQ(reset_vehicle.nn_tire_runtime_diagnostics().prediction_calls, 4u);

    reset_vehicle.reset_nn_tire_runtime_diagnostics();
    reset_vehicle.state().vx += 1.0;
    reset_vehicle.step(0.001, {});
    EXPECT_EQ(reset_vehicle.nn_tire_runtime_diagnostics().prediction_calls, 4u);
}

TEST(RealNNModel, RequestedPinnVehicleFailsWhenRuntimeUnspecified) {
    fsae::vehicle::VehicleParams params = fsae::vehicle::VehicleParams::default_fsae();
    params.tire.model_type = "pinn";
    params.tire.model_name = "unspecified_runtime_test_model";
    params.tire.model_manifest.clear();
    params.tire.model_dir.clear();

    EXPECT_THROW(
        {
            fsae::vehicle::Vehicle14DOF_New vehicle(params);
            (void)vehicle;
        },
        std::runtime_error);
}

TEST(RealNNModel, NonPinnVehicleTireRuntimeIsRejected) {
    fsae::vehicle::VehicleParams params = fsae::vehicle::VehicleParams::default_fsae();
    params.tire.model_type = "tmeasy";

    EXPECT_THROW(
        {
            fsae::vehicle::Vehicle14DOF_New vehicle(params);
            (void)vehicle;
        },
        std::runtime_error);
}

TEST(RealNNModel, ExportedRuntimeMatchesTestVectors) {
    namespace fs = std::filesystem;

    const fs::path source_root = fs::path(__FILE__).parent_path().parent_path().parent_path();
    const fs::path export_dir =
        source_root / "output" / "tire_models" / "Hoosier_18p0X6p0-10_R20_Rim6";
    const fs::path manifest_path = export_dir / "manifest.json";
    const fs::path vectors_path = export_dir / "test_vectors.json";
    if (!fs::exists(manifest_path) || !fs::exists(vectors_path)) {
        GTEST_SKIP() << "Exported PINN runtime assets not found at: " << export_dir;
    }

    NNTextTireModel model;
    ASSERT_NO_THROW(model.load_from_manifest(manifest_path.string()));

    nlohmann::json vectors_json;
    {
        std::ifstream file(vectors_path);
        ASSERT_TRUE(file.is_open());
        file >> vectors_json;
    }

    const double abs_tol = vectors_json["tolerance"].value("absolute", 1e-6);
    const double rel_tol = vectors_json["tolerance"].value("relative", 1e-4);
    const auto& inputs = vectors_json["inputs"];
    const auto& outputs = vectors_json["outputs"];
    ASSERT_EQ(inputs.size(), outputs.size());

    for (size_t i = 0; i < inputs.size(); ++i) {
        TireInput input {
            inputs[i][0].get<double>(),
            inputs[i][1].get<double>(),
            inputs[i][2].get<double>(),
            inputs[i][3].get<double>(),
            inputs[i][4].get<double>(),
        };
        const TireForces predicted = model.predict(input);
        const std::array<double, 4> expected = {
            outputs[i][0].get<double>(),
            outputs[i][1].get<double>(),
            outputs[i][2].get<double>(),
            outputs[i][3].get<double>(),
        };
        const std::array<double, 4> actual = {predicted.Fx, predicted.Fy, predicted.Mx, predicted.Mz};

        for (size_t j = 0; j < expected.size(); ++j) {
            const double tolerance = abs_tol + rel_tol * std::abs(expected[j]);
            EXPECT_NEAR(actual[j], expected[j], tolerance)
                << "Mismatch at vector " << i << ", component " << j;
        }
    }
}

// タイヤ熱モデルのテスト
class TireThermalTest : public ::testing::Test {
protected:
    TireThermalParams params;
    
    void SetUp() override {
        params = TireThermalParams();
    }
};

TEST_F(TireThermalTest, Initialization) {
    TireThermalIntegrator thermal(params);
    
    for (int i = 0; i < 4; i++) {
        EXPECT_DOUBLE_EQ(thermal.get_temperature(i), params.T_initial);
    }
}

TEST_F(TireThermalTest, HeatingFromFriction) {
    TireThermalIntegrator thermal(params);
    
    std::array<TireForces, 4> forces;
    std::array<TireSlipInfo, 4> slip_info;
    
    for (int i = 0; i < 4; i++) {
        forces[i].Fx = 500.0;
        forces[i].Fy = 300.0;
        slip_info[i].vsx = 2.0;
        slip_info[i].vsy = 1.0;
    }
    
    double T_initial = thermal.get_temperature(0);
    
    double dt = 0.01;
    for (int step = 0; step < 1000; step++) {
        thermal.update(dt, forces, slip_info, 20.0, 25.0);
    }
    
    double T_final = thermal.get_temperature(0);
    EXPECT_GT(T_final, T_initial);
}

TEST_F(TireThermalTest, CoolingFromConvection) {
    TireThermalIntegrator thermal(params);
    thermal.reset(80.0);
    
    std::array<TireForces, 4> forces{};
    std::array<TireSlipInfo, 4> slip_info{};
    
    double T_initial = thermal.get_temperature(0);
    
    double dt = 0.01;
    for (int step = 0; step < 1000; step++) {
        thermal.update(dt, forces, slip_info, 20.0, 25.0);
    }
    
    double T_final = thermal.get_temperature(0);
    EXPECT_LT(T_final, T_initial);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
