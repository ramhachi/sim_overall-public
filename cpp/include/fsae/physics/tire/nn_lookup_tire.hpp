#pragma once
/// @file nn_lookup_tire.hpp
/// @brief テキスト/JSONベースNNタイヤモデルローダー

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fsae::physics::tire {

struct TireForces {
    double Fx = 0.0;
    double Fy = 0.0;
    double Mx = 0.0;
    double Mz = 0.0;
};

struct TireInput {
    double SR;
    double SA;
    double Fz;
    double IA;
    double TSTC;
};

/// Opt-in counters for investigating runtime postprocess amplification.
///
/// Diagnostics are per model instance. Snapshot or reset them only while
/// inference on that instance is paused; predict() already uses mutable scratch
/// buffers and does not support concurrent calls on the same instance.
struct NNRuntimeDiagnostics {
    uint64_t prediction_calls = 0;
    uint64_t forward_physical_calls = 0;
    uint64_t postprocess_calls = 0;
    uint64_t zero_reference_forwards = 0;
    uint64_t symmetry_forwards = 0;
    uint64_t sa_zero_forwards = 0;
    uint64_t zero_slip_projections = 0;
    uint64_t symmetry_projections = 0;
    uint64_t sa_zero_projections = 0;
    uint64_t friction_cap_projections = 0;
    /// predict() calls with at least one input outside input_bounds
    /// (rate = out_of_domain_calls / prediction_calls).
    uint64_t out_of_domain_calls = 0;
    /// OR of bits seen: bit i set when input i (SR, SA, Fz, IA, TSTC) left its bounds.
    uint32_t out_of_domain_mask = 0;
};

struct FCLayer {
    std::vector<std::vector<double>> weights;
    std::vector<double> weights_flat;
    std::vector<double> biases;

    int n_in = 0;
    int n_out = 0;

    std::vector<double> forward(const std::vector<double>& input) const {
        if (static_cast<int>(input.size()) != n_in) {
            throw std::runtime_error("NNTextTireModel: input size mismatch");
        }

        std::vector<double> output(n_out, 0.0);
        for (int i = 0; i < n_out; ++i) {
            output[i] = biases[i];
            if (!weights_flat.empty()) {
                const double* row = weights_flat.data() + static_cast<size_t>(i) * n_in;
                for (int j = 0; j < n_in; ++j) {
                    output[i] += row[j] * input[j];
                }
            } else {
                for (int j = 0; j < n_in; ++j) {
                    output[i] += weights[i][j] * input[j];
                }
            }
        }
        return output;
    }
};

struct BatchNormLayer {
    std::vector<double> gamma;
    std::vector<double> beta;
    std::vector<double> mean;
    std::vector<double> var;
    double epsilon = 1e-5;

    int n_channels = 0;

    std::vector<double> forward(const std::vector<double>& input) const {
        if (static_cast<int>(input.size()) != n_channels) {
            throw std::runtime_error("NNTextTireModel: batch norm input size mismatch");
        }

        std::vector<double> output(n_channels, 0.0);
        for (int i = 0; i < n_channels; ++i) {
            output[i] =
                gamma[i] * (input[i] - mean[i]) / std::sqrt(var[i] + epsilon) + beta[i];
        }
        return output;
    }
};

struct RuntimeLayer {
    enum class Type { Linear, LayerNorm, Swish, ReLU };

    Type type = Type::Linear;
    std::vector<std::vector<double>> weights;
    std::vector<double> weights_flat;
    std::vector<double> bias;
    double epsilon = 1e-5;
    size_t n_in = 0;
    size_t n_out = 0;
};

struct RuntimeHead {
    std::string name;
    std::vector<size_t> output_indices;
    std::vector<RuntimeLayer> layers;
};

class NNTextTireModel {
public:
    NNTextTireModel() = default;

    void load_from_export(const std::string& base_dir) {
        namespace fs = std::filesystem;
        const fs::path root(base_dir);
        const fs::path manifest_path = root / "manifest.json";
        const fs::path runtime_path = root / "pinn_runtime.json";
        use_runtime_postprocess_ = false;
        friction_cap_mu_ = 0.0;
        zero_slip_sr_scale_ = 0.0;
        zero_slip_sa_scale_deg_ = 0.0;
        postprocess_epsilon_ = 1e-9;
        runtime_is_multi_head_ = false;
        runtime_trunk_layers_.clear();
        runtime_heads_.clear();

        if (fs::exists(manifest_path)) {
            load_from_manifest(manifest_path.string());
            return;
        }

        if (fs::exists(runtime_path)) {
            load_runtime_json(runtime_path.string());
            loaded_ = true;
            return;
        }

        load_normalization((root / "normalization_params.txt").string());
        load_weights((root / "network_weights.txt").string());
        load_architecture((root / "network_architecture.txt").string());
        runtime_layers_.clear();
        loaded_ = true;
    }

    void load_from_manifest(const std::string& manifest_path) {
        namespace fs = std::filesystem;

        std::ifstream file(manifest_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open manifest file: " + manifest_path);
        }

        nlohmann::json manifest;
        file >> manifest;

        if (manifest.contains("deployable") && !manifest["deployable"].get<bool>()) {
            throw std::runtime_error("Requested PINN manifest is marked non-deployable");
        }

        fs::path runtime_file = fs::path(manifest_path).parent_path() / "pinn_runtime.json";
        if (manifest.contains("runtime_files") &&
            manifest["runtime_files"].contains("runtime_model")) {
            runtime_file =
                fs::path(manifest_path).parent_path() /
                manifest["runtime_files"]["runtime_model"].get<std::string>();
        }

        load_runtime_json(runtime_file.string());
        loaded_ = true;
    }

    TireForces predict(const TireInput& input, bool include_mx = true) const {
        if (!loaded_) {
            throw std::runtime_error("NNTextTireModel: model not loaded");
        }
        if (runtime_diagnostics_enabled_) {
            ++runtime_diagnostics_.prediction_calls;
        }

        std::array<double, 5> x = {input.SR, input.SA, input.Fz, input.IA, input.TSTC};
        double sr_decay = 1.0;
        double sa_decay = 1.0;
        double fz_gate = 1.0;
        if (has_input_bounds_) {
            uint32_t ood = 0;
            for (size_t i = 0; i < x.size(); ++i) {
                if (x[i] < input_min_[i] || x[i] > input_max_[i]) {
                    ood |= 1u << i;
                }
                x[i] = std::clamp(x[i], input_min_[i], input_max_[i]);
            }
            if (ood != 0 && runtime_diagnostics_enabled_) {
                ++runtime_diagnostics_.out_of_domain_calls;
                runtime_diagnostics_.out_of_domain_mask |= ood;
            }
            sr_decay = post_peak_decay(input.SR, 0);
            sa_decay = post_peak_decay(input.SA, 1);
            // Below the trained Fz floor fade every output to zero so a lifted
            // wheel (Fz=0) produces no force instead of the Fz_min force.
            if (input_min_[2] > 0.0 && input.Fz < input_min_[2]) {
                const double t = std::max(input.Fz, 0.0) / input_min_[2];
                fz_gate = t * t * (3.0 - 2.0 * t);
            }
            // ponytail: Fz above input_max still holds the Fz_max output; extend the
            // trained domain with TTC high-load runs before extrapolating here.
        }
        std::array<double, 4> y = forward_physical(x, include_mx);
        apply_runtime_postprocess(x, y, include_mx);
        y[0] *= sr_decay;
        y[1] *= sa_decay;
        y[3] *= sa_decay;
        for (double& v : y) {
            v *= fz_gate;
        }

        TireForces forces;
        forces.Fx = y[0];
        forces.Fy = y[1];
        forces.Mx = y[2];
        forces.Mz = y[3];
        return forces;
    }

    bool is_loaded() const { return loaded_; }

    bool is_multi_head_runtime() const { return runtime_is_multi_head_; }

    void set_runtime_diagnostics_enabled(bool enabled) {
        runtime_diagnostics_enabled_ = enabled;
    }

    bool runtime_diagnostics_enabled() const { return runtime_diagnostics_enabled_; }

    NNRuntimeDiagnostics runtime_diagnostics() const { return runtime_diagnostics_; }

    void reset_runtime_diagnostics() const { runtime_diagnostics_ = {}; }

    std::string runtime_description() const {
        if (runtime_is_multi_head_) {
            return "pinn_runtime_v2_multi_head";
        }
        if (!runtime_layers_.empty()) {
            return "pinn_runtime_v1";
        }
        return "legacy_text_export";
    }

    int num_fc_layers() const {
        if (runtime_is_multi_head_) {
            int count = 0;
            for (const auto& layer : runtime_trunk_layers_) {
                if (layer.type == RuntimeLayer::Type::Linear) {
                    ++count;
                }
            }
            for (const auto& head : runtime_heads_) {
                for (const auto& layer : head.layers) {
                    if (layer.type == RuntimeLayer::Type::Linear) {
                        ++count;
                    }
                }
            }
            return count;
        }
        if (!runtime_layers_.empty()) {
            int count = 0;
            for (const auto& layer : runtime_layers_) {
                if (layer.type == RuntimeLayer::Type::Linear) {
                    ++count;
                }
            }
            return count;
        }
        return static_cast<int>(fc_layers_.size());
    }

private:
    std::vector<FCLayer> fc_layers_;
    std::vector<BatchNormLayer> bn_layers_;
    std::vector<RuntimeLayer> runtime_layers_;
    std::vector<RuntimeLayer> runtime_trunk_layers_;
    std::vector<RuntimeHead> runtime_heads_;
    std::array<double, 5> input_mean_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 5> input_std_ = {1.0, 1.0, 1.0, 1.0, 1.0};
    std::array<double, 5> input_min_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 5> input_max_ = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> output_mean_ = {0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> output_std_ = {1.0, 1.0, 1.0, 1.0};
    bool has_input_bounds_ = false;
    bool runtime_is_multi_head_ = false;
    bool use_runtime_postprocess_ = false;
    double friction_cap_mu_ = 0.0;
    double zero_slip_sr_scale_ = 0.0;
    double zero_slip_sa_scale_deg_ = 0.0;
    double symmetry_sr_scale_ = 0.0;
    double symmetry_ia_scale_deg_ = 0.0;
    double sa_zero_fy_scale_deg_ = 0.0;
    double postprocess_epsilon_ = 1e-9;
    bool loaded_ = false;
    bool runtime_diagnostics_enabled_ = false;
    mutable NNRuntimeDiagnostics runtime_diagnostics_;
    mutable std::vector<double> runtime_eval_b_;
    mutable std::vector<double> runtime_input_buffer_;
    mutable std::vector<double> runtime_output_buffer_;
    mutable std::vector<double> runtime_feature_buffer_;
    mutable std::vector<double> runtime_head_buffer_;

    static double swish(double x) { return x / (1.0 + std::exp(-x)); }

    /// Monotone post-peak decay for |slip| beyond the trained bound: the bound
    /// value decays toward kSlidingRatio of itself with an e-folding length equal
    /// to the half-width of the trained range (13 deg SA / 0.5 SR for TTC).
    /// Basis: racing-slick force past peak falls to roughly 0.8-0.9 of peak in
    /// full slide (TTC Hoosier sweeps); 0.8 is the conservative end.
    double post_peak_decay(double value, size_t i) const {
        constexpr double kSlidingRatio = 0.8;
        const double excess = value > input_max_[i]   ? value - input_max_[i]
                              : value < input_min_[i] ? input_min_[i] - value
                                                      : 0.0;
        const double scale = 0.5 * (input_max_[i] - input_min_[i]);
        if (excess <= 0.0 || scale <= 0.0) {
            return 1.0;
        }
        return kSlidingRatio + (1.0 - kSlidingRatio) * std::exp(-excess / scale);
    }

    static std::vector<double> flatten_weights(const std::vector<std::vector<double>>& weights) {
        std::vector<double> flat;
        size_t total = 0;
        for (const auto& row : weights) {
            total += row.size();
        }
        flat.reserve(total);
        for (const auto& row : weights) {
            flat.insert(flat.end(), row.begin(), row.end());
        }
        return flat;
    }

    static void layer_norm_into(
        const std::vector<double>& input,
        const std::vector<double>& weight,
        const std::vector<double>& bias,
        double epsilon,
        std::vector<double>& output) {
        if (input.size() != weight.size() || input.size() != bias.size()) {
            throw std::runtime_error("NNTextTireModel: layer norm size mismatch");
        }

        double mean = 0.0;
        for (double value : input) {
            mean += value;
        }
        mean /= static_cast<double>(input.size());

        double variance = 0.0;
        for (double value : input) {
            const double centered = value - mean;
            variance += centered * centered;
        }
        variance /= static_cast<double>(input.size());

        const double denom = 1.0 / std::sqrt(variance + epsilon);
        output.resize(input.size());
        for (size_t i = 0; i < input.size(); ++i) {
            output[i] = weight[i] * (input[i] - mean) * denom + bias[i];
        }
    }

    void forward_runtime_layers_into(
        const std::vector<RuntimeLayer>& layers,
        const std::vector<double>& input,
        std::vector<double>& output) const {
        output.assign(input.begin(), input.end());
        for (const auto& layer : layers) {
            switch (layer.type) {
            case RuntimeLayer::Type::Linear: {
                if (layer.weights.empty()) {
                    throw std::runtime_error("NNTextTireModel: empty runtime linear layer");
                }

                runtime_eval_b_.resize(layer.n_out);
                if (!layer.weights_flat.empty()) {
                    using RowMajorMatrix = Eigen::Matrix<
                        double,
                        Eigen::Dynamic,
                        Eigen::Dynamic,
                        Eigen::RowMajor>;
                    const Eigen::Map<const RowMajorMatrix> weights(
                        layer.weights_flat.data(),
                        static_cast<Eigen::Index>(layer.n_out),
                        static_cast<Eigen::Index>(layer.n_in));
                    const Eigen::Map<const Eigen::VectorXd> input_map(
                        output.data(),
                        static_cast<Eigen::Index>(layer.n_in));
                    Eigen::Map<Eigen::VectorXd> result_map(
                        runtime_eval_b_.data(),
                        static_cast<Eigen::Index>(layer.n_out));
                    result_map.noalias() = weights * input_map;
                    if (!layer.bias.empty()) {
                        result_map += Eigen::Map<const Eigen::VectorXd>(
                            layer.bias.data(),
                            static_cast<Eigen::Index>(layer.n_out));
                    }
                } else {
                    for (size_t i = 0; i < layer.n_out; ++i) {
                        runtime_eval_b_[i] = layer.bias.empty() ? 0.0 : layer.bias[i];
                        for (size_t j = 0; j < layer.n_in; ++j) {
                            runtime_eval_b_[i] += layer.weights[i][j] * output[j];
                        }
                    }
                }
                output.swap(runtime_eval_b_);
                break;
            }
            case RuntimeLayer::Type::LayerNorm:
                layer_norm_into(
                    output,
                    layer.weights.front(),
                    layer.bias,
                    layer.epsilon,
                    runtime_eval_b_);
                output.swap(runtime_eval_b_);
                break;
            case RuntimeLayer::Type::Swish:
                for (double& value : output) {
                    value = swish(value);
                }
                break;
            case RuntimeLayer::Type::ReLU:
                for (double& value : output) {
                    value = std::max(0.0, value);
                }
                break;
            }
        }
    }

    void forward_multi_head_runtime_into(
        const std::vector<double>& input,
        std::vector<double>& output,
        bool include_mx) const {
        forward_runtime_layers_into(runtime_trunk_layers_, input, runtime_feature_buffer_);
        output.assign(4, 0.0);
        for (const auto& head : runtime_heads_) {
            if (!include_mx && head.output_indices.size() == 1 && head.output_indices[0] == 2) {
                continue;
            }
            forward_runtime_layers_into(head.layers, runtime_feature_buffer_, runtime_head_buffer_);
            if (runtime_head_buffer_.size() != head.output_indices.size()) {
                throw std::runtime_error(
                    "NNTextTireModel: multi-head output size mismatch for head " + head.name);
            }
            for (size_t i = 0; i < head.output_indices.size(); ++i) {
                const size_t output_index = head.output_indices[i];
                if (output_index >= output.size()) {
                    throw std::runtime_error("NNTextTireModel: invalid multi-head output index");
                }
                output[output_index] = runtime_head_buffer_[i];
            }
        }
    }

    std::array<double, 4> forward_physical(
        const std::array<double, 5>& physical_input,
        bool include_mx = true) const {
        if (runtime_diagnostics_enabled_) {
            ++runtime_diagnostics_.forward_physical_calls;
        }
        runtime_input_buffer_.resize(physical_input.size());
        for (size_t i = 0; i < 5; ++i) {
            runtime_input_buffer_[i] =
                (physical_input[i] - input_mean_[i]) / input_std_[i];
        }

        if (runtime_is_multi_head_) {
            forward_multi_head_runtime_into(runtime_input_buffer_, runtime_output_buffer_, include_mx);
        } else if (!runtime_layers_.empty()) {
            forward_runtime_layers_into(runtime_layers_, runtime_input_buffer_, runtime_output_buffer_);
        } else {
            runtime_output_buffer_ = forward_legacy(runtime_input_buffer_);
        }

        std::array<double, 4> output = {0.0, 0.0, 0.0, 0.0};
        for (size_t i = 0; i < 4; ++i) {
            output[i] = runtime_output_buffer_[i] * output_std_[i] + output_mean_[i];
        }
        return output;
    }

    void apply_runtime_postprocess(
        const std::array<double, 5>& physical_input,
        std::array<double, 4>& output,
        bool include_mx = true) const {
        if (!use_runtime_postprocess_) {
            return;
        }
        if (runtime_diagnostics_enabled_) {
            ++runtime_diagnostics_.postprocess_calls;
        }

        const bool use_zero_slip =
            zero_slip_sr_scale_ > 0.0 && zero_slip_sa_scale_deg_ > 0.0;
        constexpr double kPostprocessGateCutoff = 1e-8;
        auto zero_slip_gate = [&](const std::array<double, 5>& gate_input) {
            if (!use_zero_slip) {
                return 0.0;
            }
            const double slip_metric =
                std::pow(gate_input[0] / zero_slip_sr_scale_, 2) +
                std::pow(gate_input[1] / zero_slip_sa_scale_deg_, 2);
            return std::exp(-slip_metric);
        };

        const double physical_zero_gate = zero_slip_gate(physical_input);

        double symmetry_gate = 0.0;
        const bool use_symmetry = symmetry_sr_scale_ > 0.0 && symmetry_ia_scale_deg_ > 0.0;
        if (use_symmetry) {
            const double symmetry_metric =
                std::pow(physical_input[0] / symmetry_sr_scale_, 2) +
                std::pow(physical_input[3] / symmetry_ia_scale_deg_, 2);
            symmetry_gate = std::exp(-symmetry_metric);
        }
        const bool needs_symmetry = symmetry_gate > kPostprocessGateCutoff;

        double residual_gate = 0.0;
        const bool use_sa_zero = sa_zero_fy_scale_deg_ > 0.0;
        if (use_sa_zero) {
            const double sa_normalized = physical_input[1] / sa_zero_fy_scale_deg_;
            double fy_metric = sa_normalized * sa_normalized;
            if (symmetry_ia_scale_deg_ > 0.0) {
                const double ia_normalized = physical_input[3] / symmetry_ia_scale_deg_;
                fy_metric += ia_normalized * ia_normalized;
            }
            residual_gate = std::exp(-fy_metric);
        }
        const bool needs_sa_zero = residual_gate > kPostprocessGateCutoff;

        std::array<double, 5> zero_fy_gate_input = physical_input;
        zero_fy_gate_input[1] = 0.0;
        zero_fy_gate_input[3] = 0.0;
        const double zero_fy_zero_gate = needs_sa_zero ? zero_slip_gate(zero_fy_gate_input) : 0.0;

        const bool needs_zero_reference =
            use_zero_slip &&
            (physical_zero_gate > kPostprocessGateCutoff ||
             (needs_symmetry && physical_zero_gate > kPostprocessGateCutoff) ||
             zero_fy_zero_gate > kPostprocessGateCutoff);

        std::array<double, 4> zero_reference_output = {0.0, 0.0, 0.0, 0.0};
        if (needs_zero_reference) {
            if (runtime_diagnostics_enabled_) {
                ++runtime_diagnostics_.zero_reference_forwards;
            }
            std::array<double, 5> zero_reference_input = physical_input;
            zero_reference_input[0] = 0.0;
            zero_reference_input[1] = 0.0;
            zero_reference_input[3] = 0.0;
            if (has_input_bounds_) {
                zero_reference_input[0] =
                    std::clamp(zero_reference_input[0], input_min_[0], input_max_[0]);
                zero_reference_input[1] =
                    std::clamp(zero_reference_input[1], input_min_[1], input_max_[1]);
                zero_reference_input[3] =
                    std::clamp(zero_reference_input[3], input_min_[3], input_max_[3]);
            }
            zero_reference_output = forward_physical(zero_reference_input, include_mx);
        }

        auto apply_zero_slip_projection = [&](double gate, std::array<double, 4>& values) {
            if (!needs_zero_reference || gate <= kPostprocessGateCutoff) {
                return;
            }
            if (runtime_diagnostics_enabled_) {
                ++runtime_diagnostics_.zero_slip_projections;
            }
            values[0] -= gate * zero_reference_output[0];
            values[1] -= gate * zero_reference_output[1];
            values[3] -= gate * zero_reference_output[3];
        };

        apply_zero_slip_projection(physical_zero_gate, output);

        if (needs_symmetry) {
            if (runtime_diagnostics_enabled_) {
                ++runtime_diagnostics_.symmetry_forwards;
                ++runtime_diagnostics_.symmetry_projections;
            }
            std::array<double, 5> mirror_input = physical_input;
            mirror_input[1] = -mirror_input[1];
            if (has_input_bounds_) {
                mirror_input[1] = std::clamp(mirror_input[1], input_min_[1], input_max_[1]);
            }

            std::array<double, 4> mirror_output = forward_physical(mirror_input, include_mx);
            apply_zero_slip_projection(physical_zero_gate, mirror_output);

            const double projected_fx = 0.5 * (output[0] + mirror_output[0]);
            const double projected_fy = 0.5 * (output[1] - mirror_output[1]);
            const double projected_mz = 0.5 * (output[3] - mirror_output[3]);

            output[0] += symmetry_gate * (projected_fx - output[0]);
            output[1] += symmetry_gate * (projected_fy - output[1]);
            if (include_mx) {
                const double projected_mx = 0.5 * (output[2] + mirror_output[2]);
                output[2] += symmetry_gate * (projected_mx - output[2]);
            }
            output[3] += symmetry_gate * (projected_mz - output[3]);
        }

        // Remove only the zero-angle Fy residual. Multiplying Fy by a near-zero
        // gate would erase the physical cornering stiffness dFy/dSA at SA=0.
        if (needs_sa_zero) {
            if (runtime_diagnostics_enabled_) {
                ++runtime_diagnostics_.sa_zero_forwards;
                ++runtime_diagnostics_.sa_zero_projections;
            }
            std::array<double, 5> zero_fy_input = physical_input;
            zero_fy_input[1] = 0.0;
            zero_fy_input[3] = 0.0;
            if (has_input_bounds_) {
                zero_fy_input[1] = std::clamp(zero_fy_input[1], input_min_[1], input_max_[1]);
                zero_fy_input[3] = std::clamp(zero_fy_input[3], input_min_[3], input_max_[3]);
            }

            std::array<double, 4> zero_fy_output = forward_physical(zero_fy_input, include_mx);
            apply_zero_slip_projection(zero_fy_zero_gate, zero_fy_output);

            if (use_symmetry) {
                const double zero_fy_symmetry_metric =
                    std::pow(zero_fy_input[0] / symmetry_sr_scale_, 2) +
                    std::pow(zero_fy_input[3] / symmetry_ia_scale_deg_, 2);
                const double zero_fy_symmetry_gate = std::exp(-zero_fy_symmetry_metric);
                if (zero_fy_symmetry_gate > kPostprocessGateCutoff) {
                    zero_fy_output[1] += zero_fy_symmetry_gate * (0.0 - zero_fy_output[1]);
                    zero_fy_output[3] += zero_fy_symmetry_gate * (0.0 - zero_fy_output[3]);
                }
            }
            output[1] -= residual_gate * zero_fy_output[1];
        }

        if (friction_cap_mu_ > 0.0) {
            const double force_mag =
                std::sqrt(output[0] * output[0] + output[1] * output[1] + postprocess_epsilon_);
            const double limit = friction_cap_mu_ * std::abs(physical_input[2]);
            if (force_mag > limit && limit >= 0.0) {
                if (runtime_diagnostics_enabled_) {
                    ++runtime_diagnostics_.friction_cap_projections;
                }
                const double scale = limit / (force_mag + postprocess_epsilon_);
                output[0] *= scale;
                output[1] *= scale;
            }
        }
    }

    std::vector<double> forward_legacy(const std::vector<double>& input) const {
        std::vector<double> x = input;
        for (size_t i = 0; i < fc_layers_.size(); ++i) {
            x = fc_layers_[i].forward(x);
            if (i < bn_layers_.size() && !bn_layers_[i].gamma.empty()) {
                x = bn_layers_[i].forward(x);
            }
            if (i < fc_layers_.size() - 1) {
                for (double& value : x) {
                    value = std::max(0.0, value);
                }
            }
        }
        return x;
    }

    static RuntimeLayer parse_runtime_layer(const nlohmann::json& layer_json) {
        RuntimeLayer layer;
        const std::string type = layer_json["type"].get<std::string>();

        if (type == "linear") {
            layer.type = RuntimeLayer::Type::Linear;
            layer.weights =
                layer_json["weight"].get<std::vector<std::vector<double>>>();
            layer.n_out = layer.weights.size();
            layer.n_in = layer.n_out > 0 ? layer.weights.front().size() : 0;
            layer.weights_flat = flatten_weights(layer.weights);
            if (!layer_json["bias"].is_null()) {
                layer.bias = layer_json["bias"].get<std::vector<double>>();
            }
        } else if (type == "layer_norm") {
            layer.type = RuntimeLayer::Type::LayerNorm;
            layer.weights = {
                layer_json["weight"].get<std::vector<double>>(),
            };
            layer.bias = layer_json["bias"].get<std::vector<double>>();
            layer.epsilon = layer_json.value("eps", 1e-5);
        } else if (type == "swish") {
            layer.type = RuntimeLayer::Type::Swish;
        } else if (type == "relu") {
            layer.type = RuntimeLayer::Type::ReLU;
        } else {
            throw std::runtime_error("NNTextTireModel: unsupported runtime layer type: " + type);
        }

        return layer;
    }

    static std::vector<RuntimeLayer> parse_runtime_layers(const nlohmann::json& layers_json) {
        std::vector<RuntimeLayer> layers;
        for (const auto& layer_json : layers_json) {
            layers.push_back(parse_runtime_layer(layer_json));
        }
        return layers;
    }

    void load_runtime_json(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open runtime model file: " + filepath);
        }

        nlohmann::json j;
        file >> j;

        if (!j.contains("input") || !j.contains("output") ||
            (!j.contains("layers") && !j.contains("heads"))) {
            throw std::runtime_error("NNTextTireModel: invalid runtime JSON structure");
        }

        for (size_t i = 0; i < 5; ++i) {
            input_mean_[i] = j["input"]["mean"][i].get<double>();
            input_std_[i] = j["input"]["std"][i].get<double>();
        }
        for (size_t i = 0; i < 4; ++i) {
            output_mean_[i] = j["output"]["mean"][i].get<double>();
            output_std_[i] = j["output"]["std"][i].get<double>();
        }
        has_input_bounds_ = j.contains("input_bounds");
        if (has_input_bounds_) {
            for (size_t i = 0; i < 5; ++i) {
                input_min_[i] = j["input_bounds"]["min"][i].get<double>();
                input_max_[i] = j["input_bounds"]["max"][i].get<double>();
            }
        }

        use_runtime_postprocess_ = false;
        friction_cap_mu_ = 0.0;
        zero_slip_sr_scale_ = 0.0;
        zero_slip_sa_scale_deg_ = 0.0;
        symmetry_sr_scale_ = 0.0;
        symmetry_ia_scale_deg_ = 0.0;
        sa_zero_fy_scale_deg_ = 0.0;
        postprocess_epsilon_ = 1e-9;
        if (j.contains("postprocess")) {
            const auto& postprocess = j["postprocess"];
            use_runtime_postprocess_ = postprocess.value("enabled", true);
            friction_cap_mu_ = postprocess.value("friction_cap_mu", 0.0);
            zero_slip_sr_scale_ = postprocess.value("zero_slip_sr_scale", 0.0);
            zero_slip_sa_scale_deg_ = postprocess.value("zero_slip_sa_scale_deg", 0.0);
            symmetry_sr_scale_ = postprocess.value("symmetry_sr_scale", 0.0);
            symmetry_ia_scale_deg_ = postprocess.value("symmetry_ia_scale_deg", 0.0);
            sa_zero_fy_scale_deg_ = postprocess.value("sa_zero_fy_scale_deg", 0.0);
            postprocess_epsilon_ = postprocess.value("epsilon", 1e-9);
        }

        runtime_layers_.clear();
        runtime_trunk_layers_.clear();
        runtime_heads_.clear();
        runtime_is_multi_head_ = false;
        fc_layers_.clear();
        bn_layers_.clear();

        if (j.contains("heads")) {
            runtime_is_multi_head_ = true;
            runtime_trunk_layers_ = j.contains("trunk_layers")
                ? parse_runtime_layers(j["trunk_layers"])
                : std::vector<RuntimeLayer>();
            for (const auto& head_json : j["heads"]) {
                RuntimeHead head;
                head.name = head_json.value("name", "");
                head.output_indices = head_json["output_indices"].get<std::vector<size_t>>();
                head.layers = parse_runtime_layers(head_json["layers"]);
                runtime_heads_.push_back(std::move(head));
            }
        } else {
            runtime_layers_ = parse_runtime_layers(j["layers"]);
        }
    }

    void load_normalization(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open normalization file: " + filepath);
        }

        std::string line;
        std::string section;
        size_t value_index = 0;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }

            if (line.find("INPUT_SIZE:") == 0 || line.find("OUTPUT_SIZE:") == 0) {
                continue;
            }
            if (line.find("INPUT_MEAN:") == 0) {
                section = "input_mean";
                value_index = 0;
                continue;
            }
            if (line.find("INPUT_STD:") == 0) {
                section = "input_std";
                value_index = 0;
                continue;
            }
            if (line.find("OUTPUT_MEAN:") == 0) {
                section = "output_mean";
                value_index = 0;
                continue;
            }
            if (line.find("OUTPUT_STD:") == 0) {
                section = "output_std";
                value_index = 0;
                continue;
            }

            std::istringstream values(line);
            double value = 0.0;
            while (values >> value) {
                if (section == "input_mean" && value_index < input_mean_.size()) {
                    input_mean_[value_index++] = value;
                } else if (section == "input_std" && value_index < input_std_.size()) {
                    input_std_[value_index++] = value;
                } else if (section == "output_mean" && value_index < output_mean_.size()) {
                    output_mean_[value_index++] = value;
                } else if (section == "output_std" && value_index < output_std_.size()) {
                    output_std_[value_index++] = value;
                }
            }
        }
    }

    void load_weights(const std::string& filepath) {
        std::ifstream file(filepath);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open weights file: " + filepath);
        }

        fc_layers_.clear();
        std::string line;

        while (std::getline(file, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }

            if (line.find("LAYER_") == 0 && line.find("_WEIGHTS:") != std::string::npos) {
                FCLayer layer;

                std::istringstream iss(line.substr(line.find(':') + 1));
                iss >> layer.n_out >> layer.n_in;

                layer.weights.resize(layer.n_out, std::vector<double>(layer.n_in, 0.0));
                for (int i = 0; i < layer.n_out; ++i) {
                    std::getline(file, line);
                    std::istringstream row_stream(line);
                    for (int j = 0; j < layer.n_in; ++j) {
                        row_stream >> layer.weights[i][j];
                    }
                }

                while (std::getline(file, line)) {
                    if (line.find("LAYER_") != std::string::npos &&
                        line.find("_BIASES:") != std::string::npos) {
                        break;
                    }
                }

                layer.biases.resize(layer.n_out);
                for (int i = 0; i < layer.n_out; ++i) {
                    std::getline(file, line);
                    layer.biases[i] = std::stod(line);
                }

                layer.weights_flat = flatten_weights(layer.weights);
                fc_layers_.push_back(std::move(layer));
            }
        }
    }

    void load_architecture(const std::string& filepath) {
        (void)filepath;
        bn_layers_.clear();
    }
};

} // namespace fsae::physics::tire
