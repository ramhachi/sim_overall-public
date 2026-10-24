#pragma once
/// @file magic_formula.hpp
/// @brief Pacejka Magic Formula (MF 5.2) タイヤモデル
///
/// MATLABの magic_formula_fitting.m から移植。
/// 4段階フィッティング結果をテキストファイルから読み込み、
/// 純スリップ・複合スリップ・セルフアライニングトルクを計算する。
///
/// 符号規約: TTC/SAE convention
///   - Fz: 負 (下向き正)
///   - α: SAE convention (rad)
///   - κ: slip ratio (-)

#include <fsae/physics/tire/nn_lookup_tire.hpp>  // TireForces
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace fsae::physics::tire {

/// MF 5.2 パラメータ (52 params total)
struct MagicFormulaParams {
    double Fz0 = -890.0;   // 基準荷重 [N] (TTC convention, 負)
    double R0 = 0.2286;    // 無負荷半径 [m]

    // Stage 1: 純横力 Fy0 (15 params)
    double pCy1 = 1.3, pDy1 = 2.0, pDy2 = -0.1, pDy3 = 0.0;
    double pKy1 = -15.0, pKy2 = 2.0, pKy4 = 2.0;
    double pEy1 = -0.5, pEy2 = 0.0, pEy3 = 0.0, pEy4 = 0.0;
    double pHy1 = 0.0, pHy2 = 0.0, pVy1 = 0.0, pVy2 = 0.0;

    // Stage 2: 純縦力 Fx0 (15 params)
    double pCx1 = 1.6, pDx1 = 1.5, pDx2 = -0.1, pDx3 = 0.0;
    double pKx1 = 20.0, pKx2 = 0.0, pKx3 = 0.0;
    double pEx1 = -0.3, pEx2 = 0.0, pEx3 = 0.0, pEx4 = 0.0;
    double pHx1 = 0.0, pHx2 = 0.0, pVx1 = 0.0, pVx2 = 0.0;

    // Stage 3: 複合スリップ Gxα (5), Gyκ (5)
    double rBx1 = 12.0, rBx2 = 8.0, rCx1 = 1.0, rHx1 = 0.0, rEx1 = 0.0;
    double rBy1 = 8.0, rBy2 = 5.0, rBy3 = 0.0, rCy1 = 1.0, rHy1 = 0.0;

    // Stage 4: セルフアライニングトルク Mz (12 params)
    double qBz1 = 10.0, qBz2 = 0.0, qCz1 = 1.3, qDz1 = 0.13, qDz2 = 0.0, qEz1 = -1.0;
    double qHz1 = 0.0, qHz2 = 0.0, qBz9 = 8.0, qDz6 = -0.03, qDz8 = -0.01, qDz9 = 0.0;
};

/// 純横力 Fy0 の計算
/// @param p MFパラメータ
/// @param alpha スリップ角 [rad]
/// @param Fz 垂直荷重 [N] (TTC convention, 負)
/// @param gamma キャンバ角 [rad]
/// @return 横力 Fy0 [N]
inline double compute_mf_Fy0(const MagicFormulaParams& p, double alpha, double Fz, double gamma) {
    double dfz = (Fz - p.Fz0) / p.Fz0;

    // 水平・垂直シフト
    double SHy = p.pHy1 + p.pHy2 * dfz;
    double SVy = Fz * (p.pVy1 + p.pVy2 * dfz);

    // シフト入力
    double ay = alpha + SHy;

    // 形状係数
    double Cy = p.pCy1;

    // ピーク係数
    double muy = (p.pDy1 + p.pDy2 * dfz) * (1.0 - p.pDy3 * gamma * gamma);
    double Dy = muy * Fz;

    // コーナリング剛性
    double Kya = p.pKy1 * p.Fz0 * std::sin(p.pKy4 * std::atan(Fz / (p.pKy2 * p.Fz0 + 1e-6)));

    // 剛性係数
    double By = Kya / (Cy * Dy + std::copysign(1e-6, Dy));

    // 曲率係数
    double Ey = (p.pEy1 + p.pEy2 * dfz) * (1.0 - (p.pEy3 + p.pEy4 * gamma) * std::copysign(1.0, ay));
    Ey = std::min(Ey, 1.0);

    // Magic Formula
    double By_ay = By * ay;
    return Dy * std::sin(Cy * std::atan(By_ay - Ey * (By_ay - std::atan(By_ay)))) + SVy;
}

/// 純縦力 Fx0 の計算
/// @param p MFパラメータ
/// @param kappa スリップ率 [-]
/// @param Fz 垂直荷重 [N] (TTC convention, 負)
/// @param gamma キャンバ角 [rad]
/// @return 縦力 Fx0 [N]
inline double compute_mf_Fx0(const MagicFormulaParams& p, double kappa, double Fz, double gamma) {
    double dfz = (Fz - p.Fz0) / p.Fz0;

    // 水平・垂直シフト
    double SHx = p.pHx1 + p.pHx2 * dfz;
    double SVx = Fz * (p.pVx1 + p.pVx2 * dfz);

    // シフト入力
    double kx = kappa + SHx;

    // 形状係数
    double Cx = p.pCx1;

    // ピーク係数
    double mux = (p.pDx1 + p.pDx2 * dfz) * (1.0 - p.pDx3 * gamma * gamma);
    double Dx = mux * Fz;

    // 縦剛性
    double Kxk = Fz * (p.pKx1 + p.pKx2 * dfz) * std::exp(p.pKx3 * dfz);

    // 剛性係数
    double Bx = Kxk / (Cx * Dx + std::copysign(1e-6, Dx));

    // 曲率係数
    double Ex = (p.pEx1 + p.pEx2 * dfz + p.pEx3 * dfz * dfz) * (1.0 - p.pEx4 * std::copysign(1.0, kx));
    Ex = std::min(Ex, 1.0);

    // Magic Formula
    double Bx_kx = Bx * kx;
    return Dx * std::sin(Cx * std::atan(Bx_kx - Ex * (Bx_kx - std::atan(Bx_kx)))) + SVx;
}

/// 複合スリップ Fx 低減係数 Gxα
/// @param p MFパラメータ
/// @param alpha スリップ角 [rad]
/// @param kappa スリップ率 [-]
/// @return Gxα ∈ [0, ∞) (通常 [0, 1])
inline double compute_mf_Gxa(const MagicFormulaParams& p, double alpha, double kappa) {
    double Bxa = p.rBx1 * std::cos(std::atan(p.rBx2 * kappa));
    double Cxa = p.rCx1;
    double as = alpha + p.rHx1;
    double Exa = p.rEx1;

    // Gxa (正規化: α=0 で Gxa=1)
    double Bxa_as = Bxa * as;
    double Gxa_num = std::cos(Cxa * std::atan(Bxa_as - Exa * (Bxa_as - std::atan(Bxa_as))));
    double Bxa_sh = Bxa * p.rHx1;
    double Gxa_den = std::cos(Cxa * std::atan(Bxa_sh - Exa * (Bxa_sh - std::atan(Bxa_sh))));

    double Gxa = Gxa_num / (Gxa_den + 1e-10);
    return std::max(Gxa, 0.0);
}

/// 複合スリップ Fy 低減係数 Gyκ
/// @param p MFパラメータ
/// @param alpha スリップ角 [rad]
/// @param kappa スリップ率 [-]
/// @return Gyκ ∈ [0, ∞) (通常 [0, 1])
inline double compute_mf_Gyk(const MagicFormulaParams& p, double alpha, double kappa) {
    double Byk = p.rBy1 * std::cos(std::atan(p.rBy2 * (alpha - p.rBy3)));
    double Cyk = p.rCy1;
    double ks = kappa + p.rHy1;

    // Gyk (正規化: κ=0 で Gyk=1)
    double Gyk_num = std::cos(Cyk * std::atan(Byk * ks));
    double Gyk_den = std::cos(Cyk * std::atan(Byk * p.rHy1));

    double Gyk = Gyk_num / (Gyk_den + 1e-10);
    return std::max(Gyk, 0.0);
}

/// コーナリング剛性 Kyα の計算 (Mz計算の前処理用)
inline double compute_mf_Kya(const MagicFormulaParams& p, double Fz) {
    return p.pKy1 * p.Fz0 * std::sin(p.pKy4 * std::atan(Fz / (p.pKy2 * p.Fz0 + 1e-6)));
}

/// セルフアライニングトルク Mz の計算
/// @param p MFパラメータ
/// @param alpha スリップ角 [rad]
/// @param Fz 垂直荷重 [N]
/// @param gamma キャンバ角 [rad]
/// @param Fy0 純横力 [N] (事前計算済み)
/// @param Kya コーナリング剛性 [N/rad]
/// @param SHy 横力の水平シフト
/// @param SVy 横力の垂直シフト [N]
/// @return Mz [Nm]
inline double compute_mf_Mz(const MagicFormulaParams& p, double alpha, double Fz,
                            [[maybe_unused]] double gamma, double Fy0, double Kya,
                            double SHy, double SVy) {
    double dfz = (Fz - p.Fz0) / p.Fz0;

    // --- ニューマチックトレイル t ---
    double SHt = p.qHz1 + p.qHz2 * dfz;
    double at = alpha + SHt;

    double Bt = p.qBz1 + p.qBz2 * dfz;
    double Ct = p.qCz1;
    double Dt = (Fz / p.Fz0) * p.R0 * (p.qDz1 + p.qDz2 * dfz);
    double Et = p.qEz1 * (1.0 - (2.0 / M_PI) * std::copysign(1.0, at) * std::atan(Bt * Ct * at));
    Et = std::min(Et, 1.0);

    double Bt_at = Bt * at;
    double trail = Dt * std::cos(Ct * std::atan(Bt_at - Et * (Bt_at - std::atan(Bt_at)))) * std::cos(alpha);

    // --- 残留トルク Mzr ---
    double Dr = Fz * p.R0 * (p.qDz6 + p.qDz8 + p.qDz9 * dfz);
    double Br = p.qBz9;
    double ar = alpha + SHy + SVy / (Kya + std::copysign(1e-6, Kya));

    double Mzr = Dr * std::cos(std::atan(Br * ar));

    // --- 合計 ---
    return -trail * Fy0 + Mzr;
}

/// MF 5.2 トップレベル計算関数 (複合スリップ対応)
/// @param params MFパラメータ
/// @param kappa スリップ率 [-]
/// @param alpha_rad スリップ角 [rad]
/// @param Fz 垂直荷重 [N] (TTC convention, 負)
/// @param gamma_rad キャンバ角 [rad]
/// @return TireForces (Fx, Fy, Mx, Mz)
inline TireForces compute_magic_formula(
    const MagicFormulaParams& params,
    double kappa,
    double alpha_rad,
    double Fz,
    double gamma_rad)
{
    // 純スリップ力
    double Fy0 = compute_mf_Fy0(params, alpha_rad, Fz, gamma_rad);
    double Fx0 = compute_mf_Fx0(params, kappa, Fz, gamma_rad);

    // 複合スリップ低減係数
    double Gxa = compute_mf_Gxa(params, alpha_rad, kappa);
    double Gyk = compute_mf_Gyk(params, alpha_rad, kappa);

    double Fx = Gxa * Fx0;
    double Fy = Gyk * Fy0;

    // Mz 計算の前処理
    double dfz = (Fz - params.Fz0) / params.Fz0;
    double Kya = compute_mf_Kya(params, Fz);
    double SHy = params.pHy1 + params.pHy2 * dfz;
    double SVy = Fz * (params.pVy1 + params.pVy2 * dfz);

    double Mz = compute_mf_Mz(params, alpha_rad, Fz, gamma_rad, Fy0, Kya, SHy, SVy);

    TireForces forces;
    forces.Fx = Fx;
    forces.Fy = Fy;
    forces.Mx = 0.0;  // MF 5.2 では Mx は未モデル化
    forces.Mz = Mz;
    return forces;
}

/// Convert raw TTC Magic Formula output to the 14DOF wheel-frame convention.
///
/// Raw Magic Formula longitudinal force follows the TTC sign used by the fitting
/// pipeline. The vehicle model and TMeasy use positive Fx as driving force along
/// the wheel x axis, so Fx is negated at the interface boundary.
inline TireForces magic_formula_to_vehicle_convention(const TireForces& raw_forces) {
    TireForces vehicle_forces = raw_forces;
    vehicle_forces.Fx = -raw_forces.Fx;
    return vehicle_forces;
}

/// テキストファイルからMFパラメータを読み込む
/// @param filepath パラメータファイルパス
/// @return MagicFormulaParams
inline MagicFormulaParams load_magic_formula_params(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open MF params file: " + filepath);
    }

    MagicFormulaParams p;
    std::string line;

    while (std::getline(file, line)) {
        // コメント・空行スキップ
        if (line.empty() || line[0] == '#' || line[0] == '[') {
            continue;
        }

        auto colon = line.find(':');
        if (colon == std::string::npos) continue;

        std::string key = line.substr(0, colon);
        double val = std::stod(line.substr(colon + 1));

        // 基本パラメータ
        if (key == "Fz0") p.Fz0 = val;
        else if (key == "R0") p.R0 = val;
        // Fy0
        else if (key == "pCy1") p.pCy1 = val;
        else if (key == "pDy1") p.pDy1 = val;
        else if (key == "pDy2") p.pDy2 = val;
        else if (key == "pDy3") p.pDy3 = val;
        else if (key == "pKy1") p.pKy1 = val;
        else if (key == "pKy2") p.pKy2 = val;
        else if (key == "pKy4") p.pKy4 = val;
        else if (key == "pEy1") p.pEy1 = val;
        else if (key == "pEy2") p.pEy2 = val;
        else if (key == "pEy3") p.pEy3 = val;
        else if (key == "pEy4") p.pEy4 = val;
        else if (key == "pHy1") p.pHy1 = val;
        else if (key == "pHy2") p.pHy2 = val;
        else if (key == "pVy1") p.pVy1 = val;
        else if (key == "pVy2") p.pVy2 = val;
        // Fx0
        else if (key == "pCx1") p.pCx1 = val;
        else if (key == "pDx1") p.pDx1 = val;
        else if (key == "pDx2") p.pDx2 = val;
        else if (key == "pDx3") p.pDx3 = val;
        else if (key == "pKx1") p.pKx1 = val;
        else if (key == "pKx2") p.pKx2 = val;
        else if (key == "pKx3") p.pKx3 = val;
        else if (key == "pEx1") p.pEx1 = val;
        else if (key == "pEx2") p.pEx2 = val;
        else if (key == "pEx3") p.pEx3 = val;
        else if (key == "pEx4") p.pEx4 = val;
        else if (key == "pHx1") p.pHx1 = val;
        else if (key == "pHx2") p.pHx2 = val;
        else if (key == "pVx1") p.pVx1 = val;
        else if (key == "pVx2") p.pVx2 = val;
        // Gxa
        else if (key == "rBx1") p.rBx1 = val;
        else if (key == "rBx2") p.rBx2 = val;
        else if (key == "rCx1") p.rCx1 = val;
        else if (key == "rHx1") p.rHx1 = val;
        else if (key == "rEx1") p.rEx1 = val;
        // Gyk
        else if (key == "rBy1") p.rBy1 = val;
        else if (key == "rBy2") p.rBy2 = val;
        else if (key == "rBy3") p.rBy3 = val;
        else if (key == "rCy1") p.rCy1 = val;
        else if (key == "rHy1") p.rHy1 = val;
        // Mz
        else if (key == "qBz1") p.qBz1 = val;
        else if (key == "qBz2") p.qBz2 = val;
        else if (key == "qCz1") p.qCz1 = val;
        else if (key == "qDz1") p.qDz1 = val;
        else if (key == "qDz2") p.qDz2 = val;
        else if (key == "qEz1") p.qEz1 = val;
        else if (key == "qHz1") p.qHz1 = val;
        else if (key == "qHz2") p.qHz2 = val;
        else if (key == "qBz9") p.qBz9 = val;
        else if (key == "qDz6") p.qDz6 = val;
        else if (key == "qDz8") p.qDz8 = val;
        else if (key == "qDz9") p.qDz9 = val;
    }

    return p;
}

} // namespace fsae::physics::tire
