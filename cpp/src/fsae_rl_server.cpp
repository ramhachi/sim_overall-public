/// @file fsae_rl_server.cpp
/// @brief RL学習用TCPソケットサーバー
///
/// MATLABのRL Toolboxからの接続を受け付け、14DOFシミュレータを
/// ステップ実行するサーバー。バイナリプロトコルで高速通信。

#include <fsae/driver/rl_control_safety.hpp>
#include <fsae/driver/gear_selection.hpp>
#include <fsae/vehicle/vehicle_14dof_new.hpp>
#include <fsae/vehicle/vehicle_params.hpp>
#include <fsae/track/track_model.hpp>
#include <fsae/track/frenet.hpp>
#include <fsae/track/track_distance.hpp>
#include <fsae/track/waypoint_loader.hpp>
#include <fsae/laptime/velocity_profile.hpp>
#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <array>

// POSIX socket
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <signal.h>

using namespace fsae;
using json = nlohmann::json;

// ========== プロトコル定義 ==========
// コマンドID (uint8_t)
static constexpr uint8_t CMD_INIT  = 0x01;
static constexpr uint8_t CMD_STEP  = 0x02;
static constexpr uint8_t CMD_RESET = 0x03;
static constexpr uint8_t CMD_PARAM = 0x04;
static constexpr uint8_t CMD_QUIT  = 0xFF;

// レスポンスID (uint8_t)
static constexpr uint8_t RSP_OK    = 0x01;
static constexpr uint8_t RSP_OBS   = 0x02;
static constexpr uint8_t RSP_ERR   = 0xFF;

static constexpr int OBS_DIM = 34;
static constexpr int INFO_DIM = 12;  // 追加情報
static constexpr int N_SUBSTEPS = 20; // 20ms / 1ms = 20ステップ
static constexpr double SIM_DT = 0.001;
static constexpr double CONTROL_DT = 0.020; // 50 Hz

// ========== グローバル状態 ==========
static volatile bool g_running = true;

void signal_handler(int) {
    g_running = false;
}

// ========== ソケットI/Oヘルパー ==========
bool send_all(int fd, const void* data, size_t len) {
    const char* ptr = static_cast<const char*>(data);
    while (len > 0) {
        ssize_t n = send(fd, ptr, len, 0);
        if (n <= 0) return false;
        ptr += n;
        len -= n;
    }
    return true;
}

bool recv_all(int fd, void* data, size_t len) {
    char* ptr = static_cast<char*>(data);
    while (len > 0) {
        ssize_t n = recv(fd, ptr, len, 0);
        if (n <= 0) return false;
        ptr += n;
        len -= n;
    }
    return true;
}

bool send_double_array(int fd, const double* data, int count) {
    return send_all(fd, data, count * sizeof(double));
}

bool recv_double_array(int fd, double* data, int count) {
    return recv_all(fd, data, count * sizeof(double));
}

// 文字列送信: [uint32_t len][char data...]
bool send_string(int fd, const std::string& s) {
    uint32_t len = static_cast<uint32_t>(s.size());
    if (!send_all(fd, &len, 4)) return false;
    if (len > 0 && !send_all(fd, s.data(), len)) return false;
    return true;
}

// 文字列受信
bool recv_string(int fd, std::string& s) {
    uint32_t len = 0;
    if (!recv_all(fd, &len, 4)) return false;
    if (len > 100000) return false; // sanity check
    s.resize(len);
    if (len > 0 && !recv_all(fd, s.data(), len)) return false;
    return true;
}

// ========== RLサーバークラス ==========
class RLServer {
public:
    RLServer() = default;

    // トラック生成
    track::CurvatureBasedTrack create_track(const std::string& name, double radius = 30.0, double width = 3.0) {
        // Parametric circle: "circle_R<num>" or "circle_cw_R<num>"
        if (name.substr(0, 10) == "circle_cw_" && name.size() > 11 && name[10] == 'R') {
            double r = std::stod(name.substr(11));
            return track::create_circle_cw_track(r, width);
        }
        if (name.substr(0, 8) == "circle_R" && name.size() > 8) {
            double r = std::stod(name.substr(8));
            return track::create_circle_track(r, width);
        }
        
        if (name == "circle") return track::create_circle_track(radius, width);
        if (name == "circle_cw") return track::create_circle_cw_track(radius, width);
        if (name == "figure_eight") return track::create_figure_eight_track(radius, width);
        if (name == "skidpad") return track::create_skidpad_track(radius, width);
        if (name == "straight_corner") return track::create_straight_corner_track(50.0, radius, width);
        if (name == "acceleration") return track::create_acceleration_track(75.0, width);
        if (name == "straight") return track::create_acceleration_track(200.0, width);

        // 名前付きオートクロスコース
        if (name == "ASE_autocross" || name == "autocross") {
            try {
                // Docker 内: /workspace/data/、ローカル: 実行ディレクトリ相対 or 環境変数
                std::string csv_path;
                const char* env_data = std::getenv("FSAE_DATA_DIR");
                if (env_data) {
                    csv_path = std::string(env_data) + "/ASE_autocross_sim.csv";
                } else if (std::ifstream("/workspace/data/ASE_autocross_sim.csv").good()) {
                    csv_path = "/workspace/data/ASE_autocross_sim.csv";
                } else {
                    csv_path = "data/ASE_autocross_sim.csv";  // ローカル相対パス
                }
                auto waypoints = track::WaypointTrackBuilder::load_from_csv(csv_path, width);
                return track::WaypointTrackBuilder(waypoints, width).build();
            } catch (const std::exception& e) {
                std::cerr << "Failed to load autocross: " << e.what() << ", falling back to circle" << std::endl;
                return track::create_circle_track(radius, width);
            }
        }

        // CSVファイル
        if (name.size() > 4 && name.substr(name.size()-4) == ".csv") {
            try {
                auto waypoints = track::WaypointTrackBuilder::load_from_csv(name, width);
                return track::WaypointTrackBuilder(waypoints, width).build();
            } catch (...) {
                std::cerr << "Failed to load CSV track: " << name << ", falling back to circle" << std::endl;
            }
        }

        std::cerr << "Unknown track: " << name << ", using circle" << std::endl;
        return track::create_circle_track(radius, width);
    }

    // 初期化
    bool init(const std::string& track_name, const std::string& params_json = "") {
        track_name_ = track_name;

        // パラメータ読み込み
        if (!params_json.empty()) {
            try {
                params_ = vehicle::VehicleParams::from_json_string(params_json);
            } catch (const std::exception& e) {
                std::cerr << "Failed to parse params: " << e.what() << std::endl;
                params_ = vehicle::VehicleParams();
            }
        } else {
            params_ = vehicle::VehicleParams();
        }

        // トラック生成
        track_ = std::make_unique<track::CurvatureBasedTrack>(create_track(track_name));
        frenet_ = std::make_unique<track::FrenetConverter>(*track_);

        // 車両生成
        vehicle_ = std::make_unique<vehicle::Vehicle14DOF_New>(params_);
        steering_limit_ = driver::rl_steering_command_limit(params_);
        reset_vehicle();

        std::cout << "Initialized: track=" << track_name
                  << ", length=" << track_->length() << "m" << std::endl;
        return true;
    }

    const std::string& current_track_name() const {
        return track_name_;
    }

    // リセット
    void reset_vehicle() {
        vehicle_->reset();
        vehicle_->set_initial_velocity(initial_speed_);
        frenet_->reset();

        auto start_point = track_->get_point(0.0);
        // Vehicle14DOF uses the SAE/NED-style convention used by the PID
        // driver: vehicle X=map Y, vehicle Y=map X, yaw positive clockwise.
        // Keep the RL server in the same state frame so observations and
        // laptime deployment share one contract.
        vehicle_->state().x = start_point.y;
        vehicle_->state().y = start_point.x;
        vehicle_->state().psi = M_PI / 2.0 - start_point.theta;

        step_count_ = 0;
        prev_s_ = 0.0;
        prev_action_ = {0.0, 0.0, 0.0};
        lap_completed_ = false;
    }

    // ステップ実行: 制御入力を受けてN_SUBSTEPSの物理シミュレーション実行
    // steer は angle-mode ではタイヤ舵角、torque-mode ではドライバートルク
    void step(double steer, double throttle, double brake) {
        vehicle::DriverInput input;
        input.steering = driver::smooth_rl_steering_command(
            steer,
            prev_action_[0],
            steering_limit_,
            params_.steering.torque_mode,
            CONTROL_DT,
            params_.steering.T_driver_rate_max);
        input.throttle = std::clamp(throttle, 0.0, 1.0);
        input.brake = std::clamp(brake, 0.0, 1.0);
        input.gear = select_gear(vehicle_->state().vx);

        // 安定性制御: 高ヨーレートで自動的にスロットルカット＋ブレーキ
        // RWDパワーオーバーステアを防止し、RLの探索空間を安全にする
        const double r_abs = std::abs(vehicle_->state().r);
        if (r_abs > 1.5) {
            double fade = std::clamp((r_abs - 1.5) / 1.0, 0.0, 1.0); // 1.5~2.5で線形フェード
            input.throttle *= (1.0 - fade);
            input.brake = std::max(input.brake, fade * 0.3);
        }

        // Save base inputs from agent & stability control to prevent accumulation over sub-steps
        const double base_steering = input.steering;
        const double base_throttle = input.throttle;
        const double base_brake = input.brake;

        for (int i = 0; i < N_SUBSTEPS; ++i) {
            input.steering = base_steering;
            input.throttle = base_throttle;
            input.brake = base_brake;
            if (!driver::rl_driver_speed_limits_disabled()) {
                driver::apply_rl_curvature_speed_limit(vehicle_->state(), *track_, *frenet_, input, params_);
            }
            vehicle_->step(SIM_DT, input);
        }
        step_count_++;
        prev_action_ = {input.steering, input.throttle, input.brake};
    }

    // 34次元の観測ベクトルを構築
    void build_observation(double obs[OBS_DIM]) {
        const auto& st = vehicle_->state();
        const double map_x = st.y;
        const double map_y = st.x;
        const double map_theta = M_PI / 2.0 - st.psi;
        auto frenet = frenet_->global_to_frenet(map_x, map_y, map_theta);
        auto tp = track_->get_point(frenet.s);

        // ======== 車両状態（15次元） ========
        obs[0] = st.vx;
        obs[1] = st.vy;
        obs[2] = st.r;  // ヨーレート

        // 加速度（車体フレーム: 近似値）
        // ax_body ≈ -vy * r (コリオリ項、縦加速度の主成分)
        // ay_body ≈  vx * r (遠心力項、横加速度の主成分)
        obs[3] = -st.vy * st.r;  // 縦加速度近似
        obs[4] = st.vx * st.r;   // 横加速度近似

        obs[5] = st.phi;  // ロール角
        obs[6] = prev_action_[0]; // 現在ステアリング角
        obs[7] = prev_action_[1]; // 現在スロットル
        obs[8] = prev_action_[2]; // 現在ブレーキ

        // 各輪スリップ比
        double r_tire = params_.tire.radius;
        for (int i = 0; i < 4; ++i) {
            double vw = st.omega[i] * r_tire;
            double vref = std::max(std::abs(st.vx), 0.5);
            obs[9 + i] = (vw - st.vx) / vref;
        }

        // フロント/リア平均スリップ角
        double alpha_f = 0.0, alpha_r = 0.0;
        if (std::abs(st.vx) > 0.5) {
            alpha_f = std::atan2(st.vy + params_.mass.a * st.r, std::max(st.vx, 0.5));
            alpha_r = std::atan2(st.vy - params_.mass.b * st.r, std::max(st.vx, 0.5));
        }
        obs[13] = alpha_f;
        obs[14] = alpha_r;

        // ======== トラック情報（15次元） ========
        obs[15] = frenet.n;          // 横偏差
        obs[16] = frenet.d_heading;  // ヘディング誤差

        // 現在の曲率
        obs[17] = track_->get_curvature(frenet.s);

        // 先読み曲率（5点）: 速度依存の先読み距離
        double v_ratio = std::clamp(st.vx / 30.0, 0.0, 1.0);
        double lookahead_distances[5] = {10.0, 20.0, 40.0, 60.0, 80.0};
        for (int i = 0; i < 5; ++i) {
            double d = lookahead_distances[i] * (0.3 + 0.7 * v_ratio);
            double s_ahead = frenet.s + d;
            if (s_ahead > track_->length()) s_ahead -= track_->length();
            obs[18 + i] = track_->get_curvature(s_ahead);
        }

        // 左右トラック端距離
        obs[23] = tp.width_left - frenet.n;   // 左端までの距離
        obs[24] = tp.width_right + frenet.n;  // 右端までの距離

        // 進行率
        obs[25] = std::fmod(std::max(frenet.s, 0.0), track_->length()) / track_->length();

        // 先読みトラック幅（4点）
        double width_distances[4] = {10.0, 20.0, 40.0, 60.0};
        for (int i = 0; i < 4; ++i) {
            double s_ahead = frenet.s + width_distances[i];
            if (s_ahead > track_->length()) s_ahead -= track_->length();
            auto tp_ahead = track_->get_point(s_ahead);
            obs[26 + i] = tp_ahead.width_left + tp_ahead.width_right;
        }

        // フレームねじり + ステアリングホイール状態
        obs[30] = st.phi_twist;
        obs[31] = st.dphi_twist;
        obs[32] = st.delta_sw;
        obs[33] = st.ddelta_sw;
    }

    // 追加情報（報酬計算用）
    void build_info(double info[INFO_DIM]) {
        const auto& st = vehicle_->state();
        const double map_x = st.y;
        const double map_y = st.x;
        const double map_theta = M_PI / 2.0 - st.psi;
        auto frenet = frenet_->global_to_frenet(map_x, map_y, map_theta);
        auto tp = track_->get_point(frenet.s);

        info[0] = st.vx;           // 縦速度
        info[1] = st.vy;           // 横速度
        info[2] = st.r;            // ヨーレート
        info[3] = frenet.s;        // 弧長位置
        info[4] = frenet.n;        // 横偏差
        info[5] = frenet.d_heading;// ヘディング誤差
        info[6] = tp.width_left;   // 左トラック幅
        info[7] = tp.width_right;  // 右トラック幅
        info[8] = st.phi;          // ロール角
        info[9] = st.sideslip();   // 車体スリップ角

        // 最大スリップ角（4輪中）
        double max_sa = 0;
        if (std::abs(st.vx) > 0.5) {
            double a_f = std::abs(std::atan2(st.vy + params_.mass.a * st.r, std::max(st.vx, 0.5)));
            double a_r = std::abs(std::atan2(st.vy - params_.mass.b * st.r, std::max(st.vx, 0.5)));
            max_sa = std::max(a_f, a_r);
        }
        info[10] = max_sa;
        info[11] = track_->length(); // トラック全長
    }

    double get_track_length() const { return track_ ? track_->length() : 0.0; }
    double get_prev_s() const { return prev_s_; }
    void set_prev_s(double s) { prev_s_ = s; }
    int select_gear(double speed) const {
        return driver::select_gear_from_vehicle_params(speed, params_);
    }

private:
    std::string track_name_;
    vehicle::VehicleParams params_;
    std::unique_ptr<track::CurvatureBasedTrack> track_;
    std::unique_ptr<track::FrenetConverter> frenet_;
    std::unique_ptr<vehicle::Vehicle14DOF_New> vehicle_;

    int step_count_ = 0;
    double prev_s_ = 0.0;
    std::array<double, 3> prev_action_ = {0.0, 0.0, 0.0};
    bool lap_completed_ = false;
    double initial_speed_ = 5.0; // 初速 [m/s] (安定した探索のため低めに設定)
    double steering_limit_ = 1.0;
};

// ========== クライアント処理 ==========
void handle_client(int client_fd) {
    std::cout << "Client connected." << std::endl;

    RLServer server;

    while (g_running) {
        // コマンドID受信
        uint8_t cmd;
        if (!recv_all(client_fd, &cmd, 1)) {
            std::cout << "Client disconnected." << std::endl;
            break;
        }

        if (cmd == CMD_INIT) {
            // トラック名受信
            std::string track_name;
            if (!recv_string(client_fd, track_name)) break;

            // パラメータJSON受信
            std::string params_json;
            if (!recv_string(client_fd, params_json)) break;

            server.init(track_name, params_json);

            // レスポンス: ACK + track_length
            uint8_t rsp = RSP_OK;
            send_all(client_fd, &rsp, 1);
            double track_len = server.get_track_length();
            send_double_array(client_fd, &track_len, 1);

        } else if (cmd == CMD_STEP) {
            // 行動受信 (3 doubles: steer, throttle, brake)
            double action[3];
            if (!recv_double_array(client_fd, action, 3)) break;

            // ステップ実行
            server.step(action[0], action[1], action[2]);

            // 観測ベクトル構築
            double obs[OBS_DIM];
            server.build_observation(obs);

            // info構築
            double info[INFO_DIM];
            server.build_info(info);

            // prev_s更新
            server.set_prev_s(info[3]);

            // レスポンス: RSP_OBS + obs + info
            // 1つのパケットにまとめて送信してレイテンシ削減
            uint8_t tx_buf[1 + (OBS_DIM + INFO_DIM) * sizeof(double)];
            tx_buf[0] = RSP_OBS;
            std::memcpy(tx_buf + 1, obs, OBS_DIM * sizeof(double));
            std::memcpy(tx_buf + 1 + OBS_DIM * sizeof(double), info, INFO_DIM * sizeof(double));
            send_all(client_fd, tx_buf, sizeof(tx_buf));

        } else if (cmd == CMD_RESET) {
            server.reset_vehicle();

            double obs[OBS_DIM];
            server.build_observation(obs);

            // まとめて送信
            uint8_t tx_buf[1 + OBS_DIM * sizeof(double)];
            tx_buf[0] = RSP_OBS;
            std::memcpy(tx_buf + 1, obs, OBS_DIM * sizeof(double));
            send_all(client_fd, tx_buf, sizeof(tx_buf));

        } else if (cmd == CMD_PARAM) {
            std::string params_json;
            if (!recv_string(client_fd, params_json)) break;

            // 現在のトラック名で再初期化
            server.init(server.current_track_name(), params_json);

            uint8_t rsp = RSP_OK;
            send_all(client_fd, &rsp, 1);

        } else if (cmd == CMD_QUIT) {
            std::cout << "Client requested quit." << std::endl;
            uint8_t rsp = RSP_OK;
            send_all(client_fd, &rsp, 1);
            break;

        } else {
            std::cerr << "Unknown command: 0x" << std::hex << (int)cmd << std::dec << std::endl;
            uint8_t rsp = RSP_ERR;
            send_all(client_fd, &rsp, 1);
        }
    }

    close(client_fd);
}

// ========== メイン ==========
int main(int argc, char** argv) {
    int port = 12345;
    if (argc > 1) {
        port = std::atoi(argv[1]);
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGPIPE, SIG_IGN);  // broken pipe無視

    // ソケット作成
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "socket() failed" << std::endl;
        return 1;
    }

    // SO_REUSEADDR + SO_REUSEPORT（再起動時の即時再バインドを保証）
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEPORT, &opt, sizeof(opt));

    // TCP_NODELAY（低レイテンシ）
    setsockopt(server_fd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

    // バインド
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "bind() failed on port " << port << std::endl;
        close(server_fd);
        return 1;
    }

    if (listen(server_fd, 8) < 0) {
        std::cerr << "listen() failed" << std::endl;
        close(server_fd);
        return 1;
    }

    std::cout << "=== FSAE RL Server ===" << std::endl;
    std::cout << "Listening on port " << port << std::endl;
    std::cout << "Observation dim: " << OBS_DIM << std::endl;
    std::cout << "Info dim: " << INFO_DIM << std::endl;
    std::cout << "Control dt: " << CONTROL_DT * 1000 << " ms" << std::endl;
    std::cout << "Waiting for connection..." << std::endl;

    while (g_running) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);

        if (client_fd < 0) {
            if (g_running) std::cerr << "accept() failed" << std::endl;
            continue;
        }

        // TCP_NODELAY for client
        int nodelay = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        handle_client(client_fd);
        std::cout << "Waiting for next connection..." << std::endl;
    }

    close(server_fd);
    std::cout << "Server shut down." << std::endl;
    return 0;
}
