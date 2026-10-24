/**
 * @file beam_fem_server.cpp
 * @brief TCP server for 3D Timoshenko beam FEM solver
 *
 * Usage:
 *   ./beam_fem_server [port]  (default: 50051)
 *
 * Protocol: JSON over TCP (newline-delimited)
 *
 * Request:
 * {"command": "SOLVE", "nodes": [[x,y,z],...], "elements": [...], "bcs": [...], "loads": [...]}
 * {"command": "SHUTDOWN"}
 *
 * Response:
 * {"success": true, "U": [...], "K_torsion": ..., "mass": ...}
 * {"success": false, "error": "..."}
 */

#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <thread>
#include <mutex>
#include <queue>
#include "torsion_rigidity.hpp"
#include <condition_variable>
#include <atomic>

#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #define CLOSE_SOCKET closesocket
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    #define CLOSE_SOCKET close
#endif

static std::mutex log_mutex;
#define LOG(msg) { std::lock_guard<std::mutex> lock(log_mutex); std::cout << msg << std::endl << std::flush; }
#define ERR(msg) { std::lock_guard<std::mutex> lock(log_mutex); std::cerr << msg << std::endl << std::flush; }

#ifdef _WIN32
using socket_handle_t = SOCKET;
constexpr socket_handle_t INVALID_SOCKET_HANDLE = INVALID_SOCKET;
#else
using socket_handle_t = int;
constexpr socket_handle_t INVALID_SOCKET_HANDLE = -1;
#endif

std::atomic<bool> should_shutdown(false);
std::mutex queue_mutex;
std::condition_variable queue_cv;
std::queue<socket_handle_t> task_queue;

using namespace Eigen;
using json = nlohmann::json;
typedef Triplet<double> T;

const double PI = 3.14159265358979323846;

bool send_all(socket_handle_t socket_fd, const std::string& message) {
    size_t total_sent = 0;
    while (total_sent < message.size()) {
        int bytes_sent = send(
            socket_fd,
            message.data() + total_sent,
            static_cast<int>(message.size() - total_sent),
            0
        );

        if (bytes_sent <= 0) {
            return false;
        }

        total_sent += static_cast<size_t>(bytes_sent);
    }

    return true;
}

// Structure to hold element data
struct BeamElement {
    int node_i, node_j;
    double D_outer, t, E, G, rho;
    int shape;  // 1=round, 2=square
    double joint_factor;
    double A, Iy, Iz, J;  // Section properties
    double L;             // Length
    Matrix3d R;           // Rotation matrix (local to global)
};

// Compute section properties
void computeSectionProperties(BeamElement &elem) {
    double D = elem.D_outer;
    double t_val = elem.t;
    double D_inner = D - 2*t_val;

    if (elem.shape == 1) {  // Round
        elem.A = PI/4 * (D*D - D_inner*D_inner);
        elem.Iy = PI/64 * (pow(D,4) - pow(D_inner,4));
        elem.Iz = elem.Iy;
        elem.J = PI/32 * (pow(D,4) - pow(D_inner,4));
    } else if (elem.shape == 2) {  // Square
        elem.A = D*D - D_inner*D_inner;
        elem.Iy = (pow(D,4) - pow(D_inner,4)) / 12.0;
        elem.Iz = elem.Iy;
        double b = D - t_val;
        elem.J = pow(b,3) * t_val;  // Bredt: 4 A_m^2 t / s, A_m = b^2, s = 4b (sharp corners)
    } else {
        throw std::runtime_error("Shape must be 1 (round) or 2 (square)");
    }
}

// Compute rotation matrix from local to global coordinates
Matrix3d computeRotationMatrix(const Vector3d &p1, const Vector3d &p2) {
    Vector3d x_local = (p2 - p1).normalized();

    Vector3d y_temp;
    if (std::abs(x_local(2)) < 0.9) {
        y_temp = Vector3d(0, 0, 1);
    } else {
        y_temp = Vector3d(0, 1, 0);
    }

    Vector3d z_local = x_local.cross(y_temp).normalized();
    Vector3d y_local = z_local.cross(x_local).normalized();

    Matrix3d R;
    R.col(0) = x_local;
    R.col(1) = y_local;
    R.col(2) = z_local;

    return R;
}

// Compute 12x12 element stiffness matrix in local coordinates
MatrixXd computeLocalStiffness(const BeamElement &elem) {
    double L = elem.L;
    double E = elem.E * 1e9;  // GPa to Pa
    double G = elem.G * 1e9;
    double A = elem.A * 1e-6;   // mm^2 to m^2
    double Iy = elem.Iy * 1e-12; // mm^4 to m^4
    double Iz = elem.Iz * 1e-12;
    double J = elem.J * 1e-12;

    double jf = elem.joint_factor;

    MatrixXd K_local = MatrixXd::Zero(12, 12);

    // Axial (no joint correction)
    double EA_L = E*A/L;
    K_local(0,0) = EA_L;   K_local(0,6) = -EA_L;
    K_local(6,0) = -EA_L;  K_local(6,6) = EA_L;

    // Torsion (with joint correction)
    double GJ_L = jf * G*J/L;
    K_local(3,3) = GJ_L;   K_local(3,9) = -GJ_L;
    K_local(9,3) = -GJ_L;  K_local(9,9) = GJ_L;

    // Bending in y-z plane (with joint correction)
    double EIz_L3 = jf * E*Iz/(L*L*L);
    K_local(1,1) = 12*EIz_L3;  K_local(1,5) = 6*EIz_L3*L;   K_local(1,7) = -12*EIz_L3;  K_local(1,11) = 6*EIz_L3*L;
    K_local(5,1) = 6*EIz_L3*L; K_local(5,5) = 4*EIz_L3*L*L; K_local(5,7) = -6*EIz_L3*L; K_local(5,11) = 2*EIz_L3*L*L;
    K_local(7,1) = -12*EIz_L3; K_local(7,5) = -6*EIz_L3*L;  K_local(7,7) = 12*EIz_L3;   K_local(7,11) = -6*EIz_L3*L;
    K_local(11,1) = 6*EIz_L3*L; K_local(11,5) = 2*EIz_L3*L*L; K_local(11,7) = -6*EIz_L3*L; K_local(11,11) = 4*EIz_L3*L*L;

    // Bending in x-z plane (with joint correction)
    double EIy_L3 = jf * E*Iy/(L*L*L);
    K_local(2,2) = 12*EIy_L3;   K_local(2,4) = -6*EIy_L3*L;  K_local(2,8) = -12*EIy_L3;  K_local(2,10) = -6*EIy_L3*L;
    K_local(4,2) = -6*EIy_L3*L; K_local(4,4) = 4*EIy_L3*L*L; K_local(4,8) = 6*EIy_L3*L;  K_local(4,10) = 2*EIy_L3*L*L;
    K_local(8,2) = -12*EIy_L3;  K_local(8,4) = 6*EIy_L3*L;   K_local(8,8) = 12*EIy_L3;   K_local(8,10) = 6*EIy_L3*L;
    K_local(10,2) = -6*EIy_L3*L; K_local(10,4) = 2*EIy_L3*L*L; K_local(10,8) = 6*EIy_L3*L; K_local(10,10) = 4*EIy_L3*L*L;

    return K_local;
}

// Transform local stiffness to global coordinates
MatrixXd transformToGlobal(const MatrixXd &K_local, const Matrix3d &R) {
    MatrixXd T = MatrixXd::Zero(12, 12);
    for (int i = 0; i < 4; i++) {
        T.block<3,3>(3*i, 3*i) = R;
    }
    return T.transpose() * K_local * T;
}

// FEM solver (extracted from main function)
json solve_fem(const json& input) {
    Eigen::setNbThreads(1); // Crucial: avoid nested oversubscription
    try {
        // Parse nodes
        auto nodes_json = input["nodes"];
        int n_nodes = nodes_json.size();
        MatrixXd nodes(n_nodes, 3);
        for (int i = 0; i < n_nodes; i++) {
            nodes(i, 0) = nodes_json[i][0];
            nodes(i, 1) = nodes_json[i][1];
            nodes(i, 2) = nodes_json[i][2];
        }

        // Parse elements
        auto elems_json = input["elements"];
        int n_elems = elems_json.size();
        std::vector<BeamElement> elements(n_elems);
        double total_mass = 0.0;

        for (int e = 0; e < n_elems; e++) {
            int i = (int)elems_json[e][0] - 1;
            int j = (int)elems_json[e][1] - 1;

            if (i < 0 || i >= n_nodes || j < 0 || j >= n_nodes) {
                std::stringstream ss;
                ss << "Element " << e + 1 << " references out-of-bounds nodes: " << i + 1 << " or " << j + 1 << " (max nodes: " << n_nodes << ")";
                throw std::runtime_error(ss.str());
            }

            elements[e].node_i = i;
            elements[e].node_j = j;
            elements[e].D_outer = elems_json[e][2];
            elements[e].t = elems_json[e][3];
            elements[e].E = elems_json[e][4];
            elements[e].G = elems_json[e][5];
            elements[e].rho = elems_json[e][6];
            elements[e].shape = (int)elems_json[e][7];
            elements[e].joint_factor = elems_json[e][8];

            computeSectionProperties(elements[e]);

            Vector3d p1 = nodes.row(elements[e].node_i);
            Vector3d p2 = nodes.row(elements[e].node_j);
            elements[e].L = (p2 - p1).norm();
            elements[e].R = computeRotationMatrix(p1, p2);

            total_mass += elements[e].rho * elements[e].A * 1e-6 * elements[e].L;
        }

        // Assemble global stiffness matrix
        int n_dof = 6 * n_nodes;
        std::vector<T> triplets;
        triplets.reserve(144 * n_elems);

        for (int e = 0; e < n_elems; e++) {
            MatrixXd K_local = computeLocalStiffness(elements[e]);
            MatrixXd K_global = transformToGlobal(K_local, elements[e].R);

            int i = elements[e].node_i;
            int j = elements[e].node_j;

            for (int ii = 0; ii < 6; ii++) {
                for (int jj = 0; jj < 6; jj++) {
                    triplets.push_back(T(6*i + ii, 6*i + jj, K_global(ii, jj)));
                    triplets.push_back(T(6*i + ii, 6*j + jj, K_global(ii, jj+6)));
                    triplets.push_back(T(6*j + ii, 6*i + jj, K_global(ii+6, jj)));
                    triplets.push_back(T(6*j + ii, 6*j + jj, K_global(ii+6, jj+6)));
                }
            }
        }

        // Build adjacency list for connectivity analysis
        std::vector<std::vector<int>> adjacency(n_nodes);
        for (int e = 0; e < n_elems; e++) {
            adjacency[elements[e].node_i].push_back(elements[e].node_j);
            adjacency[elements[e].node_j].push_back(elements[e].node_i);
        }

        // Find which nodes have BCs
        auto bcs_json = input["bcs"];
        std::vector<bool> has_bc(n_nodes, false);
        for (size_t b = 0; b < bcs_json.size(); b++) {
            int node = (int)bcs_json[b][0] - 1;
            has_bc[node] = true;
        }

        // BFS from all BC nodes to find reachable nodes
        std::vector<bool> reachable(n_nodes, false);
        std::vector<int> bfs_queue;
        for (int n = 0; n < n_nodes; n++) {
            if (has_bc[n]) {
                reachable[n] = true;
                bfs_queue.push_back(n);
            }
        }
        for (size_t qi = 0; qi < bfs_queue.size(); qi++) {
            int cur = bfs_queue[qi];
            for (int neighbor : adjacency[cur]) {
                if (!reachable[neighbor]) {
                    reachable[neighbor] = true;
                    bfs_queue.push_back(neighbor);
                }
            }
        }

        // Apply boundary conditions to triplets
        VectorXd F = VectorXd::Zero(n_dof);
        double penalty = 1e15;

        // Constrain unreachable nodes
        for (int n = 0; n < n_nodes; n++) {
            if (!reachable[n]) {
                for (int d = 0; d < 6; d++) {
                    triplets.push_back(T(6*n + d, 6*n + d, penalty));
                }
            }
        }

        for (size_t b = 0; b < bcs_json.size(); b++) {
            int node = (int)bcs_json[b][0] - 1;
            int dof = (int)bcs_json[b][1] - 1;
            double value = bcs_json[b][2];

            int global_dof = 6*node + dof;
            triplets.push_back(T(global_dof, global_dof, penalty));
            F(global_dof) += penalty * value;
        }

        // Build sparse matrix
        SparseMatrix<double> K(n_dof, n_dof);
        K.setFromTriplets(triplets.begin(), triplets.end());

        // Apply loads
        auto loads_json = input["loads"];
        for (size_t l = 0; l < loads_json.size(); l++) {
            int node = (int)loads_json[l][0] - 1;
            int dof = (int)loads_json[l][1] - 1;
            double value = loads_json[l][2];

            int global_dof = 6*node + dof;
            F(global_dof) += value;
        }

        // Solve K*U = F
        SparseLU<SparseMatrix<double>> solver;
        solver.compute(K);
        if (solver.info() != Success) {
            throw std::runtime_error("Sparse LU decomposition failed");
        }

        VectorXd U = solver.solve(F);
        if (solver.info() != Success) {
            throw std::runtime_error("Linear solve failed");
        }

        std::vector<frame_optimize::FemLoad> fem_loads;
        fem_loads.reserve(loads_json.size());
        for (const auto& load_json : loads_json) {
            fem_loads.push_back({(int)load_json[0] - 1, (int)load_json[1] - 1, load_json[2]});
        }
        const double K_torsion = frame_optimize::compute_torsional_rigidity(nodes, U, fem_loads);


        // Create output JSON
        json output;
        output["success"] = true;
        output["mass"] = total_mass;
        output["K_torsion"] = K_torsion;

        // Convert displacement vector to 2D array
        std::vector<std::vector<double>> U_array(n_nodes, std::vector<double>(6));
        for (int i = 0; i < n_nodes; i++) {
            for (int j = 0; j < 6; j++) {
                U_array[i][j] = U(6*i + j);
            }
        }
        output["U"] = U_array;

        return output;

    } catch (const std::exception& e) {
        ERR("solve_fem exception: " << e.what());
        json error_output;
        error_output["success"] = false;
        error_output["error"] = e.what();
        return error_output;
    }
}

// TCP server main loop
int main(int argc, char* argv[]) {
    int port = 50051;
    if (argc > 1) port = std::atoi(argv[1]);

#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        ERR("WSAStartup failed");
        return 1;
    }
#endif

    LOG("Starting FEM server on port " << port << "...");

    socket_handle_t server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET_HANDLE) {
        ERR("Failed to create socket");
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    int opt = 1;
    setsockopt(
        server_fd,
        SOL_SOCKET,
        SO_REUSEADDR,
#ifdef _WIN32
        reinterpret_cast<const char*>(&opt),
#else
        &opt,
#endif
        sizeof(opt)
    );

    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
        ERR("Failed to bind socket");
        CLOSE_SOCKET(server_fd);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    if (listen(server_fd, 128) < 0) {
        ERR("Failed to listen");
        CLOSE_SOCKET(server_fd);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    const int num_workers = 16;
    std::vector<std::thread> workers;

    auto client_handler = [](socket_handle_t client_fd) {
        struct timeval tv;
        tv.tv_sec = 10;
        tv.tv_usec = 0;
        setsockopt(
            client_fd,
            SOL_SOCKET,
            SO_RCVTIMEO,
#ifdef _WIN32
            reinterpret_cast<const char*>(&tv),
#else
            &tv,
#endif
            sizeof(tv)
        );

        std::string request_str;
        char buffer[16384];
        int bytes_read = 0;
        while ((bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0)) > 0) {
            buffer[bytes_read] = '\0';
            request_str += buffer;
            if (request_str.find('\n') != std::string::npos) break;
        }

        if (!request_str.empty()) {
            try {
                json request = json::parse(request_str);
                std::string command = request["command"];

                if (command == "SHUTDOWN") {
                    LOG("Received shutdown command");
                    json response = {{"success", true}, {"message", "Shutting down"}};
                    std::string res_str = response.dump() + "\n";
                    send_all(client_fd, res_str);
                    should_shutdown = true;
                    queue_cv.notify_all();
                } else if (command == "SOLVE") {
                    json response = solve_fem(request);
                    std::string response_str = response.dump() + "\n";
                    send_all(client_fd, response_str);
                } else if (command == "PING") {
                    json response = {{"success", true}, {"status", "ALIVE"}};
                    std::string response_str = response.dump() + "\n";
                    send_all(client_fd, response_str);
                }
            } catch (...) {}
        }
        CLOSE_SOCKET(client_fd);
    };

    for (int i = 0; i < num_workers; i++) {
        workers.emplace_back([&client_handler]() {
            while (true) {
                socket_handle_t client_fd;
                {
                    std::unique_lock<std::mutex> lock(queue_mutex);
                    queue_cv.wait(lock, [] { return !task_queue.empty() || should_shutdown; });
                    if (should_shutdown && task_queue.empty()) return;
                    client_fd = task_queue.front();
                    task_queue.pop();
                }
                client_handler(client_fd);
            }
        });
    }

    LOG("Server ready. Listening on port " << port << " (Worker limit: " << num_workers << ")");

    while (!should_shutdown) {
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(server_fd, &readfds);

#ifdef _WIN32
        int ret = select(0, &readfds, nullptr, nullptr, &tv);
#else
        int ret = select(static_cast<int>(server_fd) + 1, &readfds, nullptr, nullptr, &tv);
#endif
        if (ret > 0 && FD_ISSET(server_fd, &readfds)) {
            socket_handle_t client_fd = accept(server_fd, nullptr, nullptr);
            if (client_fd != INVALID_SOCKET_HANDLE) {
                {
                    std::lock_guard<std::mutex> lock(queue_mutex);
                    task_queue.push(client_fd);
                }
                queue_cv.notify_one();
            }
        }
    }

    for (auto& w : workers) if (w.joinable()) w.join();
    CLOSE_SOCKET(server_fd);
#ifdef _WIN32
    WSACleanup();
#endif
    LOG("Server stopped");
    return 0;
}
