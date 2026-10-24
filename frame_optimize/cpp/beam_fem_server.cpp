/**
 * @file beam_fem_server.cpp
 * @brief TCP server for 3D Timoshenko beam FEM solver (kernel: beam_fem_core.hpp)
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
 * {"success": true, "U": [...], "K_torsion": ..., "mass": ..., "element_forces": ..., "element_stress_vm": ...,
 *  "stress_max": ..., "element_buckling_ratio": ..., "buckling_ratio": ...}   (see beam_fem_standalone.cpp)
 * {"success": false, "error": "..."}
 */

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
#include "beam_fem_core.hpp"
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

using json = nlohmann::json;

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

json solve_fem(const json& input) {
    Eigen::setNbThreads(1); // Crucial: avoid nested oversubscription
    try {
        return frame_optimize::solve_json(input);
    } catch (const std::exception& e) {
        ERR("solve_fem exception: " << e.what());
        return json{{"success", false}, {"error", e.what()}};
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
