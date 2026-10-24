#pragma once
/// @file velocity_profile_io.hpp
/// @brief Versioned, validated I/O for station-based velocity profiles.

#include "velocity_profile.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace fsae::laptime {

inline constexpr const char* kVelocityProfileSchema = "fsae_velocity_profile_v1";

struct VelocityProfileArtifact {
    std::string track_name;
    double track_length_m = 0.0;
    bool closed_loop = false;
    std::string track_geometry_fingerprint;
    std::string profile_fingerprint;
    std::vector<VelocityPoint> points;
};

namespace detail {

inline void fnv1a_append(std::uint64_t& hash, const std::string& text) {
    constexpr std::uint64_t prime = 1099511628211ULL;
    for (const unsigned char byte : text) {
        hash ^= static_cast<std::uint64_t>(byte);
        hash *= prime;
    }
}

inline std::string fnv1a_hex(const std::string& text) {
    std::uint64_t hash = 14695981039346656037ULL;
    fnv1a_append(hash, text);
    std::ostringstream output;
    output << "fnv1a64:" << std::hex << std::setfill('0') << std::setw(16) << hash;
    return output.str();
}

inline nlohmann::json point_to_json(const VelocityPoint& point) {
    nlohmann::json result = {
        {"s_m", point.s},
        {"speed_mps", point.v},
        {"ax_mps2", point.ax},
        {"ay_mps2", point.ay},
        {"curvature_1pm", point.kappa},
    };
    result["steering_speed_cap_mps"] = std::isfinite(point.steering_speed_cap)
        ? nlohmann::json(point.steering_speed_cap)
        : nlohmann::json(nullptr);
    return result;
}

inline nlohmann::json points_to_json(const std::vector<VelocityPoint>& points) {
    nlohmann::json result = nlohmann::json::array();
    for (const auto& point : points) {
        result.push_back(point_to_json(point));
    }
    return result;
}

inline double required_number(const nlohmann::json& object, const char* key) {
    if (!object.contains(key) || !object.at(key).is_number()) {
        throw std::runtime_error(std::string("Velocity profile point is missing numeric '") + key + "'");
    }
    const double value = object.at(key).get<double>();
    if (!std::isfinite(value)) {
        throw std::runtime_error(std::string("Velocity profile point has non-finite '") + key + "'");
    }
    return value;
}

inline VelocityPoint point_from_json(const nlohmann::json& object) {
    if (!object.is_object()) {
        throw std::runtime_error("Velocity profile point must be a JSON object");
    }
    VelocityPoint point{
        required_number(object, "s_m"),
        required_number(object, "speed_mps"),
        required_number(object, "ax_mps2"),
        required_number(object, "ay_mps2"),
        required_number(object, "curvature_1pm"),
    };
    if (!object.contains("steering_speed_cap_mps") ||
        object.at("steering_speed_cap_mps").is_null()) {
        point.steering_speed_cap = std::numeric_limits<double>::infinity();
    } else if (object.at("steering_speed_cap_mps").is_number()) {
        point.steering_speed_cap = object.at("steering_speed_cap_mps").get<double>();
        if (!std::isfinite(point.steering_speed_cap) || point.steering_speed_cap <= 0.0) {
            throw std::runtime_error("Velocity profile steering speed cap must be finite and positive or null");
        }
    } else {
        throw std::runtime_error("Velocity profile steering speed cap must be numeric or null");
    }
    return point;
}

} // namespace detail

inline std::string velocity_profile_fingerprint(const std::vector<VelocityPoint>& points) {
    return detail::fnv1a_hex(detail::points_to_json(points).dump());
}

inline std::string track_geometry_fingerprint(const track::CurvatureBasedTrack& track) {
    nlohmann::json samples = nlohmann::json::array();
    const auto quantize = [](double value, double scale) {
        if (!std::isfinite(value)) {
            throw std::runtime_error("Track geometry contains non-finite data");
        }
        return static_cast<std::int64_t>(std::llround(value * scale));
    };
    for (const auto& point : track.points()) {
        samples.push_back({
            quantize(point.s, 1e6),
            quantize(point.x, 1e6),
            quantize(point.y, 1e6),
            quantize(point.theta, 1e9),
            quantize(point.kappa, 1e9),
            quantize(point.width_left, 1e6),
            quantize(point.width_right, 1e6),
        });
    }
    return detail::fnv1a_hex(samples.dump());
}

inline void validate_velocity_profile_points(
    const std::vector<VelocityPoint>& points,
    double track_length_m,
    bool closed_loop)
{
    if (!std::isfinite(track_length_m) || track_length_m <= 0.0) {
        throw std::runtime_error("Velocity profile track length must be finite and positive");
    }
    if (points.size() < 2) {
        throw std::runtime_error("Velocity profile must contain at least two points");
    }
    if (std::abs(points.front().s) > 1e-9) {
        throw std::runtime_error("Velocity profile must start at station zero");
    }

    double max_station_step = 0.0;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const auto& point = points[index];
        if (!std::isfinite(point.s) || !std::isfinite(point.v) ||
            !std::isfinite(point.ax) || !std::isfinite(point.ay) ||
            !std::isfinite(point.kappa)) {
            throw std::runtime_error("Velocity profile contains non-finite point data");
        }
        if (point.v <= 0.0) {
            throw std::runtime_error("Velocity profile speeds must be positive");
        }
        if (std::isnan(point.steering_speed_cap) || point.steering_speed_cap <= 0.0) {
            throw std::runtime_error(
                "Velocity profile steering speed caps must be positive or positive infinity");
        }
        if (index > 0) {
            const double station_step = point.s - points[index - 1].s;
            if (!(station_step > 0.0)) {
                throw std::runtime_error("Velocity profile stations must be strictly increasing");
            }
            max_station_step = std::max(max_station_step, station_step);
        }
    }

    const double end_tolerance = std::max(1e-6, 1.01 * max_station_step);
    if (points.back().s > track_length_m + 1e-6 ||
        track_length_m - points.back().s > end_tolerance) {
        throw std::runtime_error("Velocity profile endpoint is inconsistent with track length");
    }
    if (closed_loop && std::abs(points.front().v - points.back().v) > 1e-9) {
        throw std::runtime_error("Closed-loop velocity profile has a discontinuous speed seam");
    }
}

inline VelocityProfileArtifact make_velocity_profile_artifact(
    const std::vector<VelocityPoint>& points,
    const std::string& track_name,
    const track::CurvatureBasedTrack& track)
{
    validate_velocity_profile_points(points, track.length(), track.is_closed_loop());
    VelocityProfileArtifact artifact;
    artifact.track_name = track_name;
    artifact.track_length_m = track.length();
    artifact.closed_loop = track.is_closed_loop();
    artifact.track_geometry_fingerprint = track_geometry_fingerprint(track);
    artifact.profile_fingerprint = velocity_profile_fingerprint(points);
    artifact.points = points;
    return artifact;
}

inline nlohmann::json velocity_profile_artifact_to_json(const VelocityProfileArtifact& artifact) {
    return {
        {"schema", kVelocityProfileSchema},
        {"profile_kind", "station_speed_reference"},
        {"station_frame", "track_centerline_s_m"},
        {"track", {
            {"name", artifact.track_name},
            {"length_m", artifact.track_length_m},
            {"closed_loop", artifact.closed_loop},
            {"geometry_fingerprint", artifact.track_geometry_fingerprint},
        }},
        {"profile_fingerprint", artifact.profile_fingerprint},
        {"points", detail::points_to_json(artifact.points)},
    };
}

inline void save_velocity_profile_artifact(
    const VelocityProfileArtifact& artifact,
    const std::string& filepath)
{
    std::ofstream output(filepath);
    if (!output) {
        throw std::runtime_error("Failed to write velocity profile: " + filepath);
    }
    output << velocity_profile_artifact_to_json(artifact).dump(2) << '\n';
    if (!output) {
        throw std::runtime_error("Failed while writing velocity profile: " + filepath);
    }
}

inline VelocityProfileArtifact load_velocity_profile_artifact(
    const std::string& filepath,
    const std::string& expected_track_name,
    const track::CurvatureBasedTrack& expected_track)
{
    std::ifstream input(filepath);
    if (!input) {
        throw std::runtime_error("Failed to open velocity profile: " + filepath);
    }
    nlohmann::json root;
    input >> root;
    if (!root.is_object() || root.value("schema", std::string()) != kVelocityProfileSchema) {
        throw std::runtime_error("Unsupported velocity profile schema: " + filepath);
    }
    if (root.value("profile_kind", std::string()) != "station_speed_reference" ||
        root.value("station_frame", std::string()) != "track_centerline_s_m") {
        throw std::runtime_error("Velocity profile kind or station frame is incompatible");
    }
    if (!root.contains("track") || !root.at("track").is_object()) {
        throw std::runtime_error("Velocity profile is missing track metadata");
    }
    const auto& track_json = root.at("track");
    if (!track_json.contains("name") || !track_json.at("name").is_string() ||
        !track_json.contains("length_m") || !track_json.at("length_m").is_number() ||
        !track_json.contains("closed_loop") || !track_json.at("closed_loop").is_boolean() ||
        !track_json.contains("geometry_fingerprint") ||
        !track_json.at("geometry_fingerprint").is_string()) {
        throw std::runtime_error("Velocity profile track metadata is incomplete");
    }

    VelocityProfileArtifact artifact;
    artifact.track_name = track_json.at("name").get<std::string>();
    artifact.track_length_m = track_json.at("length_m").get<double>();
    artifact.closed_loop = track_json.at("closed_loop").get<bool>();
    artifact.track_geometry_fingerprint =
        track_json.at("geometry_fingerprint").get<std::string>();
    if (!std::isfinite(artifact.track_length_m)) {
        throw std::runtime_error("Velocity profile track length is non-finite");
    }
    if (artifact.track_name != expected_track_name) {
        throw std::runtime_error("Velocity profile track name does not match the active track");
    }
    const double length_tolerance = std::max(1e-6, 1e-6 * expected_track.length());
    if (std::abs(artifact.track_length_m - expected_track.length()) > length_tolerance) {
        throw std::runtime_error("Velocity profile track length does not match the active track");
    }
    if (artifact.closed_loop != expected_track.is_closed_loop()) {
        throw std::runtime_error("Velocity profile closed-loop flag does not match the active track");
    }
    if (artifact.track_geometry_fingerprint != track_geometry_fingerprint(expected_track)) {
        throw std::runtime_error("Velocity profile track geometry does not match the active track");
    }
    if (!root.contains("points") || !root.at("points").is_array()) {
        throw std::runtime_error("Velocity profile is missing its point array");
    }
    artifact.points.reserve(root.at("points").size());
    for (const auto& point_json : root.at("points")) {
        artifact.points.push_back(detail::point_from_json(point_json));
    }
    validate_velocity_profile_points(
        artifact.points, artifact.track_length_m, artifact.closed_loop);

    artifact.profile_fingerprint = velocity_profile_fingerprint(artifact.points);
    if (!root.contains("profile_fingerprint") ||
        !root.at("profile_fingerprint").is_string() ||
        root.at("profile_fingerprint").get<std::string>() != artifact.profile_fingerprint) {
        throw std::runtime_error("Velocity profile fingerprint mismatch");
    }
    return artifact;
}

} // namespace fsae::laptime
