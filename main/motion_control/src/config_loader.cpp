#include "xtnetrc_motion/controller.hpp"

#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace xtnetrc::motion {
namespace {

[[nodiscard]] std::unordered_map<std::string, std::string> read_motion_values(
    const std::string& text) {
    const std::regex motion_pattern(R"("motion"\s*:\s*\{([^{}]*)\})");
    std::smatch motion_match;
    if (!std::regex_search(text, motion_match, motion_pattern)) {
        throw std::invalid_argument("configuration has no simple motion object");
    }

    const std::string body = motion_match[1].str();
    const std::regex pair_pattern(
        R"json("([A-Za-z0-9_]+)"\s*:\s*([-+]?(?:[0-9]+\.?[0-9]*|\.[0-9]+)(?:[eE][-+]?[0-9]+)?))json");
    std::unordered_map<std::string, std::string> values;
    std::string separators = std::regex_replace(body, pair_pattern, "");
    for (const char ch : separators) {
        if (ch != ',' && ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
            throw std::invalid_argument("motion object contains a non-numeric value");
        }
    }
    for (std::sregex_iterator it(body.begin(), body.end(), pair_pattern), end;
         it != end; ++it) {
        const std::string key = (*it)[1].str();
        if (!values.emplace(key, (*it)[2].str()).second) {
            throw std::invalid_argument("duplicate motion configuration key: " + key);
        }
    }
    return values;
}

[[nodiscard]] double take_double(std::unordered_map<std::string, std::string>& values,
                                 const std::string& key) {
    const auto found = values.find(key);
    if (found == values.end()) {
        throw std::invalid_argument("missing motion configuration key: " + key);
    }
    const double value = std::stod(found->second);
    values.erase(found);
    return value;
}

}  // namespace

MotionConfig load_motion_config_json(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open motion configuration: " + path);
    }
    const std::string text((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    auto values = read_motion_values(text);
    MotionConfig config;
    config.wheelbase_m = take_double(values, "wheelbase_m");
    config.lookahead_m = take_double(values, "lookahead_m");
    config.max_steering_deg = take_double(values, "max_steering_deg");
    config.max_steering_rate_deg_s = take_double(values, "max_steering_rate_deg_s");
    config.max_speed_mps = take_double(values, "max_speed_mps");
    config.min_curve_speed_mps = take_double(values, "min_curve_speed_mps");
    config.degraded_max_speed_mps = take_double(values, "degraded_max_speed_mps");
    config.curvature_speed_gain = take_double(values, "curvature_speed_gain");
    config.confidence_stop = take_double(values, "confidence_stop");
    config.confidence_degraded = take_double(values, "confidence_degraded");
    config.max_path_distance_m = take_double(values, "max_path_distance_m");
    config.max_lateral_offset_m = take_double(values, "max_lateral_offset_m");
    config.max_accel_mps2 = take_double(values, "max_accel_mps2");
    config.max_decel_mps2 = take_double(values, "max_decel_mps2");
    config.confidence_hysteresis = take_double(values, "confidence_hysteresis");
    const double frames = take_double(values, "confidence_reacquire_frames");
    if (frames < 1.0 ||
        frames > static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
        std::floor(frames) != frames) {
        throw std::invalid_argument("confidence_reacquire_frames must be an integer");
    }
    config.confidence_reacquire_frames = static_cast<std::size_t>(frames);
    if (!values.empty()) {
        throw std::invalid_argument("unknown motion configuration key: " +
                                    values.begin()->first);
    }
    config.validate();
    return config;
}

}  // namespace xtnetrc::motion
