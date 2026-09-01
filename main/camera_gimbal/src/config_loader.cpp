#include "xtnetrc_gimbal/gimbal.hpp"

#include <fstream>
#include <iterator>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace xtnetrc::gimbal {
namespace {

[[nodiscard]] std::unordered_map<std::string, std::string> read_values(
    const std::string& text) {
    const std::regex object_pattern(R"("camera_gimbal"\s*:\s*\{([^{}]*)\})");
    std::smatch object_match;
    if (!std::regex_search(text, object_match, object_pattern)) {
        throw std::invalid_argument(
            "configuration has no simple camera_gimbal object");
    }

    const std::string body = object_match[1].str();
    const std::regex pair_pattern(
        R"json("([A-Za-z0-9_]+)"\s*:\s*([-+]?[0-9]+))json");
    std::unordered_map<std::string, std::string> values;

    // 删除所有合法的 key:value 后，只应剩逗号和空白；由此拒绝字符串、
    // 注释、嵌套对象和其他无法被本模块安全理解的配置内容。
    const std::string separators = std::regex_replace(body, pair_pattern, "");
    for (const char ch : separators) {
        if (ch != ',' && ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
            throw std::invalid_argument(
                "camera_gimbal object contains a non-integer value");
        }
    }

    for (std::sregex_iterator it(body.begin(), body.end(), pair_pattern), end;
         it != end; ++it) {
        const std::string key = (*it)[1].str();
        if (!values.emplace(key, (*it)[2].str()).second) {
            throw std::invalid_argument("duplicate camera_gimbal key: " + key);
        }
    }
    return values;
}

[[nodiscard]] int take_int(std::unordered_map<std::string, std::string>& values,
                           const std::string& key) {
    const auto found = values.find(key);
    if (found == values.end()) {
        throw std::invalid_argument("missing camera_gimbal key: " + key);
    }
    const long long parsed = std::stoll(found->second);
    if (parsed < std::numeric_limits<int>::min() ||
        parsed > std::numeric_limits<int>::max()) {
        throw std::invalid_argument("camera_gimbal integer is out of range: " + key);
    }
    values.erase(found);
    return static_cast<int>(parsed);
}

}  // namespace

GimbalConfig load_gimbal_config_json(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open gimbal configuration: " + path);
    }
    const std::string text((std::istreambuf_iterator<char>(input)),
                           std::istreambuf_iterator<char>());
    auto values = read_values(text);

    GimbalConfig config;
    config.pan_gpio_bcm = take_int(values, "pan_gpio_bcm");
    config.tilt_gpio_bcm = take_int(values, "tilt_gpio_bcm");
    config.pwm_frequency_hz = take_int(values, "pwm_frequency_hz");
    config.pwm_range = take_int(values, "pwm_range");
    config.pan_center_pwm = take_int(values, "pan_center_pwm");
    config.pan_min_pwm = take_int(values, "pan_min_pwm");
    config.pan_max_pwm = take_int(values, "pan_max_pwm");
    config.tilt_center_pwm = take_int(values, "tilt_center_pwm");
    config.tilt_min_pwm = take_int(values, "tilt_min_pwm");
    config.tilt_max_pwm = take_int(values, "tilt_max_pwm");
    config.max_step_pwm = take_int(values, "max_step_pwm");
    config.step_interval_ms = take_int(values, "step_interval_ms");

    if (!values.empty()) {
        throw std::invalid_argument("unknown camera_gimbal key: " +
                                    values.begin()->first);
    }
    config.validate();
    return config;
}

}  // namespace xtnetrc::gimbal
