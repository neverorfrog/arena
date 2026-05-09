#include "ModelConfig.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

// ── helpers ──────────────────────────────────────────────────────────────────
namespace {

std::string trim(std::string s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char c) { return !std::isspace(c); }));
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char c) { return !std::isspace(c); }).base(), s.end());
    return s;
}

int count_indent(const std::string& line) {
    int n = 0;
    for (char c : line) {
        if (c == ' ') n++;
        else break;
    }
    return n;
}

// Split "key: value" into {key, value}.  Strips whitespace.
std::pair<std::string, std::string> split_kv(const std::string& line) {
    auto pos = line.find(':');
    if (pos == std::string::npos) return {"", ""};
    return {trim(line.substr(0, pos)), trim(line.substr(pos + 1))};
}

std::string read_file(const fs::path& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open " + path.string());
    std::stringstream buf;
    buf << f.rdbuf();
    return buf.str();
}

} // anonymous namespace

// ── PolicyModelConfig parser ──────────────────────────────────────────────────
//
// Expected format (default field is optional — central models.yaml is the source of truth):
//   versions:
//     v5:
//       path: v5/t1-velocity_ppo_v5.onnx
//       backend: trt    # optional
//     experimental:
//       path: experimental/t1-velocity_ppo_v6.onnx
//
PolicyModelConfig ModelConfig::parse_policy(const fs::path& yaml_path) {
    PolicyModelConfig cfg;
    std::string content = read_file(yaml_path);
    std::istringstream in(content);
    std::string line;

    std::string current_version;

    while (std::getline(in, line)) {
        int indent = count_indent(line);
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        auto [key, value] = split_kv(line);

        if (indent == 0) {
            if (key == "default") {
                cfg.default_version = value;
            }
            // "versions:" is just a section header, skip it
        } else if (indent == 2) {
            // Version name (value is empty for section headers like "v5:")
            current_version = key;
            cfg.versions[current_version] = ModelVersion{};
        } else if (indent == 4 && !current_version.empty()) {
            if (key == "path") {
                cfg.versions[current_version].path = value;
            } else if (key == "backend") {
                cfg.versions[current_version].backend = value;
            }
        }
    }

    if (cfg.versions.empty())
        throw std::runtime_error(yaml_path.string() + ": no versions defined");

    return cfg;
}

// ── ModelDefaults parser ─────────────────────────────────────────────────────
//
// Expected format (central models.yaml):
//   t1-velocity: v5
//   shooting: v3
//
ModelDefaults ModelConfig::parse_defaults(const fs::path& yaml_path) {
    ModelDefaults defaults;
    std::string content = read_file(yaml_path);
    std::istringstream in(content);
    std::string line;

    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        auto [key, value] = split_kv(line);
        if (!key.empty() && !value.empty())
            defaults[key] = value;
    }

    return defaults;
}
