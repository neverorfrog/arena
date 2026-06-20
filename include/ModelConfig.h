#pragma once
#include <filesystem>
#include <map>
#include <optional>
#include <string>

struct ModelVersion {
    std::string path;
    std::optional<std::string> backend;
};

struct PolicyModelConfig {
    std::string default_version;
    std::map<std::string, ModelVersion> versions;
};

using ModelDefaults = std::map<std::string, std::string>;

// Per-joint deploy gains, distilled from the training config and shipped next to
// each ONNX as gains.yaml. Loaded by joint name so it is independent of joint order.
struct JointGains {
    float kp           = 0.0f;
    float kd           = 0.0f;
    float armature     = 0.0f;
    float frictionloss = 0.0f;
    float effort_limit = 0.0f;
    float default_pos  = 0.0f;
    float action_scale = 0.0f;
};

using GainsMap = std::map<std::string, JointGains>;

class ModelConfig {
public:
    // Parse per-policy models.yaml (version → path mapping).
    static PolicyModelConfig parse_policy(const std::filesystem::path& yaml_path);

    // Parse central models.yaml (policy → default version).
    static ModelDefaults parse_defaults(const std::filesystem::path& yaml_path);

    // Parse gains.yaml (flat "Joint.field: value" lines) → joint-name-keyed gains.
    static GainsMap parse_gains(const std::filesystem::path& yaml_path);
};
