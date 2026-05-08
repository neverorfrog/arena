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

class ModelConfig {
public:
    // Parse per-policy models.yaml (version → path mapping).
    static PolicyModelConfig parse_policy(const std::filesystem::path& yaml_path);

    // Parse central models.yaml (policy → default version).
    static ModelDefaults parse_defaults(const std::filesystem::path& yaml_path);
};
