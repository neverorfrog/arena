#include "ModelRegistry.h"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>

#include "ModelConfig.h"

namespace fs = std::filesystem;

fs::path ModelRegistry::default_models_dir() {
    if (const char* env = std::getenv("MODELS_DIR"))
        return fs::path(env);

    // Installed layout: <prefix>/bin/main  →  <prefix>/models/
    char buf[4096];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len >= 0) {
        buf[len] = '\0';
        fs::path installed = fs::path(buf).parent_path().parent_path() / "models";
        if (fs::exists(installed)) return installed;
    }

    // Development layout: build/x86/main  →  PROJECT_ROOT/models/
    fs::path dev = fs::path(PROJECT_ROOT) / "models";
    if (fs::exists(dev)) return dev;

    throw std::runtime_error(
        "ModelRegistry: cannot locate models directory.\n"
        "Set MODELS_DIR to the models/ directory.");
}

namespace {

// ── YAML-based resolution ──────────────────────────────────────────────────

// Resolve via per-policy models.yaml (version → path mapping).
// Returns empty path if models.yaml doesn't exist.
fs::path try_yaml_resolve(const std::string& task_name,
                          const std::string& version_name,
                          const fs::path& models_dir) {
    fs::path yaml_path = models_dir / task_name / "models.yaml";
    if (!fs::exists(yaml_path)) return {};

    try {
        auto cfg = ModelConfig::parse_policy(yaml_path);

        // Use the named version if given, otherwise the default.
        const std::string& ver = version_name.empty()
            ? cfg.default_version
            : version_name;

        auto it = cfg.versions.find(ver);
        if (it == cfg.versions.end()) {
            // If explicitly requested but not found, error.
            if (!version_name.empty())
                throw std::runtime_error(
                    "ModelRegistry: version '" + version_name + "' not found in " +
                    yaml_path.string());
            // Default version missing? Fall through to directory scan.
            return {};
        }

        fs::path model_path = models_dir / task_name / it->second.path;
        if (fs::exists(model_path)) return fs::canonical(model_path);

        throw std::runtime_error(
            "ModelRegistry: model file not found: " + model_path.string());
    } catch (const std::runtime_error&) {
        if (!version_name.empty()) throw; // explicit request — propagate error
        return {};                         // default fallback
    }
}

// Read central models.yaml for the default version of a task.
std::string read_default_version(const std::string& task_name,
                                 const fs::path& models_dir) {
    fs::path yaml_path = models_dir / "models.yaml";
    if (!fs::exists(yaml_path)) return {};

    auto defaults = ModelConfig::parse_defaults(yaml_path);
    auto it = defaults.find(task_name);
    return (it != defaults.end()) ? it->second : std::string{};
}

// ── Directory-scan fallback ────────────────────────────────────────────────

fs::path resolve_dir(const std::string& task_name,
                     const std::string& model_name,
                     const fs::path& models_dir) {
    fs::path task_dir = models_dir / task_name;

    if (!fs::is_directory(task_dir))
        throw std::runtime_error(
            "ModelRegistry: task directory not found: " + task_dir.string() +
            "\nSet MODELS_DIR to the project-level models/ directory.");

    // Named resolution: models/<task>/<model_name>/<first>.onnx
    if (!model_name.empty()) {
        fs::path model_dir = task_dir / model_name;
        if (!fs::is_directory(model_dir))
            throw std::runtime_error(
                "ModelRegistry: model '" + model_name + "' not found for task '" +
                task_name + "'\nExpected: " + model_dir.string());

        for (const auto& entry : fs::directory_iterator(model_dir)) {
            if (!entry.is_regular_file()) continue;
            const std::string fname = entry.path().filename().string();
            if (fname.size() >= 5 && fname.substr(fname.size() - 5) == ".onnx")
                return fs::canonical(entry.path());
        }
        throw std::runtime_error(
            "ModelRegistry: no .onnx file in " + model_dir.string());
    }

    // 1. Prefer default.onnx symlink.
    fs::path default_link = task_dir / "default.onnx";
    if (fs::exists(default_link))
        return fs::canonical(default_link);

    // 2. Scan for <task_name>_*.onnx — lexicographically largest.
    const std::string prefix = task_name + "_";
    std::optional<fs::path> best_path;

    for (const auto& entry : fs::recursive_directory_iterator(task_dir)) {
        if (!entry.is_regular_file() && !entry.is_symlink()) continue;
        const std::string fname = entry.path().filename().string();
        if (fname.rfind(prefix, 0) != 0) continue;
        if (fname.size() < prefix.size() + 5) continue;
        if (fname.substr(fname.size() - 5) != ".onnx") continue;

        if (!best_path || fname > best_path->filename().string())
            best_path = entry.path();
    }

    if (best_path) return *best_path;

    throw std::runtime_error(
        "ModelRegistry: no ONNX model found for task '" + task_name +
        "' in " + task_dir.string() +
        "\nExpected: default.onnx  or  " + task_name + "_<suffix>.onnx");
}

} // anonymous namespace

// ── Public resolve overloads ─────────────────────────────────────────────────

fs::path ModelRegistry::resolve(const std::string& task_name) {
    return resolve(task_name, default_models_dir());
}

fs::path ModelRegistry::resolve(const std::string& task_name,
                                const std::string& model_name) {
    fs::path models_dir = default_models_dir();

    // Use central models.yaml default when no explicit model_name given.
    std::string effective_name = model_name;
    if (model_name.empty()) {
        effective_name = read_default_version(task_name, models_dir);
    }

    // Try YAML-based resolution first.
    fs::path yaml_result = try_yaml_resolve(task_name, effective_name, models_dir);
    if (!yaml_result.empty()) return yaml_result;

    // Fall back to directory scanning.
    return resolve_dir(task_name, effective_name, models_dir);
}

fs::path ModelRegistry::resolve(const std::string& task_name,
                                const fs::path& models_dir) {
    // Try YAML-based resolution first (no MODELS_CONFIG override for this overload).
    fs::path yaml_result = try_yaml_resolve(task_name, "", models_dir);
    if (!yaml_result.empty()) return yaml_result;

    return resolve_dir(task_name, "", models_dir);
}
