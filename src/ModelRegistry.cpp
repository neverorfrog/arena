#include "ModelRegistry.h"

#include <algorithm>
#include <cstdlib>
#include <regex>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;

fs::path ModelRegistry::default_models_dir() {
    // Allow override via environment variable (useful during development when
    // running from build dir rather than install prefix).
    if (const char* env = std::getenv("MODELS_DIR"))
        return fs::path(env);

    char buf[4096];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len < 0)
        throw std::runtime_error("ModelRegistry: cannot read /proc/self/exe");
    buf[len] = '\0';
    // Installed layout: <prefix>/bin/main  ->  <prefix>/models/
    return fs::path(buf).parent_path().parent_path() / "models";
}

namespace {
    fs::path resolve_named(const std::string& task_name,
                           const std::string& model_name,
                           const fs::path& models_dir) {
        fs::path model_dir = models_dir / task_name / model_name;
        if (!fs::is_directory(model_dir))
            throw std::runtime_error(
                "ModelRegistry: model '" + model_name + "' not found for task '" + task_name +
                "'\nExpected: " + model_dir.string());

        for (const auto& entry : fs::directory_iterator(model_dir)) {
            if (!entry.is_regular_file()) continue;
            const std::string fname = entry.path().filename().string();
            if (fname.size() >= 5 && fname.substr(fname.size() - 5) == ".onnx")
                return fs::canonical(entry.path());
        }

        throw std::runtime_error(
            "ModelRegistry: no .onnx file in " + model_dir.string());
    }
}

fs::path ModelRegistry::resolve(const std::string& task_name) {
    return resolve(task_name, default_models_dir());
}

fs::path ModelRegistry::resolve(const std::string& task_name,
                                const std::string& model_name) {
    return resolve_named(task_name, model_name, default_models_dir());
}

fs::path ModelRegistry::resolve(const std::string& task_name,
                                const fs::path& models_dir) {
    // Models are stored per-task in named subdirectories:
    //   <models_dir>/<task_name>/<name>/<task_name>_<algo>_<name>.onnx
    //   <models_dir>/<task_name>/default.onnx -> <name>/<task_name>_<algo>_<name>.onnx
    fs::path task_dir = models_dir / task_name;

    if (!fs::is_directory(task_dir))
        throw std::runtime_error(
            "ModelRegistry: task directory not found: " + task_dir.string() +
            "\nSet MODELS_DIR to the project-level models/ directory.");

    // 1. Prefer default.onnx symlink (set by 'pixi run export-onnx').
    fs::path default_link = task_dir / "default.onnx";
    if (fs::exists(default_link))
        return fs::canonical(default_link);

    // 2. Scan subdirectories for <task_name>_*.onnx and pick the
    //    lexicographically largest name.
    const std::string prefix = task_name + "_";
    std::optional<fs::path> best_path;

    for (const auto& entry : fs::recursive_directory_iterator(task_dir)) {
        if (!entry.is_regular_file() && !entry.is_symlink()) continue;
        const std::string fname = entry.path().filename().string();
        if (fname.rfind(prefix, 0) != 0) continue;          // must start with prefix
        if (fname.size() < prefix.size() + 5) continue;     // at least some suffix
        if (fname.substr(fname.size() - 5) != ".onnx") continue;

        if (!best_path || fname > best_path->filename().string())
            best_path = entry.path();
    }

    if (best_path)
        return *best_path;

    throw std::runtime_error(
        "ModelRegistry: no ONNX model found for task '" + task_name +
        "' in " + task_dir.string() +
        "\nExpected: default.onnx  or  " + task_name + "_<suffix>.onnx");
}
