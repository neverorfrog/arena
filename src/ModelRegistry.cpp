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

fs::path ModelRegistry::resolve(const std::string& task_name) {
    return resolve(task_name, default_models_dir());
}

fs::path ModelRegistry::resolve(const std::string& task_name,
                                const fs::path& models_dir) {
    // Models are stored per-task: <models_dir>/<task_name>/<task_name>_*.onnx
    fs::path task_dir = models_dir / task_name;

    if (!fs::is_directory(task_dir))
        throw std::runtime_error(
            "ModelRegistry: task directory not found: " + task_dir.string() +
            "\nSet MODELS_DIR to the project-level models/ directory.");

    // 1. Prefer explicit _latest symlink or file.
    fs::path latest = task_dir / (task_name + "_latest.onnx");
    if (fs::exists(latest))
        return fs::canonical(latest);

    // 2. Scan for <task_name>_<timestamp>.onnx and pick the lexicographically
    //    largest name (ISO timestamps sort correctly as strings).
    // Pattern: "<task_name>_<digits_and_underscores>.onnx"
    const std::string prefix = task_name + "_";
    std::optional<fs::path> best_path;

    for (const auto& entry : fs::directory_iterator(task_dir)) {
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
        "\nExpected: " + task_name + "_latest.onnx  or  " + task_name + "_<timestamp>.onnx");
}
