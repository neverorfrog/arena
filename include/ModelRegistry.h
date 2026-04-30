#pragma once
#include <filesystem>
#include <optional>
#include <string>

// Discovers ONNX model files for a given task name.
//
// Search order (stops at first hit):
//   1. <models_dir>/<task_name>_latest.onnx
//   2. <models_dir>/<task_name>_latest.onnx -> symlink target
//   3. <models_dir>/<task_name>_<step>.onnx  with highest numeric <step>
//
// The models_dir defaults to the directory containing the running executable's
// parent: <exe_parent>/models/  (mirrors the CMake install layout).
class ModelRegistry {
public:
    // Returns the path to the best available ONNX model for task_name,
    // or throws std::runtime_error if none is found.
    static std::filesystem::path resolve(const std::string& task_name);

    // Same as resolve() but with an explicit models directory.
    static std::filesystem::path resolve(const std::string& task_name,
                                         const std::filesystem::path& models_dir);

    // Returns the default models directory (next to executable).
    static std::filesystem::path default_models_dir();
};
