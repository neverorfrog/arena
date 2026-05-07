#pragma once
#include <filesystem>
#include <optional>
#include <string>

// Discovers ONNX model files for a given task name.
//
// Search order (stops at first hit):
//   1. <models_dir>/<task_name>/default.onnx  (registry default)
//   2. <models_dir>/<task_name>/<task_name>_*.onnx  (lexicographically largest)
//
// Named resolution (resolve(task, model_name)):
//   Looks in <models_dir>/<task_name>/<model_name>/ for the first .onnx file.
//
// The models_dir defaults to the directory containing the running executable's
// parent: <exe_parent>/models/  (mirrors the CMake install layout).
class ModelRegistry {
public:
    // Returns the path to the best available ONNX model for task_name,
    // or throws std::runtime_error if none is found.
    static std::filesystem::path resolve(const std::string& task_name);

    // Resolve a specific named model: models/<task>/<model_name>/<first>.onnx
    static std::filesystem::path resolve(const std::string& task_name,
                                         const std::string& model_name);

    // Same as resolve() but with an explicit models directory.
    static std::filesystem::path resolve(const std::string& task_name,
                                         const std::filesystem::path& models_dir);

    // Returns the default models directory (next to executable).
    static std::filesystem::path default_models_dir();
};
