#pragma once

#include "Policy.h"
#include <functional>
#include <unordered_map>

// Singleton factory registry that maps task names to Policy constructors.
//
// Tasks self-register via the REGISTER_TASK macro, which creates a static
// Registrar object whose constructor calls register_task() before main() runs.
// main() then looks up the desired task by name and calls create(), threading
// the inference backend through so the factory can pass it to make_config().
//
// Usage:
//   // In a task .cpp file:
//   REGISTER_TASK("t1-velocity", T1Velocity);
//
//   // In main():
//   auto policy = TaskRegistry::instance().create("t1-velocity-flat", "onnx");
class TaskRegistry {
public:
    using Factory = std::function<std::unique_ptr<Policy>(const std::string& model_name,
                                                          const std::string& inference_backend)>;

    static TaskRegistry& instance();

    void register_task(const std::string& name, Factory factory);

    std::unique_ptr<Policy> create(const std::string& name,
                                   const std::string& inference_backend) const;
    std::unique_ptr<Policy> create(const std::string& name,
                                   const std::string& model_name,
                                   const std::string& inference_backend) const;
    bool has(const std::string& name) const;

    struct Registrar {
        Registrar(const std::string& name, Factory factory) {
            TaskRegistry::instance().register_task(name, std::move(factory));
        }
    };

private:
    TaskRegistry() = default;
    std::unordered_map<std::string, Factory> factories_;
};

#define REGISTER_TASK(task_name, ClassName)                                 \
    static TaskRegistry::Registrar __##ClassName##_registrar(               \
        task_name,                                                          \
        [](const std::string& model_name,                                   \
           const std::string& backend) -> std::unique_ptr<Policy> {         \
            return std::make_unique<ClassName>(model_name, backend);        \
        }                                                                   \
    );
