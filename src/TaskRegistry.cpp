#include "TaskRegistry.h"


TaskRegistry& TaskRegistry::instance() {
    static TaskRegistry inst;
    return inst;
}

void TaskRegistry::register_task(const std::string& name, Factory factory) {
    factories_.emplace(name, std::move(factory));
}

std::unique_ptr<Policy> TaskRegistry::create(const std::string& name,
                                      const std::string& inference_backend) const {
    return create(name, "", inference_backend);
}

std::unique_ptr<Policy> TaskRegistry::create(const std::string& name,
                                      const std::string& model_name,
                                      const std::string& inference_backend) const {
    auto it = factories_.find(name);
    if (it == factories_.end())
        throw std::runtime_error("TaskRegistry: unknown task '" + name + "'");
    return it->second(model_name, inference_backend);
}

bool TaskRegistry::has(const std::string& name) const {
    return factories_.count(name) > 0;
}
