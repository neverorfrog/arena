#include "engines/OnnxInferenceEngine.h"

#include <algorithm>
#include <array>
#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <string>

// Validates model path before it's passed to Ort::Session (which segfaults on bad paths).
static const std::string& checked_path(const std::string& path) {
    if (!std::filesystem::exists(path))
        throw std::runtime_error("Policy: model file not found: " + path);
    return path;
}

OnnxInferenceEngine::OnnxInferenceEngine(const std::string& model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "Policy"),
      session_opts_(),
      session_(env_, checked_path(model_path).c_str(), session_opts_),
      mem_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
    Ort::AllocatorWithDefaultOptions allocator;
    const size_t num_inputs  = session_.GetInputCount();
    const size_t num_outputs = session_.GetOutputCount();

    // Stateful models pair each extra input with an output (state_in/state_out),
    // so the counts must match: 1 obs + N states  <->  1 actions + N states_out.
    if (num_inputs != num_outputs) {
        throw std::runtime_error(
            "OnnxInferenceEngine: inputs and outputs must pair up "
            "(obs+states / actions+states_out), got " +
            std::to_string(num_inputs) + " inputs vs " +
            std::to_string(num_outputs) + " outputs");
    }

    auto last_dim = [](const Ort::TypeInfo& info) {
        return static_cast<int>(
            info.GetTensorTypeAndShapeInfo().GetShape().back());
    };

    // Index 0 is obs/actions; indices 1..N are the recurrent state pairs.
    input_names_.push_back(session_.GetInputNameAllocated(0, allocator).get());
    output_names_.push_back(session_.GetOutputNameAllocated(0, allocator).get());
    input_dim_  = last_dim(session_.GetInputTypeInfo(0));
    output_dim_ = last_dim(session_.GetOutputTypeInfo(0));

    for (size_t i = 1; i < num_inputs; ++i) {
        StateBuffer sb;
        sb.name_in  = session_.GetInputNameAllocated(i, allocator).get();
        sb.name_out = session_.GetOutputNameAllocated(i, allocator).get();
        sb.shape = session_.GetInputTypeInfo(i)
                       .GetTensorTypeAndShapeInfo().GetShape();
        const size_t numel = std::accumulate(
            sb.shape.begin(), sb.shape.end(),
            size_t{1}, std::multiplies<size_t>{});
        sb.data.assign(numel, 0.0f);

        input_names_.push_back(sb.name_in);
        output_names_.push_back(sb.name_out);
        states_.push_back(std::move(sb));
    }

    // Cache c_str() pointers once; the backing strings are never modified again,
    // so these stay valid for every Run() call.
    for (const auto& n : input_names_)  input_name_ptrs_.push_back(n.c_str());
    for (const auto& n : output_names_) output_name_ptrs_.push_back(n.c_str());
}

void OnnxInferenceEngine::reset_state() {
    for (auto& s : states_)
        std::fill(s.data.begin(), s.data.end(), 0.0f);
}

Eigen::VectorXf OnnxInferenceEngine::infer(const Eigen::VectorXf& input)
{
    if (input.size() != input_dim_) {
        throw std::runtime_error(
            "OnnxInferenceEngine::infer: expected input_dim=" +
            std::to_string(input_dim_) + ", got " +
            std::to_string(input.size()));
    }

    // Assemble inputs in declared order: obs first, then each state buffer.
    std::vector<Ort::Value> inputs;
    inputs.reserve(input_name_ptrs_.size());

    std::array<int64_t, 2> obs_shape{1, input_dim_};
    inputs.push_back(Ort::Value::CreateTensor<float>(
        mem_info_, const_cast<float*>(input.data()),
        static_cast<size_t>(input_dim_),
        obs_shape.data(), obs_shape.size()));

    for (auto& s : states_) {
        inputs.push_back(Ort::Value::CreateTensor<float>(
            mem_info_, s.data.data(), s.data.size(),
            s.shape.data(), s.shape.size()));
    }

    auto outputs = session_.Run(
        Ort::RunOptions{nullptr},
        input_name_ptrs_.data(),  inputs.data(),  input_name_ptrs_.size(),
        output_name_ptrs_.data(), output_name_ptrs_.size());

    // Feed each state output back into its buffer for the next step.
    for (size_t i = 0; i < states_.size(); ++i) {
        const float* src = outputs[i + 1].GetTensorData<float>();
        std::copy(src, src + states_[i].data.size(), states_[i].data.begin());
    }

    // Actions are output 0.
    const float* act = outputs[0].GetTensorData<float>();
    return Eigen::Map<const Eigen::VectorXf>(act, output_dim_);
}
