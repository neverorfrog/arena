#include "engines/OnnxInferenceEngine.h"

#include <filesystem>
#include <numeric>
#include <stdexcept>
#include <string>

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

    if (num_inputs != num_outputs) {
        throw std::runtime_error(
            "OnnxInferenceEngine: expected equal number of inputs and outputs "
            "(obs+states / actions+states_out), got " +
            std::to_string(num_inputs) + " inputs and " +
            std::to_string(num_outputs) + " outputs"
        );
    }

    // Input 0: obs
    input_name_ = session_.GetInputNameAllocated(0, allocator).get();
    input_dim_  = static_cast<int>(
        session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().back());

    // Output 0: actions
    output_name_ = session_.GetOutputNameAllocated(0, allocator).get();
    output_dim_  = static_cast<int>(
        session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape().back());

    // Inputs 1..N / Outputs 1..N: paired state tensors
    for (size_t i = 1; i < num_inputs; ++i) {
        StateBuffer sb;
        sb.input_name  = session_.GetInputNameAllocated(i, allocator).get();
        sb.output_name = session_.GetOutputNameAllocated(i, allocator).get();
        sb.shape = session_.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
        const size_t numel = static_cast<size_t>(
            std::accumulate(sb.shape.begin(), sb.shape.end(),
                            int64_t{1}, std::multiplies<int64_t>{}));
        sb.data.assign(numel, 0.0f);
        state_bufs_.push_back(std::move(sb));
    }

    // Build name arrays after all strings are final (c_str() must stay stable)
    all_input_names_.push_back(input_name_);
    all_output_names_.push_back(output_name_);
    for (const auto& sb : state_bufs_) {
        all_input_names_.push_back(sb.input_name);
        all_output_names_.push_back(sb.output_name);
    }
    for (const auto& n : all_input_names_)  input_name_ptrs_.push_back(n.c_str());
    for (const auto& n : all_output_names_) output_name_ptrs_.push_back(n.c_str());
}

void OnnxInferenceEngine::reset_state() {
    for (auto& sb : state_bufs_)
        std::fill(sb.data.begin(), sb.data.end(), 0.0f);
}

Eigen::VectorXf OnnxInferenceEngine::infer(const Eigen::VectorXf& input)
{
    if (input.size() != input_dim_) {
        throw std::runtime_error(
            "OnnxInferenceEngine::infer: expected obs_dim=" +
            std::to_string(input_dim_) + ", got " + std::to_string(input.size()));
    }

    std::vector<Ort::Value> input_tensors;
    input_tensors.reserve(1 + state_bufs_.size());

    // obs tensor — (1, obs_dim)
    std::array<int64_t, 2> obs_shape{1, input_dim_};
    input_tensors.push_back(Ort::Value::CreateTensor<float>(
        mem_info_,
        const_cast<float*>(input.data()),
        static_cast<size_t>(input_dim_),
        obs_shape.data(), obs_shape.size()
    ));

    // State tensors
    for (auto& sb : state_bufs_) {
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            mem_info_,
            sb.data.data(), sb.data.size(),
            sb.shape.data(), sb.shape.size()
        ));
    }

    auto output_tensors = session_.Run(
        Ort::RunOptions{nullptr},
        input_name_ptrs_.data(),  input_tensors.data(),  input_name_ptrs_.size(),
        output_name_ptrs_.data(), output_name_ptrs_.size()
    );

    // Update state buffers from outputs 1..N
    for (size_t i = 0; i < state_bufs_.size(); ++i) {
        const float* src = output_tensors[i + 1].GetTensorData<float>();
        std::copy(src, src + state_bufs_[i].data.size(), state_bufs_[i].data.begin());
    }

    // Return actions from output 0
    const float* act = output_tensors[0].GetTensorData<float>();
    return Eigen::Map<const Eigen::VectorXf>(act, output_dim_);
}
