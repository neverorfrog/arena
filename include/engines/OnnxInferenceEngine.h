#pragma once

#include <string>
#include <vector>
#include "IInferenceEngine.h"
#include <Eigen/Dense>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

// ONNX Runtime inference engine.
//
// Supports both stateless models (obs → actions) and stateful RMA models
// (obs + state_in... → actions + state_out...).
//
// For stateful models, state buffers are allocated at construction (zeroed)
// and automatically updated each infer() call from the corresponding outputs.
// The caller does not need to know how many state tensors exist.
//
// I/O layout convention (must match the Python export):
//   inputs:  obs, [state_0, state_1, ...]
//   outputs: actions, [state_0_out, state_1_out, ...]
class OnnxInferenceEngine : public IInferenceEngine {
public:
    explicit OnnxInferenceEngine(const std::string& model_path);

    // Forward pass. obs must have exactly input_dim() elements.
    // Returns output_dim() action values.
    Eigen::VectorXf infer(const Eigen::VectorXf& input) override;

    // Zero all state buffers (call on episode reset for stateful models).
    void reset_state();

    int input_dim()  const override { return input_dim_; }
    int output_dim() const override { return output_dim_; }

private:
    Ort::Env            env_;
    Ort::SessionOptions session_opts_;
    Ort::Session        session_;
    Ort::MemoryInfo     mem_info_;

    // Primary I/O (obs / actions)
    std::string input_name_;
    std::string output_name_;
    int input_dim_;
    int output_dim_;

    // Per-state-tensor bookkeeping for stateful (RMA) models
    struct StateBuffer {
        std::string       input_name;
        std::string       output_name;
        std::vector<int64_t> shape;
        std::vector<float>   data;   // zero-initialised, updated each step
    };
    std::vector<StateBuffer> state_bufs_;

    // Flat name arrays for ORT Run() — must outlive Run() calls
    std::vector<std::string>  all_input_names_;
    std::vector<std::string>  all_output_names_;
    std::vector<const char*>  input_name_ptrs_;
    std::vector<const char*>  output_name_ptrs_;
};
