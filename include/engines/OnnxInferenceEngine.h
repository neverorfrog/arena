#pragma once

#include <string>
#include <vector>
#include "IInferenceEngine.h"
#include <Eigen/Dense>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

// ONNX Runtime inference engine.
//
// Handles two model shapes through a single code path:
//   stateless:  obs                  ->  actions
//   stateful:   obs, state_0, ...    ->  actions, state_0_out, ...
//
// "Stateful" is how the RMA policy is exported: its adaptation encoder carries a
// rolling window of the last N observations as a recurrent tensor. Every input
// after "obs" is treated as such state — its buffer lives here, zero-initialised,
// is fed in each step, and is overwritten from the matching output. The caller
// only ever passes obs and reads actions; it never sees the window.
//
// I/O order must match the Python ONNX export:
//   inputs:  obs,     [state_0,     state_1,     ...]
//   outputs: actions, [state_0_out, state_1_out, ...]
//
// The model runs on CPU. I/O names come from the model metadata and are kept
// alive as std::string members (ORT requires const char* that outlive Run()).
class OnnxInferenceEngine : public IInferenceEngine {
public:
    explicit OnnxInferenceEngine(const std::string& model_path);

    // Forward pass. `input` must have input_dim() elements; returns output_dim().
    Eigen::VectorXf infer(const Eigen::VectorXf& input) override;

    // Zero the recurrent state buffers (call on episode reset). No-op when the
    // model is stateless.
    void reset_state() override;

    int input_dim()  const override { return input_dim_; }
    int output_dim() const override { return output_dim_; }
    int warmup_steps() const override { return warmup_steps_; }

private:
    Ort::Env            env_;
    Ort::SessionOptions session_opts_;
    Ort::Session        session_;
    Ort::MemoryInfo     mem_info_;

    int input_dim_;   // "obs" width
    int output_dim_;  // "actions" width
    int warmup_steps_ = 0;  // recurrent-window depth (0 if stateless)

    // One entry per recurrent state tensor (empty for stateless models). `data`
    // is fed in as input `name_in` and refilled from output `name_out` each step.
    struct StateBuffer {
        std::string          name_in;
        std::string          name_out;
        std::vector<int64_t> shape;
        std::vector<float>   data;   // zero-initialised, updated each step
    };
    std::vector<StateBuffer> states_;

    // Name arrays passed to Ort::Run(), in declared order:
    //   input_names_  = [obs,     state_0,     ...]
    //   output_names_ = [actions, state_0_out, ...]
    // The const char* arrays point into these strings, which are fixed after
    // construction, so the pointers stay valid across every Run().
    std::vector<std::string> input_names_;
    std::vector<std::string> output_names_;
    std::vector<const char*> input_name_ptrs_;
    std::vector<const char*> output_name_ptrs_;
};
