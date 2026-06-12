#pragma once

#include <string>
#include <vector>
#include "IInferenceEngine.h"
#include <Eigen/Dense>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>

// Thin wrapper around an ONNX Runtime inference session.
//
// Loads a single-input / single-output ONNX model at construction time and
// caches the I/O names and tensor dimensions so each forward pass only
// allocates the minimum required memory.
//
// Expected model shape:
//   input  — float32[1, obs_dim]   (flattened observation vector)
//   output — float32[1, act_dim]   (raw policy actions, before scaling)
//
// The model is run on CPU. Input/output names are read from the model
// metadata and kept alive as std::string members (ORT requires const char*
// that outlive the Run() call).
class OnnxInferenceEngine : public IInferenceEngine {
public:
    explicit OnnxInferenceEngine(const std::string& model_path);

    // Forward pass. obs must have exactly obs_dim() elements.
    // Returns action_dim() joint position targets in simulation order.
    Eigen::VectorXf infer(const Eigen::VectorXf& input);

    int input_dim()    const { return input_dim_; }
    int output_dim() const { return output_dim_; }

private:
    Ort::Env            env_;
    Ort::SessionOptions session_opts_;
    Ort::Session        session_;
    Ort::MemoryInfo     mem_info_;

    // Cached I/O names (ORT requires const char* that outlives Run())
    std::string input_name_;
    std::string output_name_;

    int input_dim_;
    int output_dim_;
};
