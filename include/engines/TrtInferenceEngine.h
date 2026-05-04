#pragma once

#include "IInferenceEngine.h"

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <string>

// TensorRT inference engine — loads a serialized .engine file (built offline by
// trtexec from an ONNX model) and runs single-sample float32 inference on GPU.
//
// Expected model I/O shape:
//   input  — float32[1, observation_dim]   (flattened observation vector)
//   output — float32[1, action_dim]         (raw policy actions)
//
// Assumes static shapes (no dynamic batch dimension). Uses names-based API for
// TRT >= 10 and binding-index API for older versions, matching the pattern in
// spqrbooster2026/YOLOEngine.
class TrtInferenceEngine : public IInferenceEngine {
public:
    explicit TrtInferenceEngine(const std::string& engine_path);
    ~TrtInferenceEngine() override;

    TrtInferenceEngine(const TrtInferenceEngine&) = delete;
    TrtInferenceEngine& operator=(const TrtInferenceEngine&) = delete;

    Eigen::VectorXf infer(const Eigen::VectorXf& input) override;

    int input_dim()  const override { return input_dim_; }
    int output_dim() const override { return output_dim_; }

private:
    void loadEngine(const std::string& engine_path);
    void allocateBuffers();

    nvinfer1::IRuntime*          runtime_ = nullptr;
    nvinfer1::ICudaEngine*       engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;
    cudaStream_t                  stream_ = nullptr;

#if NV_TENSORRT_MAJOR >= 10
    std::string input_name_;
    std::string output_name_;
#else
    int input_index_ = -1;
    int output_index_ = -1;
#endif

    int input_dim_ = 0;
    int output_dim_ = 0;

    void*  gpu_input_    = nullptr;
    void*  gpu_output_   = nullptr;
    float* host_input_   = nullptr;
    float* host_output_  = nullptr;

    size_t input_bytes_  = 0;
    size_t output_bytes_ = 0;
};
