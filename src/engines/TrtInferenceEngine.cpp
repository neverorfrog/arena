#include "engines/TrtInferenceEngine.h"

#ifdef WITH_TENSORRT

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

class Logger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cerr << "[TensorRT] " << msg << std::endl;
    }
};

Logger logger;

size_t volume(const nvinfer1::Dims& d) {
    size_t v = 1;
    for (int i = 0; i < d.nbDims; i++)
        v *= static_cast<size_t>(d.d[i]);
    return v;
}

}  // namespace

TrtInferenceEngine::TrtInferenceEngine(const std::string& engine_path) {
    loadEngine(engine_path);
}

TrtInferenceEngine::~TrtInferenceEngine() {
    if (host_output_)
        cudaFreeHost(host_output_);
    if (host_input_)
        cudaFreeHost(host_input_);
    if (gpu_output_)
        cudaFree(gpu_output_);
    if (gpu_input_)
        cudaFree(gpu_input_);
    if (stream_)
        cudaStreamDestroy(stream_);
    if (context_)
        delete context_;
    if (engine_)
        delete engine_;
    if (runtime_)
        delete runtime_;
}

void TrtInferenceEngine::loadEngine(const std::string& engine_path) {
    std::ifstream input(engine_path, std::ios::binary);
    if (!input) {
        throw std::runtime_error(
            "TrtInferenceEngine: cannot open engine file: " + engine_path);
    }
    input.seekg(0, input.end);
    size_t size = input.tellg();
    input.seekg(0, input.beg);
    std::vector<char> data(size);
    input.read(data.data(), static_cast<std::streamsize>(size));

    runtime_ = nvinfer1::createInferRuntime(logger);
    if (!runtime_)
        throw std::runtime_error("TrtInferenceEngine: failed to create InferRuntime");

    engine_ = runtime_->deserializeCudaEngine(data.data(), size);
    if (!engine_)
        throw std::runtime_error("TrtInferenceEngine: failed to deserialize engine");

    context_ = engine_->createExecutionContext();
    if (!context_)
        throw std::runtime_error("TrtInferenceEngine: failed to create execution context");

    cudaStreamCreate(&stream_);

    // Discover input/output tensors and their dimensions
#if NV_TENSORRT_MAJOR >= 10
    int nb = engine_->getNbIOTensors();
    for (int i = 0; i < nb; i++) {
        const char* name = engine_->getIOTensorName(i);
        if (!name)
            continue;
        if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT &&
            input_name_.empty())
            input_name_ = name;
        else if (engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT &&
                 output_name_.empty())
            output_name_ = name;
    }
    if (input_name_.empty() || output_name_.empty())
        throw std::runtime_error("TrtInferenceEngine: could not find input/output tensors");

    nvinfer1::Dims input_dims  = engine_->getTensorShape(input_name_.c_str());
    nvinfer1::Dims output_dims = engine_->getTensorShape(output_name_.c_str());
#else
    int nb = engine_->getNbBindings();
    for (int i = 0; i < nb; i++) {
        if (engine_->bindingIsInput(i) && input_index_ < 0)
            input_index_ = i;
        else if (!engine_->bindingIsInput(i) && output_index_ < 0)
            output_index_ = i;
    }
    if (input_index_ < 0 || output_index_ < 0)
        throw std::runtime_error("TrtInferenceEngine: could not find input/output bindings");

    nvinfer1::Dims input_dims  = engine_->getBindingDimensions(input_index_);
    nvinfer1::Dims output_dims = engine_->getBindingDimensions(output_index_);
#endif

    input_dim_  = static_cast<int>(volume(input_dims));
    output_dim_ = static_cast<int>(volume(output_dims));

    allocateBuffers();
}

void TrtInferenceEngine::allocateBuffers() {
    input_bytes_  = static_cast<size_t>(input_dim_)  * sizeof(float);
    output_bytes_ = static_cast<size_t>(output_dim_) * sizeof(float);

    if (gpu_input_)
        cudaFree(gpu_input_);
    if (gpu_output_)
        cudaFree(gpu_output_);
    if (host_input_)
        cudaFreeHost(host_input_);
    if (host_output_)
        cudaFreeHost(host_output_);

    cudaMalloc(&gpu_input_,  input_bytes_);
    cudaMalloc(&gpu_output_, output_bytes_);
    cudaMallocHost(reinterpret_cast<void**>(&host_input_),  input_bytes_);
    cudaMallocHost(reinterpret_cast<void**>(&host_output_), output_bytes_);
}

Eigen::VectorXf TrtInferenceEngine::infer(const Eigen::VectorXf& input) {
    if (input.size() != input_dim_) {
        throw std::runtime_error(
            "TrtInferenceEngine::infer: expected input_dim=" +
            std::to_string(input_dim_) +
            ", got " + std::to_string(input.size()));
    }

    // Copy observation to pinned host buffer
    std::copy(input.data(), input.data() + input_dim_, host_input_);

    // Host → Device
    cudaMemcpyAsync(gpu_input_, host_input_, input_bytes_,
                    cudaMemcpyHostToDevice, stream_);

    // Run inference
#if NV_TENSORRT_MAJOR >= 10
    context_->setTensorAddress(input_name_.c_str(), gpu_input_);
    context_->setTensorAddress(output_name_.c_str(), gpu_output_);
    bool ok = context_->enqueueV3(stream_);
#else
    std::vector<void*> bindings(engine_->getNbBindings(), nullptr);
    bindings[input_index_]  = gpu_input_;
    bindings[output_index_] = gpu_output_;

    bool ok;
    if (engine_->hasImplicitBatchDimension())
        ok = context_->enqueue(1, bindings.data(), stream_, nullptr);
    else
        ok = context_->enqueueV2(bindings.data(), stream_, nullptr);
#endif

    if (!ok)
        throw std::runtime_error("TrtInferenceEngine: inference failed");

    // Device → Host
    cudaMemcpyAsync(host_output_, gpu_output_, output_bytes_,
                    cudaMemcpyDeviceToHost, stream_);
    cudaStreamSynchronize(stream_);

    return Eigen::Map<Eigen::VectorXf>(host_output_, output_dim_);
}

#endif  // WITH_TENSORRT
