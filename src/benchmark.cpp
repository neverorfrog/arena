#include "engines/IInferenceEngine.h"
#include "engines/OnnxInferenceEngine.h"
#include "ModelRegistry.h"
#ifdef WITH_TENSORRT
#include "engines/TrtInferenceEngine.h"
#endif

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

using Clock = std::chrono::high_resolution_clock;
using Ms    = std::chrono::duration<double, std::milli>;

struct Stats {
    double min   = 0;
    double max   = 0;
    double mean  = 0;
    double p50   = 0;
    double p95   = 0;
    double p99   = 0;
    double total = 0;
    size_t count = 0;
};

Stats compute_stats(const std::vector<double>& times_ms) {
    Stats s;
    s.count = times_ms.size();
    if (s.count == 0) return s;

    s.total = std::accumulate(times_ms.begin(), times_ms.end(), 0.0);
    s.mean  = s.total / static_cast<double>(s.count);
    s.min   = *std::min_element(times_ms.begin(), times_ms.end());
    s.max   = *std::max_element(times_ms.begin(), times_ms.end());

    std::vector<double> sorted = times_ms;
    std::sort(sorted.begin(), sorted.end());
    auto percentile = [&](double p) {
        size_t idx = static_cast<size_t>(std::ceil(p * 0.01 * sorted.size())) - 1;
        if (idx >= sorted.size()) idx = sorted.size() - 1;
        return sorted[idx];
    };
    s.p50 = percentile(50.0);
    s.p95 = percentile(95.0);
    s.p99 = percentile(99.0);
    return s;
}

void print_stats(const std::string& label, const Stats& s) {
    std::cout << "\n── " << label << " ──\n"
              << "  iterations : " << s.count << "\n"
              << "  total      : " << s.total << " ms\n"
              << "  mean       : " << s.mean << " ms\n"
              << "  min        : " << s.min << " ms\n"
              << "  max        : " << s.max << " ms\n"
              << "  p50        : " << s.p50 << " ms\n"
              << "  p95        : " << s.p95 << " ms\n"
              << "  p99        : " << s.p99 << " ms\n"
              << "  throughput : " << (s.count / (s.total / 1000.0)) << " inf/s\n";
}

std::vector<double> run_benchmark(IInferenceEngine& engine, int input_dim,
                                  int warmup, int iterations) {
    std::vector<double> times;
    times.reserve(iterations);

    Eigen::VectorXf obs(input_dim);
    obs.setRandom();

    // Warm-up
    for (int i = 0; i < warmup; i++)
        engine.infer(obs);

    // Benchmark
    for (int i = 0; i < iterations; i++) {
        auto t0 = Clock::now();
        engine.infer(obs);
        auto t1 = Clock::now();
        times.push_back(std::chrono::duration_cast<Ms>(t1 - t0).count());
    }
    return times;
}

int main(int argc, char** argv) {
    std::string model_path;
    std::string engine_path;
    int warmup     = 100;
    int iterations = 1000;

    for (int i = 0; i < argc; i++) {
        if (std::strcmp(argv[i], "--model")  == 0 && i + 1 < argc) model_path  = argv[++i];
        if (std::strcmp(argv[i], "--engine") == 0 && i + 1 < argc) engine_path = argv[++i];
        if (std::strcmp(argv[i], "--warmup") == 0 && i + 1 < argc) warmup      = std::stoi(argv[++i]);
        if (std::strcmp(argv[i], "--iters")  == 0 && i + 1 < argc) iterations  = std::stoi(argv[++i]);
        if (std::strcmp(argv[i], "--task")   == 0 && i + 1 < argc) {
            if (model_path.empty())
                model_path = ModelRegistry::resolve(argv[++i]).string();
        }
    }

    if (model_path.empty()) {
        std::cerr << "Usage: benchmark --task <name> | --model <path> [--engine <path>]\n"
                  << "              [--warmup 100] [--iters 1000]\n";
        return 1;
    }

    // ── ONNX Runtime ───────────────────────────────────────────────────────
    std::cout << "Benchmark: ONNX model=" << model_path << "\n"
              << "           warmup=" << warmup << " iters=" << iterations << "\n";

    OnnxInferenceEngine onnx(model_path);
    auto onnx_times = run_benchmark(onnx, onnx.input_dim(), warmup, iterations);
    print_stats("ONNX Runtime", compute_stats(onnx_times));

    // ── TensorRT ───────────────────────────────────────────────────────────
#ifdef WITH_TENSORRT
    if (engine_path.empty()) {
        engine_path = model_path;
        auto pos = engine_path.rfind(".onnx");
        if (pos != std::string::npos)
            engine_path.replace(pos, 5, ".engine");
    }
    std::cout << "\nBenchmark: TRT  engine=" << engine_path << "\n";

    TrtInferenceEngine trt(engine_path);
    auto trt_times = run_benchmark(trt, trt.input_dim(), warmup, iterations);
    print_stats("TensorRT", compute_stats(trt_times));

    // ── Speedup ───────────────────────────────────────────────────────────
    double onnx_mean = compute_stats(onnx_times).mean;
    double trt_mean  = compute_stats(trt_times).mean;
    if (onnx_mean > 0)
        std::cout << "\n  TRT speedup: " << (onnx_mean / trt_mean) << "x\n";
#else
    std::cout << "\n  TensorRT not built (build with TensorRT for TRT benchmark)\n";
#endif

    return 0;
}
