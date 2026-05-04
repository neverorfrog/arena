#pragma once
#include "inputs/IInputSource.h"
#include "engines/IInferenceEngine.h"
#include "engines/OnnxInferenceEngine.h"
#ifdef WITH_TENSORRT
#include "engines/TrtInferenceEngine.h"
#endif
#include "RobotData.h"
#include "RobotState.h"
#include "TaskConfig.h"
#include <memory>

inline std::unique_ptr<IInferenceEngine> make_engine(const TaskConfig& cfg) {
    if (cfg.inference_backend == "trt") {
#ifdef WITH_TENSORRT
        std::string engine_path = cfg.model_path;
        auto pos = engine_path.rfind(".onnx");
        if (pos != std::string::npos)
            engine_path.replace(pos, 5, ".engine");
        return std::make_unique<TrtInferenceEngine>(engine_path);
#else
        throw std::runtime_error(
            "TensorRT backend not available (build without TensorRT)");
#endif
    }
    return std::make_unique<OnnxInferenceEngine>(cfg.model_path);
}

// Abstract base class for task-specific deployment policies.
//
// A Policy owns the inference engine, the input source, and drives the
// inference loop. Subclasses implement build_observation() to match training,
// and optionally override update_input() to read from input_source_ and update
// task-specific state (e.g. a VelocityCommand in T1VelocityFlat).
//
// Execution flow per step:
//   1. reset()           — zero last_action before the first step
//   2. get_action(state):
//      a. update_input()        — read input_source_, update task state
//      b. build_observation()   — fill `observation` from state + task state
//      c. engine_->infer()      — forward pass
//      d. decode action         — raw * action_scale[i] + default_joint_pos[i]
class Policy {
public:
    explicit Policy(TaskConfig cfg)
        : config_(std::move(cfg)),
          engine_(make_engine(config_)) {
        observation.reserve(engine_->input_dim());
    }

    virtual ~Policy() {
        if (input_source_) input_source_->stop();
    }

    // Zero last_action so the first observation does not carry stale values.
    void reset() {
        std::fill(std::begin(last_action), std::end(last_action), 0.0f);
    }

    // Run one inference step. Returns joint position targets in hardware order.
    std::array<float, TaskConfig::NUM_JOINTS> get_action(const RobotState& state) {
        update_input();
        build_observation(state);
        Eigen::VectorXf obs_vec = Eigen::Map<Eigen::VectorXf>(observation.data(), observation.size());
        Eigen::VectorXf action_vec = engine_->infer(obs_vec);

        // Store last_action in sim order (used by build_observation next step).
        for (int i = 0; i < TaskConfig::NUM_JOINTS; i++)
            last_action[i] = action_vec[i];

        // Decode: network outputs in sim order → remap to hardware order.
        // action[i] = net_out[sim2real[i]] * scale + default_joint_pos[i]
        // For T1, sim2real is identity so this is a no-op permutation.
        std::array<float, TaskConfig::NUM_JOINTS> action{};
        for (int i = 0; i < TaskConfig::NUM_JOINTS; i++) {
            int s = robot_data_.sim2real[i];
            action[i] = action_vec[s] * config_.action_scale + config_.robot.default_joint_pos[i];
        }
        return action;
    }

    const TaskConfig& config() const { return config_; }

protected:
    TaskConfig config_;
    RobotData<TaskConfig::NUM_JOINTS> robot_data_{config_.robot};
    float last_action[TaskConfig::NUM_JOINTS]{};  // in sim order
    std::vector<float> observation;
    std::unique_ptr<IInferenceEngine> engine_;
    std::unique_ptr<IInputSource> input_source_;

    // Called once per step before build_observation(). Default: no-op.
    // Override to read from input_source_ and update task-specific state
    // (e.g. a VelocityCommand).
    virtual void update_input() {}

    // Subclass fills `observation` to match the training observation layout.
    virtual void build_observation(const RobotState& state) = 0;
};
