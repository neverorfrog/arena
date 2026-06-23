#pragma once

#include <Eigen/Dense>

class IInferenceEngine {
    public:
        virtual ~IInferenceEngine() = default;
        virtual Eigen::VectorXf infer(const Eigen::VectorXf& input) = 0;
        virtual int input_dim()  const = 0;
        virtual int output_dim() const = 0;

        // Zero any internal recurrent state. Call on episode reset. Default is a
        // no-op; only stateful engines (e.g. RMA window models) override it.
        virtual void reset_state() {}

        // Number of inference steps needed to fill the recurrent state from a
        // cold (zeroed) start. For an RMA observation-window model this is the
        // window depth; running this many warmup steps in place (holding pose,
        // discarding actions) avoids the zero-window startup transient. 0 means
        // no warmup needed (stateless models).
        virtual int warmup_steps() const { return 0; }
};
