#pragma once

#include <Eigen/Dense>

class IInferenceEngine {
    public:
        virtual ~IInferenceEngine() = default;
        virtual Eigen::VectorXf infer(const Eigen::VectorXf& input) = 0;
        virtual int input_dim()  const = 0;
        virtual int output_dim() const = 0;
};
