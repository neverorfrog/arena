#include "engines/OnnxInferenceEngine.h"
#include "RobotData.h"
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>

// Identity model lives next to the test sources so PROJECT_ROOT always works.
static std::string model_path() {
    return std::string(PROJECT_ROOT) + "/external/colosseum/models/identity.onnx";
}

TEST(PolicyTest, LoadsModel) {
    OnnxInferenceEngine policy(model_path());
    EXPECT_EQ(policy.input_dim(),    82);
    EXPECT_EQ(policy.output_dim(), 82);
}

TEST(PolicyTest, IdentityForwardPass) {
    OnnxInferenceEngine policy(model_path());

    Eigen::VectorXf obs = Eigen::VectorXf::LinSpaced(policy.input_dim(), 0.f, 1.f);
    Eigen::VectorXf out = policy.infer(obs);

    ASSERT_EQ(out.size(), obs.size());
    float max_err = (out - obs).cwiseAbs().maxCoeff();
    EXPECT_LT(max_err, 1e-6f) << "Identity model output differs from input";
}

TEST(PolicyTest, WrongObsDimThrows) {
    OnnxInferenceEngine policy(model_path());
    Eigen::VectorXf bad_obs(policy.input_dim() + 1);
    EXPECT_THROW(policy.infer(bad_obs), std::runtime_error);
}

// ──────────────────────────────────────────────────────────────────────────────
// RobotData remapping tests
// ──────────────────────────────────────────────────────────────────────────────

// Helper: build a RobotConfig<4> with the given name orderings.
static RobotConfig<4> make_cfg(
    const std::array<std::string, 4>& joint_names,
    const std::array<std::string, 4>& sim_joint_names)
{
    RobotConfig<4> cfg;
    cfg.name            = "test_robot";
    cfg.joint_names     = joint_names;
    cfg.sim_joint_names = sim_joint_names;
    return cfg;
}

// When hardware and sim order are identical, both maps must be the identity.
TEST(RobotDataTest, IdentityMapping) {
    auto cfg = make_cfg({"A", "B", "C", "D"}, {"A", "B", "C", "D"});
    RobotData<4> data(cfg);

    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(data.real2sim[i], i) << "real2sim[" << i << "] should be identity";
        EXPECT_EQ(data.sim2real[i], i) << "sim2real[" << i << "] should be identity";
    }
}

// When sim order is a known permutation of hardware order, verify both maps.
// Hardware: [A, B, C, D]   Sim: [B, D, A, C]   (MuJoCo alphabetical example)
//
// real2sim[i] = index in hardware that holds sim_joint_names[i]:
//   real2sim[0] = index of "B" in hardware = 1
//   real2sim[1] = index of "D" in hardware = 3
//   real2sim[2] = index of "A" in hardware = 0
//   real2sim[3] = index of "C" in hardware = 2
//
// sim2real[i] = index in sim that holds joint_names[i]:
//   sim2real[0] = index of "A" in sim = 2
//   sim2real[1] = index of "B" in sim = 0
//   sim2real[2] = index of "C" in sim = 3
//   sim2real[3] = index of "D" in sim = 1
TEST(RobotDataTest, PermutedMapping) {
    auto cfg = make_cfg({"A", "B", "C", "D"}, {"B", "D", "A", "C"});
    RobotData<4> data(cfg);

    const std::array<int, 4> expected_r2s = {1, 3, 0, 2};
    const std::array<int, 4> expected_s2r = {2, 0, 3, 1};

    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(data.real2sim[i], expected_r2s[i]) << "real2sim[" << i << "]";
        EXPECT_EQ(data.sim2real[i], expected_s2r[i]) << "sim2real[" << i << "]";
    }
}

// The two maps must be inverses of each other: real2sim[sim2real[i]] == i
// and sim2real[real2sim[i]] == i for all i.
TEST(RobotDataTest, MapsAreInverses) {
    auto cfg = make_cfg({"A", "B", "C", "D"}, {"C", "A", "D", "B"});
    RobotData<4> data(cfg);

    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(data.real2sim[data.sim2real[i]], i)
            << "real2sim[sim2real[" << i << "]] != " << i;
        EXPECT_EQ(data.sim2real[data.real2sim[i]], i)
            << "sim2real[real2sim[" << i << "]] != " << i;
    }
}

// Applying real2sim to hardware-order values must produce sim-order values.
TEST(RobotDataTest, RemappingProducesSimOrder) {
    // Hardware: [A=10, B=20, C=30, D=40]
    // Sim order: [B, D, A, C] → expected sim-order values: [20, 40, 10, 30]
    auto cfg = make_cfg({"A", "B", "C", "D"}, {"B", "D", "A", "C"});
    RobotData<4> data(cfg);

    std::array<float, 4> hardware_vals = {10.f, 20.f, 30.f, 40.f};
    std::array<float, 4> expected_sim  = {20.f, 40.f, 10.f, 30.f};

    for (int i = 0; i < 4; i++) {
        EXPECT_FLOAT_EQ(hardware_vals[data.real2sim[i]], expected_sim[i])
            << "sim slot " << i;
    }
}

// A missing sim joint name must throw at construction, not silently misbehave.
TEST(RobotDataTest, UnknownSimJointThrows) {
    auto cfg = make_cfg({"A", "B", "C", "D"}, {"A", "B", "C", "X"});  // "X" not in hardware
    EXPECT_THROW(RobotData<4>{cfg}, std::runtime_error);
}

// A missing hardware joint name must also throw.
TEST(RobotDataTest, UnknownHardwareJointThrows) {
    auto cfg = make_cfg({"A", "B", "C", "X"}, {"A", "B", "C", "D"});  // "X" not in sim
    EXPECT_THROW(RobotData<4>{cfg}, std::runtime_error);
}
