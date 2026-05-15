#pragma once

// Pure sensor state. Contains only data that comes from the robot's sensors or
// is derived from them. Task-specific data (velocity commands, goal positions,
// etc.) belongs in the policy or portal, not here.
struct RobotState {
    // From LowState.imu_state
    float rpy[3];              // roll, pitch, yaw (rad)
    float gyro[3];             // angular velocity in body frame (rad/s)
    float acc[3];              // linear acceleration (m/s²)

    // From LowState.motor_state_serial
    float joint_pos[23];       // joint positions (rad)
    float joint_vel[23];       // joint velocities (rad/s)
    float feedback_torque[23]; // estimated torques (Nm)

    // Derived from IMU
    float projected_gravity[3]; // gravity vector in body frame
    float root_quat[4];         // orientation quaternion (w,x,y,z)

    // Derived — base linear velocity in body frame
    float base_lin_vel[3] = {0, 0, 0};
};
