#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 通过闭环实验估计实装机构等效惯量、粘性阻尼和库仑摩擦，为云台 LQR、前馈和仿真提供参数依据 / Closed-loop identification of inertia, viscous damping and Coulomb friction for gimbal modeling
standalone: true
depends:
- id: QDU-Robomaster/Motor
  ref: same-or-dev
- id: QDU-Robomaster/RMMotor
  ref: same-or-dev
- id: QDU-Robomaster/DMMotor
  ref: same-or-dev
- id: xrobot-org/BMI088
  ref: same-or-dev
- id: xrobot-org/MadgwickAHRS
  ref: same-or-dev
=== END MANIFEST === */
// clang-format on

#include "MotorIdentifier.hpp"
#include "thread.hpp"

// Instantiate motor, BMI088 and AHRS BEFORE this module in YAML.
// The motor belongs exclusively to this instance. Owns Update/Control/Relax
// on one 2 ms thread. Instantiation starts the thread; command_ stays zero.
class SysID : public SysId::MotorIdentifier {
public:
  // Route the unmodified motor driver through this bus in YAML.
  static LibXR::CAN &Observe(LibXR::CAN &physical) {
    return SysId::CanCommandObserver::Bus(physical);
  }
  struct Param {
    uint32_t feedback_id = 0;
    float feedback_torque_sign = 1.0f;
    const char *gyro_topic_name = "bmi088_gyro";
    const char *euler_topic_name = "ahrs_euler";
    uint32_t gyro_axis = 2;
    float gyro_sign = 1.0f;
    float pitch_zero = 0.0f;
    float min_position = -1000000.0f;
    float max_position = 1000000.0f;
    float max_travel = .06f;
    float amplitude = .07f;
    float kp = 2.0f;
    float kd = .12f;
    float torque_limit = .20f;
    uint32_t tx_id = 0;
    uint32_t tx_byte_offset = 0;
    SysId::CanCommandObserver::Encoding tx_encoding =
        SysId::CanCommandObserver::Encoding::SIGNED16;
    float tx_torque_scale = 1.0f;
    float tx_torque_bias = 0.0f;
    float torque_constant = 0.0f;
    uint32_t seed = 0xACE1;
    uint32_t task_stack_depth = 2048;
    SysId::FrictionSweep::Param sweep = {};
  };

  // motor/can_bus refer to an existing motor and its actual CAN bus.
  // Topics: Eigen::Matrix<float,3,1> in rad/s and EulerAngle<float> in rad.
  // Axis/sign apply AFTER mounting rotation. Fix base/big yaw and pitch near
  // calibrated zero: an arbitrary body-axis gyro component is not yaw rate.
  SysID(Motor &motor, LibXR::CAN &can_bus, const Param &param)
      : controlled_motor_(motor) {
    const bool MOTOR_BOUND =
        Bind(motor, can_bus, param.feedback_id, param.feedback_torque_sign);
    const bool IMU_BOUND =
        BindImu(param.gyro_topic_name, param.euler_topic_name, param.gyro_axis,
                param.gyro_sign, param.pitch_zero);
    const bool COMMAND_BOUND =
        ConfigureCommand({.tx_id = param.tx_id,
                          .byte_offset = param.tx_byte_offset,
                          .encoding = param.tx_encoding,
                          .torque_scale = param.tx_torque_scale,
                          .torque_bias = param.tx_torque_bias},
                         param.torque_constant);
    if (!MOTOR_BOUND || !IMU_BOUND || !COMMAND_BOUND ||
        param.task_stack_depth < 1024) {
      Cancel(Status::BAD_INPUT);
      controlled_motor_.Relax();
      return;
    }
    min_position_ = param.min_position;
    max_position_ = param.max_position;
    max_travel_ = param.max_travel;
    amplitude_ = param.amplitude;
    kp_ = param.kp;
    kd_ = param.kd;
    torque_limit_ = param.torque_limit;
    seed_ = param.seed;
    sweep_param_ = param.sweep;
    command_ = 0;
    controlled_motor_.Relax();
    ready_ = true;
    thread_.Create(this, ThreadFunc, "sysid", param.task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  volatile bool ready_ = false;

private:
  static void ThreadFunc(SysID *self) {
    for (;;) {
      if (!self->Poll()) {
        self->controlled_motor_.Update();
        self->controlled_motor_.Relax();
      }
      LibXR::Thread::Sleep(2);
    }
  }
  Motor &controlled_motor_;
  LibXR::Thread thread_;
};
