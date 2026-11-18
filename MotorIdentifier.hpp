#pragma once

#include "CanCommandObserver.hpp"
#include "DMMotor.hpp"
#include "Experiment.hpp"
#include "ImuFeedback.hpp"
#include "Motor.hpp"
#include "RMMotor.hpp"
#include "timebase.hpp"

namespace SysId {
// Restored experiment; motor/CAN/IMU adapter remains independent of Gimbal.
class MotorIdentifier : public Experiment {
public:
  bool Bind(Motor &motor, LibXR::CAN &bus, uint32_t feedback_id,
            float feedback_torque_sign = 1.0f) {
    if (motor_ || feedback_id > 0x7ff ||
        (feedback_torque_sign != 1 && feedback_torque_sign != -1))
      return false;
    motor_ = &motor;
    feedback_torque_sign_ = feedback_torque_sign;
    observer_ = &CanCommandObserver::Bus(bus);
    auto cb = LibXR::CAN::Callback::Create(
        [](bool, MotorIdentifier *self, const LibXR::CAN::ClassicPack &pack) {
          if (pack.dlc != 8)
            return;
          self->last_rx_us_ =
              static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
          self->rx_count_ = self->rx_count_ + 1;
        },
        this);
    bus.Register(cb, LibXR::CAN::Type::STANDARD,
                 LibXR::CAN::FilterMode::ID_RANGE, feedback_id, feedback_id);
    return true;
  }
  bool ConfigureCommand(const CanCommandObserver::Param &p,
                        float torque_constant) {
    if (!observer_ || !observer_->Configure(p) ||
        !std::isfinite(torque_constant) || torque_constant < 0)
      return false;
    command_quantum_ = std::fabs(p.torque_scale);
    torque_constant_ = torque_constant;
    return true;
  }
  bool BindImu(const char *gyro_topic, const char *euler_topic,
               uint32_t axis = 2, float sign = 1, float pitch_zero = 0) {
    return imu_.Bind(gyro_topic, euler_topic, axis, sign, pitch_zero);
  }
  // 0 stop/idle; 1 PRBS; 2 multi-speed b/fc sweep. Never starts on boot.
  volatile uint32_t command_ = 0;
  volatile float omega_imu_ = 0, omega_motor_ = 0, pitch_imu_ = 0, theta_ = 0;
  bool Busy() const { return session_active_ || command_ != 0; }
  bool Poll() {
    const uint64_t NOW = LibXR::Timebase::GetMicroseconds();
    float dt =
        last_poll_us_ ? static_cast<float>(NOW - last_poll_us_) * 1e-6f : .002f;
    last_poll_us_ = NOW;
    imu_.Update(NOW);
    interval_measurements_ = true;
    gyro_area_ = imu_.gyro_area_;
    gyro_stamp_us_ = imu_.GyroTimestamp();
    omega_imu_ = imu_.omega_;
    pitch_imu_ = imu_.pitch_;
    const uint32_t COMMAND = command_;
    if (!session_active_ && COMMAND == 0)
      return false;
    if (!motor_ || !observer_)
      return Finish(BAD_INPUT);
    if (COMMAND == 0)
      return Finish(MANUAL_STOP);
    if (COMMAND != 1 && COMMAND != 2)
      return Finish(INVALID_COMMAND);
    if (!session_active_) {
      run_command_ = COMMAND;
      test_mode_ = COMMAND == 1 ? TestMode::PRBS : TestMode::FRICTION;
      session_active_ = true;
      start_us_ = NOW;
      last_enable_us_ = 0;
      dt = .002f;
      enable_ = true;
    }
    if (COMMAND != run_command_)
      return Finish(INVALID_COMMAND);
    const uint32_t RX = rx_count_;
    const bool MOTOR_NEW = RX != last_rx_count_;
    motor_->Update();
    last_rx_count_ = RX;
    const auto &FB = motor_->GetFeedback();
    omega_motor_ = FB.omega;
    if (!imu_.Fresh(static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds())))
      return Finish(IMU_OFFLINE);
    const uint32_t STAMP = last_rx_us_;
    // Load clock AFTER stamp: a CAN ISR can run after loop-start NOW.
    const uint32_t CHECK_NOW =
        static_cast<uint32_t>(LibXR::Timebase::GetMicroseconds());
    const bool FRESH =
        STAMP && static_cast<uint32_t>(CHECK_NOW - STAMP) <= 50000;
    if (!FRESH || FB.state != 1) {
      if (Active() || NOW - start_us_ > 5000000)
        return Finish(MOTOR_OFFLINE);
      if (FRESH && (last_enable_us_ == 0 || NOW - last_enable_us_ >= 200000)) {
        motor_->Enable();
        last_enable_us_ = NOW;
      }
      return true;
    }
    const Input IN{dt,
                   FB.position,
                   imu_.omega_,
                   FB.omega,
                   FB.torque * feedback_torque_sign_,
                   imu_.pitch_,
                   -imu_.gyro_y_,
                   FB.temp,
                   true,
                   true,
                   MOTOR_NEW,
                   imu_.gyro_new_};
    const float OUTPUT = Step(IN);
    theta_ = FB.position - capture_.header.center;
    if (done_)
      return Finish(status_);
    const uint32_t PREVIOUS = observer_->Count();
    motor_->Control(
        Motor::MotorCmd{.mode = Motor::MODE_TORQUE, .torque = OUTPUT});
    if (observer_->Count() == PREVIOUS)
      return Finish(TX_NOT_OBSERVED);
    if (!observer_->Queued())
      return Finish(TX_QUEUE_FAILED);
    // Read the exact torque represented by the driver's submitted frame.
    CommandQueued(observer_->Torque());
    return true;
  }

private:
  bool Finish(Status reason) {
    Cancel(reason);
    command_ = 0;
    session_active_ = false;
    if (motor_)
      motor_->Relax();
    return true;
  }
  Motor *motor_ = nullptr;
  CanCommandObserver *observer_ = nullptr;
  ImuFeedback imu_;
  float feedback_torque_sign_ = 1;
  bool session_active_ = false;
  volatile uint32_t last_rx_us_ = 0, rx_count_ = 0;
  uint32_t last_rx_count_ = 0, run_command_ = 0;
  uint64_t last_poll_us_ = 0, start_us_ = 0, last_enable_us_ = 0;
};
} // namespace SysId
