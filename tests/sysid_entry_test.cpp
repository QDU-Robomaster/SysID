#include "SysID.hpp"
#include <cassert>
#include <iostream>
struct FakeMotor : Motor {
  Feedback feedback;
  unsigned controls = 0, relaxes = 0;
  LibXR::ErrorCode Update() override { return LibXR::ErrorCode::OK; }
  const Feedback &GetFeedback() override { return feedback; }
  void Control(const MotorCmd &) override { ++controls; }
  void Relax() override { ++relaxes; }
  void Enable() override { feedback.state = 1; }
};
int main() {
  FakeMotor motor;
  LibXR::CAN can;
  SysID::Param p;
  p.feedback_id = 0x205;
  p.feedback_torque_sign = -1;
  p.gyro_sign = -1;
  p.min_position = -2;
  p.max_position = -1;
  p.amplitude = .08f;
  p.tx_id = 0x1fe;
  p.tx_torque_scale = -.741f * 3 / 16384;
  p.kp = 3;
  p.seed = 123;
  SysID entry(motor, can, p);
  assert(entry.ready_ && LibXR::Thread::created == 1);
  assert(LibXR::Thread::stack == 2048);
  assert(entry.command_ == 0 && !entry.Busy() && motor.controls == 0);
  assert(motor.relaxes == 1 && entry.min_position_ == -2 &&
         entry.max_position_ == -1);
  assert(entry.amplitude_ == .08f && entry.kp_ == 3 && entry.seed_ == 123);
  assert(entry.sweep_param_.speed4 == .5f && entry.sweep_param_.speed_ki == 30);
  p.feedback_id = 0x800;
  SysID invalid(motor, can, p);
  assert(!invalid.ready_ && LibXR::Thread::created == 1);
  assert(invalid.status_ == SysId::Experiment::Status::BAD_INPUT);
  std::cout << "PASS: YAML entry parameters, automatic thread creation, idle "
               "default, bad binding.\n";
}
