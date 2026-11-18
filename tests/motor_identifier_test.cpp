#include "MotorIdentifier.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
struct FakeMotor : Motor {
  Feedback fb;
  LibXR::CAN &bus;
  LibXR::CAN *interrupt_can = nullptr;
  float output = 0;
  unsigned relaxes = 0, enables = 0;
  explicit FakeMotor(LibXR::CAN &c) : bus(c) {}
  LibXR::ErrorCode Update() override {
    if (interrupt_can) {
      LibXR::Timebase::now += 10;
      interrupt_can->Receive();
    }
    return LibXR::ErrorCode::OK;
  }
  const Feedback &GetFeedback() override { return fb; }
  void Control(const MotorCmd &cmd) override {
    output = cmd.torque;
    const int16_t RAW =
        static_cast<int16_t>(-cmd.torque / .741f / 3.0f * 16384.0f);
    LibXR::CAN::ClassicPack p;
    p.id = 0x1fe;
    p.data[0] = static_cast<uint8_t>((RAW >> 8) & 255);
    p.data[1] = static_cast<uint8_t>(RAW & 255);
    bus.AddMessage(p);
  }
  void Relax() override {
    ++relaxes;
    Control({MODE_TORQUE, 0});
  }
  void Enable() override {
    ++enables;
    fb.state = 1;
  }
};
using Status = SysId::Experiment::Status;
using Gyro = LibXR::Topic::ASyncSubscriber<Eigen::Matrix<float, 3, 1>>;
using Euler = LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>>;
void Imu(float z = 0, float pitch = 0) {
  Eigen::Matrix<float, 3, 1> g{};
  g(2) = z;
  LibXR::EulerAngle<float> e{};
  e.pitch = pitch;
  Gyro::Publish(g);
  Euler::Publish(e);
}
int main() {
  LibXR::CAN physical;
  auto &observed = SysId::CanCommandObserver::Bus(physical);
  FakeMotor motor(observed);
  motor.fb.position = -1.5;
  SysId::MotorIdentifier id;
  assert(id.Bind(motor, physical, 0x205, -1));
  assert(id.BindImu("gyro", "euler", 2, -1));
  const float Q = .741f * 3 / 16384;
  assert(id.ConfigureCommand(
      {0x1fe, 0, SysId::CanCommandObserver::Encoding::SIGNED16, -Q, 0}, .741f));
  assert(!id.Poll());
  id.command_ = 1;
  LibXR::Timebase::now += 1000000;
  Imu();
  physical.Receive();
  id.Poll();
  assert(motor.enables == 1);
  LibXR::Timebase::now += 2000;
  Imu();
  physical.Receive();
  id.Poll();
  assert(id.status_ == Status::RUNNING);
  for (int n = 0; n < 1100; ++n) {
    LibXR::Timebase::now += 2000;
    Imu();
    physical.Receive();
    id.Poll();
  }
  assert(motor.output != 0 && id.capture_.header.version == 3);
  assert(std::fabs(id.tau_cmd_ - observed.Torque()) < 1e-7f);
  assert(id.capture_.samples[1].stage == 0);
  id.command_ = 0;
  id.Poll();
  assert(id.status_ == Status::MANUAL_STOP && motor.output == 0);
  const auto COUNT = id.count_;
  id.Poll();
  assert(id.count_ == COUNT);
  motor.interrupt_can = &physical;
  id.command_ = 1;
  Imu();
  LibXR::Timebase::now += 2000;
  id.Poll();
  assert(id.status_ == Status::RUNNING);
  for (int n = 0; n < 30; ++n) {
    Imu();
    LibXR::Timebase::now += 2000;
    id.Poll();
    assert(!id.done_);
  }
  motor.interrupt_can = nullptr;
  LibXR::Timebase::now += 51000;
  Imu();
  id.Poll();
  assert(id.status_ == Status::FEEDBACK_LOST && motor.output == 0);
  Imu();
  physical.Receive();
  id.command_ = 1;
  LibXR::Timebase::now += 2000;
  id.Poll();
  physical.queue_result = LibXR::ErrorCode::FAILED;
  Imu();
  physical.Receive();
  LibXR::Timebase::now += 2000;
  id.Poll();
  assert(id.status_ == Status::TX_QUEUE_FAILED && motor.output == 0);
  physical.queue_result = LibXR::ErrorCode::OK;
  Imu();
  physical.Receive();
  id.command_ = 1;
  LibXR::Timebase::now += 2000;
  id.Poll();
  Imu(0, .1f);
  physical.Receive();
  LibXR::Timebase::now += 2000;
  id.Poll();
  assert(id.status_ == Status::PITCH_MOVED);
  Imu();
  physical.Receive();
  id.command_ = 2;
  LibXR::Timebase::now += 2000;
  id.Poll();
  assert(id.status_ == Status::RUNNING);
  assert(id.capture_.header.reserved_flags & 1);
  for (int n = 0; n < 1100; ++n) {
    LibXR::Timebase::now += 2000;
    Imu();
    physical.Receive();
    id.Poll();
  }
  assert(id.stage_ == 5 && motor.output < 0);
  assert(std::fabs(id.tau_cmd_ - observed.Torque()) < 1e-7f);
  id.command_ = 1;
  id.Poll();
  assert(id.status_ == Status::INVALID_COMMAND && motor.output == 0);
  std::cout << "PASS: observed packed torque/queue failure, CAN interrupt "
               "race, stop/freeze, feedback/pitch exits.\n";
}
