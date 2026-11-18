#include "Experiment.hpp"
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

// Exercise the actual planner, torque limits, logger and completion against
// independent plants; none of these parameters are firmware assumptions.
void Simulate(float inertia, float damping, float friction, float stiffness,
              const std::string &file, bool disturbance = false) {
  SysId::Experiment id;
  id.interval_measurements_ = true;
  id.test_mode_ = SysId::Experiment::TestMode::FRICTION;
  id.min_position_ = -2.18f;
  id.max_position_ = -1.19f;
  id.enable_ = true;
  float position = 0, speed = 0, torque = 0, area = 0;
  unsigned stages[14] = {};
  float peak = 0, actuator = 0, delayed[3] = {};
  unsigned delay_index = 0;
  SysId::Experiment::Input input{.002f, -1.68f, 0,    0,    0,    0,
                                 0,     30,     true, true, true, true};
  for (unsigned tick = 0; tick < 15500; ++tick) {
    if (tick) {
      const float APPLIED = delayed[delay_index];
      delayed[delay_index] = torque;
      delay_index = (delay_index + 1) % 3;
      for (unsigned j = 0; j < 10; ++j) {
        actuator = disturbance
                       ? actuator + .0002f / .0062f * (APPLIED - actuator)
                       : torque;
        const float NET =
            actuator - damping * speed - stiffness * position - .008f;
        float f = std::copysign(friction, speed);
        if (std::fabs(speed) < .0001f)
          f = std::clamp(NET, -friction, friction);
        const float NEXT = speed + (NET - f) / inertia * .0002f;
        speed = speed * NEXT < 0 && std::fabs(NET) <= friction ? 0 : NEXT;
        position += speed * .0002f;
        area += speed * .0002f;
      }
    }
    input.position = -1.68f + position;
    // Delayed actuator + high-frequency body-axis disturbance absent from
    // the original ideal-plant tests. Rate measurement remains IMU-based.
    input.omega_gyro =
        speed +
        (disturbance ? .8f * std::sin(6.2831853f * 140 * tick * .002f) : 0);
    input.omega_motor = speed;
    input.torque_feedback = actuator;
    id.gyro_area_ = area;
    id.gyro_stamp_us_ = uint64_t(tick + 1) * 2000;
    torque = id.Step(input);
    torque = std::trunc(torque / .00013f) * .00013f;
    id.CommandQueued(torque);
    assert(std::fabs(torque) <= .20001f);
    peak = std::max(peak, std::fabs(position));
    if (id.stage_ < 14)
      ++stages[id.stage_];
    if (id.done_)
      break;
  }
  std::cout << "Sweep J=" << inertia << " status=" << id.status_
            << " time=" << id.time_s_ << " leg=" << id.sweep_leg_
            << " peak=" << peak << " count=" << id.count_ << std::endl;
  assert(id.status_ == SysId::Experiment::DONE);
  assert(id.time_s_ < 30 && id.count_ < SysId::Experiment::CAPACITY);
  assert(peak < .06f && std::fabs(position) < .002f);
  for (unsigned stage = 10; stage < 14; ++stage)
    assert(stages[stage] > 100);
  assert(id.capture_.header.reserved_flags ==
         (1U | (3U << 8) | (20000U << 16)));
  if (!file.empty()) {
    std::ofstream out(file, std::ios::binary);
    out.write(reinterpret_cast<const char *>(&id.capture_),
              sizeof(id.capture_));
  }
  const auto COUNT = id.count_;
  assert(id.Step(input) == 0 && id.count_ == COUNT);
}

int main(int argc, char **argv) {
  SysId::FrictionSweep sweep;
  SysId::FrictionSweep::Param p;
  assert(sweep.Reset(p, .2f, .06f));
  p.stroke = .06f;
  assert(!sweep.Reset(p, .2f, .06f));
  p = {};
  p.speed3 = p.speed2;
  assert(!sweep.Reset(p, .2f, .06f));
  SysId::Experiment stuck;
  stuck.interval_measurements_ = true;
  stuck.test_mode_ = SysId::Experiment::TestMode::FRICTION;
  stuck.sweep_param_.timeout_s = 10;
  stuck.enable_ = true;
  SysId::Experiment::Input input{.002f, -1.68f, 0,    0,    0,    0,
                                 0,     30,     true, true, true, true};
  for (unsigned tick = 0; tick < 5100 && !stuck.done_; ++tick) {
    stuck.gyro_stamp_us_ = uint64_t(tick + 1) * 2000;
    stuck.Step(input);
    if (tick == 1)
      stuck.sweep_param_.timeout_s = 30; // Running settings were latched.
  }
  assert(stuck.status_ == SysId::Experiment::SWEEP_TIMEOUT);
  assert(stuck.time_s_ < 10.01f && stuck.tau_cmd_ == 0);
  Simulate(.006f, .08f, .03f, 1.2f,
           argc > 1 ? std::string(argv[1]) + "/friction.bin" : "");
  Simulate(.012f, .06f, .02f, .7f, "");
  Simulate(.006f, .08f, .03f, 1.2f, "", true);
  Simulate(.012f, .08f, .03f, 1.2f, "", true);
  std::cout
      << "PASS: bounded multi-speed traversal, return, freeze and limits.\n";
}
