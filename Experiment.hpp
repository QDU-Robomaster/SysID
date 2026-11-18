#pragma once

#include "FrictionSweep.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

// Restored Double Yaw excitation and interval-recording core. No hardware
// access. Generic: no motor type, machine zero/limits or known inertia is
// embedded. Input torque, feedback torque and both rates use position-positive
// signs.
namespace SysId {
class Experiment {
public:
  enum Status : uint32_t {
    IDLE = 0,
    WAITING = 0,
    RUNNING = 1,
    DONE = 2,
    INVALID_INPUT = 3,
    BAD_INPUT = 3,
    MOTOR_OFFLINE = 4,
    FEEDBACK_LOST = 4,
    OVERSPEED = 5,
    POSITION_LIMIT = 6,
    POSITION_JUMP = 6,
    BUFFER_FULL = 7,
    UNITS_UNCONFIRMED = 8,
    STOP_NOT_SETTLED = 9,
    TRAVEL_LIMIT = 10,
    OVER_TEMPERATURE = 11,
    INVALID_COMMAND = 12,
    IMU_OFFLINE = 13,
    PITCH_MOVED = 14,
    TX_QUEUE_FAILED = 15,
    MANUAL_STOP = 16,
    BAD_DT = 17,
    TX_NOT_OBSERVED = 18,
    SWEEP_TIMEOUT = 19
  };
  enum Flag : uint32_t { SATURATED = 1, MOTOR_REUSED = 2, GYRO_REUSED = 4 };
  static constexpr uint32_t CAPACITY = 1536;
  // Supplied by the CAN torque encoding, never a fixed motor model.
  volatile float torque_constant_ = 0.0f;
  volatile float command_quantum_ = 0.0f;
  struct Header {
    uint32_t magic, version, sample_bytes, capacity;
    uint32_t count, status, seed, reserved_flags;
    float center, min_position, max_position, amplitude;
    float kp, kd, torque_limit, max_travel;
    float settle_s, excite_s, return_s, log_period_s;
    float hold_fast_s, hold_mid_s, hold_slow_s;
    float torque_constant, command_quantum;
    uint32_t saturation_intervals, feedback_stale_intervals;
    float elapsed_s;
    float reserved[4];
  };
  struct Sample {
    float time_s, position, omega_gyro, omega_motor;
    // Interval ending at time_s. Commands are AFTER PD, clipping and GM6020
    // quantization; feedback is current-derived torque, not a torque sensor.
    float torque_cmd_mean, torque_feedback_mean, excitation_mean, pd_mean;
    float pitch, gyro_y;
    uint32_t stage, flags;
  };
  struct Capture {
    Header header;
    Sample samples[CAPACITY];
  };
  struct Input {
    float dt, position, omega_gyro, omega_motor, torque_feedback;
    float pitch, gyro_y, temperature;
    bool motor_online, imu_online, motor_new, gyro_new;
  };

  // Ozone: edit parameters while disabled, then set enable_=true. Parameters
  // are latched for the entire capture. Starting again overwrites the old run.
  volatile bool enable_ = false;
  volatile bool done_ = false;
  volatile Status status_ = IDLE;
  volatile uint32_t count_ = 0;
  volatile uint32_t stage_ = 0;
  volatile float time_s_ = 0.0f;
  volatile float position_ = 0.0f;
  volatile float omega_ = 0.0f;
  volatile float tau_cmd_ = 0.0f;
  volatile float prbs_ = 0.0f;
  volatile float pd_ = 0.0f;
  volatile float amplitude_ = 0.07f;
  volatile float kp_ = 2.0f;
  volatile float kd_ = 0.12f;
  volatile float torque_limit_ = 0.20f;
  volatile float max_travel_ = 0.06f;
  // Absolute limits belong to the application YAML.
  volatile float min_position_ = -1000000.0f;
  volatile float max_position_ = 1000000.0f;
  volatile uint32_t seed_ = 0xACE1;
  Capture capture_{};
  enum class TestMode : uint32_t { PRBS = 0, FRICTION = 1 };
  volatile TestMode test_mode_ = TestMode::PRBS;
  FrictionSweep::Param sweep_param_{};
  volatile float omega_ref_ = 0;
  volatile uint32_t sweep_leg_ = 0;
  // V3 adapter supplies full-rate gyro integrals. Core-only reference mode
  // stays endpoint-based so the restored experiment can be regression-tested.
  bool interval_measurements_ = false;
  float gyro_area_ = 0;
  uint64_t gyro_stamp_us_ = 0;

  bool Active() const { return active_; }
  void Cancel(Status reason) { Stop(reason); }

  float Step(const Input &input) {
    if (!enable_) {
      return active_ ? Stop(MANUAL_STOP, &input) : 0.0f;
    }
    const bool STARTING = !active_;
    if (STARTING && !Start(input))
      return 0.0f;
    if (!Finite(input))
      return Stop(INVALID_INPUT);
    if (!input.motor_online)
      return Stop(MOTOR_OFFLINE, &input);
    if (!input.imu_online)
      return Stop(IMU_OFFLINE, &input);
    if (input.dt <= 0.0f || input.dt > 0.02f)
      return Stop(BAD_DT);
    if (input.temperature >= 75.0f)
      return Stop(OVER_TEMPERATURE, &input);
    if (STARTING)
      return 0.0f;
    // Record the input held over the elapsed interval, BEFORE computing a new
    // command. This avoids attributing the next PRBS bit to the last response.
    time_ += input.dt;
    interval_ += input.dt;
    position_integral_ += previous_position_ * input.dt;
    previous_position_ = input.position;
    cmd_integral_ += previous_tau_ * input.dt;
    feedback_integral_ += input.torque_feedback * input.dt;
    excitation_integral_ += previous_prbs_ * input.dt;
    pd_integral_ += previous_pd_ * input.dt;
    if (previous_saturated_) {
      flags_ |= SATURATED;
      capture_.header.saturation_intervals++;
    }
    if (!input.motor_new) {
      flags_ |= MOTOR_REUSED;
      capture_.header.feedback_stale_intervals++;
    }
    if (!input.gyro_new)
      flags_ |= GYRO_REUSED;
    position_ = input.position;
    omega_ = input.omega_gyro;
    time_s_ = time_;
    capture_.header.elapsed_s = time_;
    const auto &H = capture_.header;
    if (input.position <= H.min_position + 0.015f ||
        input.position >= H.max_position - 0.015f)
      return Stop(POSITION_LIMIT, &input);
    if (std::fabs(input.position - H.center) >= H.max_travel)
      return Stop(TRAVEL_LIMIT, &input);
    if (std::fabs(input.omega_gyro) > 3.0f ||
        std::fabs(input.omega_motor) > 3.0f)
      return Stop(OVERSPEED, &input);
    if (std::fabs(input.pitch - initial_pitch_) > 0.05f)
      return Stop(PITCH_MOVED, &input);
    if (interval_ >= H.log_period_s && !Record(input))
      return Stop(BUFFER_FULL);
    if (latched_mode_ == TestMode::FRICTION) {
      if (time_ >= sweep_timeout_)
        return Stop(SWEEP_TIMEOUT, &input);
      float raw = 0;
      if (time_ < H.settle_s) {
        raw = -H.kp * (input.position - H.center) - H.kd * input.omega_gyro;
        stage_ = 0;
        omega_ref_ = 0;
      } else {
        raw =
            sweep_.Step(input.position - H.center, input.omega_gyro, input.dt);
        stage_ = sweep_.Stage();
        omega_ref_ = sweep_.Reference();
        sweep_leg_ = sweep_.Leg();
        if (sweep_.Done())
          return Stop(DONE, &input);
      }
      const float REQUEST = std::clamp(raw, -H.torque_limit, H.torque_limit);
      previous_tau_ = REQUEST;
      previous_prbs_ = omega_ref_;
      previous_pd_ = REQUEST;
      previous_saturated_ =
          raw != REQUEST || (time_ >= H.settle_s && sweep_.Saturated());
      previous_stage_ = stage_;
      tau_cmd_ = REQUEST;
      prbs_ = omega_ref_;
      pd_ = REQUEST;
      return REQUEST;
    }
    if (time_ >= H.settle_s + H.excite_s + H.return_s)
      return Stop(DONE, &input);

    const float EXCITE_TIME = time_ - H.settle_s;
    float excitation = 0.0f;
    uint32_t stage = 0;
    if (EXCITE_TIME >= 0.0f && EXCITE_TIME < H.excite_s) {
      stage = 1U + static_cast<uint32_t>(EXCITE_TIME / (H.excite_s / 3.0f));
      const float HOLD = stage == 1   ? H.hold_fast_s
                         : stage == 2 ? H.hold_mid_s
                                      : H.hold_slow_s;
      if (stage != previous_stage_ || time_ >= next_bit_time_) {
        const uint16_t BIT = static_cast<uint16_t>(
            (lfsr_ ^ (lfsr_ >> 2) ^ (lfsr_ >> 3) ^ (lfsr_ >> 5)) & 1U);
        lfsr_ = static_cast<uint16_t>((lfsr_ >> 1) | (BIT << 15));
        sign_ = (lfsr_ & 1U) ? 1.0f : -1.0f;
        next_bit_time_ = time_ + HOLD;
      }
      excitation = sign_ * H.amplitude * (stage == 2 ? 0.65f : 1.0f);
    } else if (EXCITE_TIME >= H.excite_s) {
      stage = 4;
    }
    const float PD =
        -H.kp * (input.position - H.center) - H.kd * input.omega_gyro;
    const float RAW = excitation + PD;
    const float CLIPPED = std::clamp(RAW, -H.torque_limit, H.torque_limit);
    // Prediction only: CommandQueued replaces this with the actual packed
    // value.
    const float QUANTUM = command_quantum_;
    previous_tau_ =
        QUANTUM > 0 ? std::trunc(CLIPPED / QUANTUM) * QUANTUM : CLIPPED;
    previous_prbs_ = excitation;
    previous_pd_ = PD;
    previous_saturated_ = RAW != CLIPPED;
    previous_stage_ = stage;
    stage_ = stage;
    tau_cmd_ = previous_tau_;
    prbs_ = excitation;
    pd_ = PD;
    return CLIPPED;
  }

  void TxFailed() {
    if (active_)
      Stop(TX_QUEUE_FAILED);
  }
  // Actual packed torque observed at CAN AddMessage, in position-positive
  // units.
  void CommandQueued(float torque) {
    if (active_) {
      previous_tau_ = torque;
      tau_cmd_ = torque;
    }
  }

private:
  static bool Finite(const Input &input) {
    return std::isfinite(input.position) && std::isfinite(input.omega_gyro) &&
           std::isfinite(input.omega_motor) &&
           std::isfinite(input.torque_feedback) && std::isfinite(input.pitch) &&
           std::isfinite(input.gyro_y) && std::isfinite(input.temperature) &&
           std::isfinite(input.dt);
  }
  bool Start(const Input &input) {
    auto &h = capture_.header;
    h = {};
    h.magic = 0x59534944;
    h.version = interval_measurements_ ? 3 : 2;
    h.sample_bytes = sizeof(Sample);
    h.capacity = CAPACITY;
    h.seed = seed_;
    h.center = input.position;
    h.min_position = min_position_;
    h.max_position = max_position_;
    h.amplitude = amplitude_;
    h.kp = kp_;
    h.kd = kd_;
    h.torque_limit = torque_limit_;
    h.max_travel = max_travel_;
    h.settle_s = 2.0f;
    h.excite_s = 12.0f;
    h.return_s = 1.0f;
    h.log_period_s = 0.01f;
    h.hold_fast_s = 0.04f;
    h.hold_mid_s = 0.10f;
    h.hold_slow_s = 0.25f;
    h.torque_constant = torque_constant_;
    h.command_quantum = command_quantum_;
    latched_mode_ = test_mode_;
    if (latched_mode_ != TestMode::PRBS &&
        latched_mode_ != TestMode::FRICTION) {
      Stop(INVALID_INPUT);
      return false;
    }
    if (latched_mode_ == TestMode::FRICTION) {
      const auto P = sweep_param_;
      if (!interval_measurements_ ||
          !sweep_.Reset(P, h.torque_limit, h.max_travel) ||
          input.position - P.stroke <= h.min_position + .02f ||
          input.position + P.stroke >= h.max_position - .02f) {
        Stop(INVALID_INPUT);
        return false;
      }
      h.reserved_flags = 1U | (P.repeats << 8);
      h.amplitude = P.speed1;
      h.hold_fast_s = P.speed2;
      h.hold_mid_s = P.speed3;
      h.hold_slow_s = P.speed4;
      h.excite_s = P.timeout_s - 3;
      h.return_s = 1;
      h.log_period_s = .02f;
      h.reserved[0] = P.stroke;
      h.reserved[1] = P.acceleration;
      h.reserved[2] = P.speed_kp;
      h.reserved[3] = P.speed_ki;
      sweep_timeout_ = P.timeout_s;
      omega_ref_ = 0;
      sweep_leg_ = 0;
    }
    count_ = 0;
    done_ = false;
    time_ = 0.0f;
    time_s_ = 0.0f;
    position_integral_ = 0;
    previous_position_ = input.position;
    last_gyro_area_ = gyro_area_;
    last_gyro_stamp_ = gyro_stamp_us_;
    interval_ = 0.0f;
    cmd_integral_ = feedback_integral_ = 0.0f;
    excitation_integral_ = pd_integral_ = 0.0f;
    flags_ = 0;
    previous_tau_ = previous_prbs_ = previous_pd_ = 0.0f;
    previous_saturated_ = false;
    previous_stage_ = 0;
    lfsr_ = static_cast<uint16_t>(h.seed);
    next_bit_time_ = 0.0f;
    initial_pitch_ = input.pitch;
    const float VALUES[] = {h.center,       h.min_position, h.max_position,
                            h.amplitude,    h.kp,           h.kd,
                            h.torque_limit, h.max_travel};
    for (const float VALUE : VALUES) {
      if (!std::isfinite(VALUE)) {
        Stop(INVALID_INPUT);
        return false;
      }
    }
    if (!Finite(input) ||
        (latched_mode_ == TestMode::PRBS &&
         (h.amplitude <= 0.0f || h.amplitude > h.torque_limit)) ||
        h.kp <= 0.0f || h.kd < 0.0f || h.torque_limit > 0.5f ||
        h.max_travel <= 0.005f || h.max_travel > 0.2f || h.seed == 0 ||
        h.seed > 65535 || h.min_position + 0.015f >= h.center ||
        h.max_position - 0.015f <= h.center) {
      Stop(INVALID_INPUT);
      return false;
    }
    active_ = true;
    status_ = RUNNING;
    h.status = RUNNING;
    stage_ = 0;
    position_ = input.position;
    omega_ = input.omega_gyro;
    // Initial sample is the left endpoint, with no elapsed input interval.
    capture_.samples[0] = {0.0f,
                           input.position,
                           input.omega_gyro,
                           input.omega_motor,
                           0.0f,
                           input.torque_feedback,
                           0.0f,
                           0.0f,
                           input.pitch,
                           input.gyro_y,
                           0,
                           0};
    h.count = 1;
    count_ = 1;
    return true;
  }
  bool Record(const Input &input) {
    auto &h = capture_.header;
    if (h.count >= CAPACITY || interval_ <= 0.0f)
      return false;
    float RECORDED_SPEED = input.omega_gyro;
    float RECORDED_POSITION = input.gyro_y;
    if (interval_measurements_) {
      if (gyro_stamp_us_ > last_gyro_stamp_) {
        const float SPAN =
            static_cast<float>(gyro_stamp_us_ - last_gyro_stamp_) * 1e-6f;
        RECORDED_SPEED = (gyro_area_ - last_gyro_area_) / SPAN;
      }
      RECORDED_POSITION = position_integral_ / interval_;
      last_gyro_area_ = gyro_area_;
      last_gyro_stamp_ = gyro_stamp_us_;
      position_integral_ = 0;
    }
    capture_.samples[h.count] = {time_,
                                 input.position,
                                 RECORDED_SPEED,
                                 input.omega_motor,
                                 cmd_integral_ / interval_,
                                 feedback_integral_ / interval_,
                                 excitation_integral_ / interval_,
                                 pd_integral_ / interval_,
                                 input.pitch,
                                 RECORDED_POSITION,
                                 previous_stage_,
                                 flags_};
    h.count++;
    count_ = h.count;
    interval_ = 0.0f;
    cmd_integral_ = feedback_integral_ = 0.0f;
    excitation_integral_ = pd_integral_ = 0.0f;
    flags_ = 0;
    return true;
  }
  float Stop(Status status, const Input *input = nullptr) {
    if (active_ && input && interval_ > 0.0f && Finite(*input))
      Record(*input);
    active_ = false;
    enable_ = false;
    done_ = true;
    status_ = status;
    capture_.header.status = status;
    tau_cmd_ = 0.0f;
    prbs_ = 0.0f;
    pd_ = 0.0f;
    return 0.0f;
  }
  FrictionSweep sweep_;
  TestMode latched_mode_ = TestMode::PRBS;
  float sweep_timeout_ = 30;
  float position_integral_ = 0, previous_position_ = 0, last_gyro_area_ = 0;
  uint64_t last_gyro_stamp_ = 0;
  bool active_ = false, previous_saturated_ = false;
  uint16_t lfsr_ = 0;
  uint32_t previous_stage_ = 0, flags_ = 0;
  float time_ = 0.0f, interval_ = 0.0f, initial_pitch_ = 0.0f;
  float next_bit_time_ = 0.0f, sign_ = 1.0f;
  float previous_tau_ = 0.0f, previous_prbs_ = 0.0f, previous_pd_ = 0.0f;
  float cmd_integral_ = 0.0f, feedback_integral_ = 0.0f;
  float excitation_integral_ = 0.0f, pd_integral_ = 0.0f;
};
static_assert(sizeof(Experiment::Header) == 128);
static_assert(sizeof(Experiment::Sample) == 48);
static_assert(sizeof(Experiment::Capture) == 0x12080);
static_assert(offsetof(Experiment::Capture, samples) == 128);

} // namespace SysId
