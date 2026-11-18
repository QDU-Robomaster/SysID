#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace SysId {
// Position-bounded velocity traversals; no assumed J/b/fc or motor model.
class FrictionSweep {
public:
  struct Param {
    float stroke = .04f;
    float speed1 = .10f;
    float speed2 = .20f;
    float speed3 = .30f;
    float speed4 = .40f;
    float acceleration = 8.0f;
    float speed_kp = .25f;
    float speed_ki = 6.0f;
    uint32_t repeats = 3;
    float dwell_s = .15f;
    float timeout_s = 30.0f;
    float speed_filter_hz = 20.0f;
  };
  bool Reset(const Param &p, float torque_limit, float travel_limit) {
    const float VALUES[] = {p.stroke,    p.speed1,     p.speed2,
                            p.speed3,    p.speed4,     p.acceleration,
                            p.speed_kp,  p.speed_ki,   p.dwell_s,
                            p.timeout_s, torque_limit, p.speed_filter_hz};
    for (const float VALUE : VALUES)
      if (!std::isfinite(VALUE) || VALUE <= 0)
        return false;
    if (p.stroke < .02f || p.stroke + .01f > travel_limit ||
        p.speed1 >= p.speed2 || p.speed2 >= p.speed3 || p.speed3 >= p.speed4 ||
        p.speed4 > 1.0f || p.acceleration > 20 || p.repeats < 2 ||
        p.repeats > 5 || p.timeout_s > 30 || p.timeout_s < 10 ||
        p.speed_filter_hz > 50)
      return false;
    param_ = p;
    limit_ = torque_limit;
    phase_ = Phase::HOME;
    leg_ = 0;
    reference_ = 0;
    integral_ = 0;
    filtered_ = 0;
    filter_first_ = 0;
    arrival_speed_ = 0;
    arrival_speed_ = 0;
    wait_ = 0;
    arrived_ = 0;
    done_ = false;
    saturated_ = false;
    return true;
  }
  float Step(float position, float omega, float dt) {
    if (done_)
      return 0;
    // Control feedback only. Raw full-rate gyro integrals remain in the log.
    const float FILTER_TAU = 1.0f / (6.2831853f * param_.speed_filter_hz);
    const float FILTER_ALPHA = dt / (FILTER_TAU + dt);
    filter_first_ += FILTER_ALPHA * (omega - filter_first_);
    filtered_ = filter_first_;
    arrival_speed_ += dt / (.1f + dt) * (filtered_ - arrival_speed_);
    const float TARGET = phase_ == Phase::HOME ? -param_.stroke
                         : phase_ == Phase::RETURN
                             ? 0
                             : (leg_ % 2 == 0 ? param_.stroke : -param_.stroke);
    const float MAX_SPEED = (phase_ == Phase::HOME || phase_ == Phase::RETURN)
                                ? param_.speed1
                                : Speed();
    const float ERROR = TARGET - position;
    // Brake before the endpoint, then wait for actual position/speed settling.
    const float BRAKING = std::sqrt(2 * param_.acceleration *
                                    std::max(std::fabs(ERROR) - .0008f, 0.0f));
    float wanted = std::copysign(
        std::min({MAX_SPEED, BRAKING, 20.0f * std::fabs(ERROR)}), ERROR);
    if (std::fabs(ERROR) < .0008f)
      wanted = 0;
    if (phase_ == Phase::DWELL)
      wanted = 0;
    reference_ += std::clamp(wanted - reference_, -param_.acceleration * dt,
                             param_.acceleration * dt);
    const float SPEED_ERROR = reference_ - filtered_;
    const float CANDIDATE = integral_ + param_.speed_ki * SPEED_ERROR * dt;
    const float RAW = param_.speed_kp * SPEED_ERROR + CANDIDATE;
    const float TORQUE = std::clamp(RAW, -limit_, limit_);
    saturated_ = RAW != TORQUE;
    if (!saturated_ || RAW * SPEED_ERROR <= 0)
      integral_ = CANDIDATE;
    integral_ = std::clamp(integral_, -limit_, limit_);
    const bool AT_TARGET =
        std::fabs(ERROR) < .002f && std::fabs(arrival_speed_) < .025f;
    if (phase_ == Phase::DWELL) {
      wait_ += dt;
      if (wait_ >= param_.dwell_s) {
        ++leg_;
        phase_ = leg_ >= 8 * param_.repeats ? Phase::RETURN : Phase::MOVE;
        arrived_ = 0;
      }
    } else {
      arrived_ = AT_TARGET ? arrived_ + dt : 0;
      if (arrived_ >= .04f) {
        if (phase_ == Phase::HOME) {
          phase_ = Phase::MOVE;
          arrived_ = 0;
        } else if (phase_ == Phase::RETURN)
          done_ = true;
        else {
          phase_ = Phase::DWELL;
          wait_ = 0;
        }
      }
    }
    return done_ ? 0 : TORQUE;
  }
  uint32_t Stage() const {
    if (phase_ == Phase::HOME)
      return 5;
    if (phase_ == Phase::DWELL)
      return 6;
    if (phase_ == Phase::RETURN)
      return 7;
    return 10 + std::min(leg_ / (2 * param_.repeats), 3U);
  }
  float Speed() const {
    const float VALUES[] = {param_.speed1, param_.speed2, param_.speed3,
                            param_.speed4};
    return VALUES[std::min(leg_ / (2 * param_.repeats), 3U)];
  }
  float FilteredSpeed() const { return filtered_; }
  float Reference() const { return reference_; }
  uint32_t Leg() const { return leg_; }
  bool Done() const { return done_; }
  bool Saturated() const { return saturated_; }

private:
  enum class Phase { HOME, MOVE, DWELL, RETURN };
  Param param_;
  float limit_ = 0, reference_ = 0, integral_ = 0, wait_ = 0, arrived_ = 0,
        filtered_ = 0, filter_first_ = 0, arrival_speed_ = 0;
  uint32_t leg_ = 0;
  Phase phase_ = Phase::HOME;
  bool done_ = false, saturated_ = false;
};
} // namespace SysId
