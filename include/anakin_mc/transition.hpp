#pragma once
#include <algorithm>
#include <anakin_mc/policy.hpp>
namespace anakin::mc {
// A discrete post-step schedule matching PolicyInterpManager/StepTimer. Permissions
// to emit the selected source model are handled separately by the ROS adapter.
class TransitionTimeline {
 public:
  void begin(bool entering_mimic, const Positions& previous_upper, const Positions& target) {
    entering_mimic_ = entering_mimic;
    count_ = 0;
    upper_ = previous_upper;
    target_ = target;
    start_ = {};
  }
  bool target_controls() const { return count_ >= (entering_mimic_ ? 101u : 25u); }
  bool target_ready() const { return count_ >= (entering_mimic_ ? 10u : 25u); }
  bool complete() const { return count_ >= 101; }
  uint64_t count() const { return count_; }
  const Positions& upper() const { return upper_; }
  void post_step(const Positions& measured) {
    ++count_;
    const uint64_t start_at = entering_mimic_ ? 1 : 25;
    if (count_ < start_at) return;
    if (count_ == start_at) start_ = measured;  // measured at the actual takeover callback.
    const double alpha = std::min(1., static_cast<double>(count_ - start_at) / 75.);
    for (size_t i = 0; i < 29; ++i) upper_[i] = (1 - alpha) * start_[i] + alpha * target_[i];
  }

 private:
  bool entering_mimic_ = true;
  uint64_t count_ = 0;
  Positions upper_{}, target_{}, start_{};
};
}  // namespace anakin::mc
