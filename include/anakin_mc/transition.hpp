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
// Explicit g1_startup_v1: ten model warmup observations precede this
// ten-callback canonical-position ramp. It never blends heterogeneous actions.
class StartupTimeline {
 public:
  void begin(const Positions& measured) {
    source_ = measured;
    count_ = 0;
  }
  Positions apply(const Positions& target) const {
    const double alpha = std::min(1., static_cast<double>(count_) / 10.);
    Positions command{};
    for (size_t i = 0; i < 29; ++i) command[i] = (1 - alpha) * source_[i] + alpha * target[i];
    return command;
  }
  void post_step() {
    if (count_ < 10) ++count_;
  }
  bool complete() const { return count_ >= 10; }

 private:
  Positions source_{};
  uint64_t count_ = 0;
};
}  // namespace anakin::mc
