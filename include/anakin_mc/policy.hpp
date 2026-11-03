#pragma once
#include <onnxruntime_cxx_api.h>

#include <anakin_sdk/msg/robot_state_frame.hpp>
#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>
namespace anakin::mc {
using Frame = anakin_sdk::msg::RobotStateFrame;
using Positions = std::array<double, 29>;
const Positions& defaults();
Positions loco_pose();
Positions mimic_init();
std::vector<size_t> mimic_indices();
const std::vector<size_t>& upper_override_indices();
struct Result {
  std::vector<float> observation, raw_action;
  Positions q_ref;
};
// Each instance owns its own history and previous action. No cross-model cache.
class Policy {
 public:
  Policy(std::string id, const std::string& path);
  void reset();
  std::vector<float> observe(const Frame& frame, const Positions& upper_reference);
  Result infer(const Frame& frame, const Positions& upper_reference);
  // Each policy chooses the reference used both by its observations and by
  // the PD targets for joints outside its action space.
  Positions uncontrolled_reference(const Positions& measured) const;
  void apply_uncontrolled_reference(Positions& command, const Positions& reference) const;
  void post_step();
  bool motion_done() const;
  const std::string& id() const { return id_; }

 private:
  std::string id_;
  Ort::Env runtime_{ORT_LOGGING_LEVEL_WARNING, "anakin_mc"};
  Ort::SessionOptions options_;
  std::unique_ptr<Ort::Session> session_;
  std::vector<float> last_;
  using Groups = std::vector<std::vector<float>>;
  std::deque<Groups> history_;
  uint64_t steps_ = 0;
  bool motion_done_ = false;
  std::vector<size_t> indices_;
  std::array<bool, 29> controlled_{};
};
}  // namespace anakin::mc
