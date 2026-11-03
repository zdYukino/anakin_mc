#pragma once
#include <anakin_sdk/contract.hpp>
#include <anakin_sdk/msg/robot_state_frame.hpp>
#include <anakin_sdk/srv/switch_inference_mode.hpp>
#include <chrono>

namespace anakin::mc {
enum class FrameDecision { Waiting, Processed, ContractMismatch };
// This is the production node's only frame admission. Waiting performs no work:
// no policy reset/observe/infer/post_step, timeline advance or target publication.
// Request timeouts are Env execution guards, not MC model-lifetime timers.
template <class Work>
FrameDecision with_inference_frame(const anakin_sdk::msg::RobotStateFrame* frame,
    const anakin_sdk::srv::SwitchInferenceMode::Request& request,
    std::chrono::steady_clock::time_point received, std::chrono::steady_clock::time_point now,
    uint64_t& last_state_sequence, Work&& work) {
  using Frame = anakin_sdk::msg::RobotStateFrame;
  if (!frame || frame->session_id != request.session_id || frame->epoch != request.next_epoch ||
      now < received || now - received > std::chrono::milliseconds(40) ||
      !frame->execution_authorized || (frame->execution_mode != Frame::MODE_PREPARING &&
      frame->execution_mode != Frame::MODE_ACTIVE) || frame->state_seq < request.source_state_seq ||
      frame->state_seq <= last_state_sequence || (frame->validity & 7) != 7)
    return FrameDecision::Waiting;
  if (frame->schema_version != schema_version || frame->schema_id != schema_id ||
      frame->gain_profile != request.gain_profile || frame->execution_profile != request.execution_profile ||
      frame->command_semantics != request.command_semantics)
    return FrameDecision::ContractMismatch;
  last_state_sequence = frame->state_seq;
  work(*frame);
  return FrameDecision::Processed;
}
}  // namespace anakin::mc
