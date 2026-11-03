#include <anakin_mc/inference_gate.hpp>
#include <anakin_mc/transition.hpp>
#include <iostream>
using namespace anakin;
using namespace anakin::mc;
using namespace std::chrono_literals;
using Switch = anakin_sdk::srv::SwitchInferenceMode;
void check(bool condition, const char* reason) { if (!condition) throw std::runtime_error(reason); }
int main() {
  try {
    using Clock = std::chrono::steady_clock;
    auto now = Clock::time_point{} + 10s;
    Switch::Request request;
    request.session_id = new_uuid(); request.next_epoch = 1; request.source_state_seq = 1;
    request.request_id = new_uuid(); request.gain_profile = merged_gain_profile;
    request.execution_profile = sdk_pd_execution_profile; request.command_semantics = pd_reference;
    Frame frame;
    frame.schema_version = schema_version; frame.schema_id = schema_id;
    frame.session_id = request.session_id; frame.epoch = 1; frame.state_seq = 1;
    frame.gain_profile = request.gain_profile; frame.execution_profile = request.execution_profile;
    frame.command_semantics = request.command_semantics; frame.validity = 7;
    frame.execution_mode = Frame::MODE_PREPARING; frame.execution_authorized = true;
    uint64_t last = 0, observations = 0, resets = 0, publications = 0;
    bool started = false;
    TransitionTimeline timeline;
    timeline.begin(true, {}, {});
    const auto work = [&](const Frame&) {
      if (!started) { ++resets; started = true; }
      ++observations; ++publications; timeline.post_step({});
    };
    auto decision = with_inference_frame(&frame, request, now, now, last, work);
    check(decision == FrameDecision::Processed && observations == 1 && resets == 1,
          "Preparing explicitly permits the first warmup frame, without waiting for Active");
    const auto count = timeline.count();
    for (int i = 0; i < 1000; ++i) {
      check(with_inference_frame(&frame, request, now, now + 30s, last, work) == FrameDecision::Waiting,
            "Long reception gaps wait instead of clearing model state");
      check(with_inference_frame(nullptr, request, now, now, last, work) == FrameDecision::Waiting,
            "Absent frame is an inert wait");
    }
    check(observations == 1 && publications == 1 && resets == 1 && timeline.count() == count && last == 1,
          "Waiting cannot invoke inference/history/reset/post_step/publication callbacks");
    frame.state_seq = 2;
    frame.execution_mode = Frame::MODE_HOLD;
    check(with_inference_frame(&frame, request, now, now, last, work) == FrameDecision::Waiting,
          "Local Hold does not advance a policy");
    frame.execution_mode = Frame::MODE_ACTIVE;
    frame.execution_authorized = false;
    check(with_inference_frame(&frame, request, now, now, last, work) == FrameDecision::Waiting,
          "State/lowcmd-off permission waits without consuming the physical frame");
    frame.execution_authorized = true;
    frame.sdk_tick = 0; // Deliberate backwards diagnostic tick has no lifecycle effect.
    check(with_inference_frame(&frame, request, now + 30s, now + 30s, last, work) == FrameDecision::Processed &&
          resets == 1 && observations == 2 && timeline.count() == count + 1,
          "A valid new sequence resumes existing context after a backwards SDK tick");
    check(with_inference_frame(&frame, request, now, now, last, work) == FrameDecision::Waiting,
          "A duplicate Env physical sequence cannot advance inference");
    ++frame.state_seq;
    frame.execution_profile = strict_execution_profile;
    check(with_inference_frame(&frame, request, now, now, last, work) == FrameDecision::ContractMismatch &&
          observations == 2, "Contract corruption is distinct from waiting");
    frame.execution_profile = sdk_pd_execution_profile;
    ++request.next_epoch; frame.epoch = request.next_epoch;
    request.request_id = new_uuid(); started = false; // Only an explicit new transaction initializes.
    check(with_inference_frame(&frame, request, now, now, last, work) == FrameDecision::Processed && resets == 2,
          "Ordinary explicit switching retains its target initialization boundary");
    check(known_execution_contract(sdk_pd_execution_profile, pd_reference, merged_gain_profile) &&
          !known_execution_contract(sdk_pd_execution_profile, mechanical_position, merged_gain_profile),
          "Common SDK PD tuple is accepted consistently, with mismatched semantics rejected");
    std::cout << "MC inference waiting contracts passed\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
