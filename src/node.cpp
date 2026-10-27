#include <anakin_mc/policy.hpp>
#include <anakin_mc/transactions.hpp>
#include <anakin_mc/transition.hpp>
#include <anakin_sdk/msg/control_target.hpp>
#include <anakin_sdk/msg/inference_status.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;
namespace anakin::mc {
using Target = anakin_sdk::msg::ControlTarget;
using Status = anakin_sdk::msg::InferenceStatus;
class Node final : public rclcpp::Node {
 public:
  Node() : rclcpp::Node("anakin_mc") {
    if (get_parameter("use_sim_time").as_bool())
      throw std::runtime_error("No official /clock publisher verified; use_sim_time must be false");
    const auto trace_path = declare_parameter<std::string>("trace_jsonl", "");
    if (!trace_path.empty()) {
      trace_.open(trace_path);
      if (!trace_) throw std::runtime_error("Cannot open inference diagnostic trace");
      trace_ << std::setprecision(17);
    }
    const auto loco = declare_parameter<std::string>("loco_model", "");
    const auto mimic = declare_parameter<std::string>("mimic_model", "");
    policies_["asap_loco"] = std::make_unique<Policy>("asap_loco", loco);
    policies_["asap_cr7"] = std::make_unique<Policy>("asap_cr7", mimic);
    pub_ = create_publisher<Target>("anakin/control_target",
                                    rclcpp::QoS(1).best_effort().durability_volatile());
    status_pub_ = create_publisher<Status>("anakin/inference_status",
                                           rclcpp::QoS(128).reliable().durability_volatile());
    sub_ = create_subscription<Frame>("anakin/state",
                                      rclcpp::QoS(1).best_effort().durability_volatile(),
                                      [this](Frame::ConstSharedPtr f) {
                                        latest_ = f;
                                        received_ = Clock::now();
                                      });
    service_ =
        create_service<Switch>("anakin/switch_inference_mode",
                               [this](const Switch::Request::SharedPtr req,
                                      Switch::Response::SharedPtr res) { request(*req, *res); });
    next_tick_ = Clock::now() + 20ms;
    timer_ = create_wall_timer(20ms, [this] { tick(); });
    hello_ = create_wall_timer(500ms, [this] { publish_statuses(); });
    RCLCPP_INFO(get_logger(), "Native MC instance %s; disarmed execution authority belongs to env",
                uuid_string(instance_).c_str());
  }

 private:
  using Clock = std::chrono::steady_clock;
  Uuid instance_ = new_uuid();
  Journal journal_;
  std::ofstream trace_;
  std::map<std::string, std::unique_ptr<Policy>> policies_;
  std::map<std::string, Status> statuses_;
  Frame::ConstSharedPtr latest_;
  Clock::time_point received_{}, next_tick_{};
  Switch::Request current_{};
  bool pending_ = false, started_ = false;
  std::string active_, source_;
  uint64_t seq_ = 0, last_state_seq_ = 0, frame_count_ = 0, epoch_ = 0;
  Uuid session_{};
  Positions upper_ = loco_pose();
  Status* status_ = nullptr;
  TransitionTimeline timeline_;
  StartupTimeline startup_;
  bool startup_initialized_ = false;
  rclcpp::Publisher<Target>::SharedPtr pub_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Subscription<Frame>::SharedPtr sub_;
  rclcpp::Service<Switch>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_, hello_;
  Clock::time_point requested_at_{};
  void request(const Switch::Request& q, Switch::Response& a) {
    a.mc_instance_id = instance_;
    a.epoch = q.next_epoch;
    if (auto old = journal_.find(q)) {
      a = old->response;
      if (!(old->request == q)) {
        a.accepted = false;
        a.reason = Switch::Request::ID_CONFLICT;
        a.detail = "Request UUID reused with different fields";
      }
      return;
    }
    auto reject = [&](uint8_t code, const std::string& why) {
      a.accepted = false;
      a.reason = code;
      a.detail = why;
      if (!journal_.full()) journal_.record(q, a);
    };
    if (journal_.full()) {
      reject(Switch::Request::BUSY, "Transaction journal full; restart disarmed MC");
      return;
    }
    if (q.expected_mc_instance_id != instance_) {
      reject(Switch::Request::WRONG_INSTANCE, "MC instance changed");
      return;
    }
    if (zero(q.session_id) || zero(q.request_id) || q.source_state_seq == 0 ||
        q.schema_id != schema_id ||
        !known_execution_contract(q.execution_profile, q.command_semantics, q.gain_profile) ||
        q.timeout.sec < 1 || q.timeout.sec > 10 || q.timeout.nanosec >= 1000000000) {
      reject(Switch::Request::INVALID, "Invalid schema, UUID, profile or timeout");
      return;
    }
    const bool startup_profile =
        q.transition_profile == "g1_startup_v1" && q.target_model == "asap_loco";
    if (!policies_.count(q.target_model) ||
        (q.transition_profile != "g1_named_upper_v1" && !startup_profile)) {
      reject(Switch::Request::UNSUPPORTED, "Unsupported model or transition profile");
      return;
    }
    if (q.session_id == session_ && q.next_epoch <= epoch_) {
      reject(Switch::Request::STALE_EPOCH, "Epoch did not advance");
      return;
    }
    if (status_ && pending_) {
      status_->state = Status::FAILED;
      status_->detail = "Superseded by a newer epoch";
      status_pub_->publish(*status_);
    }
    if (q.session_id != session_) {
      active_.clear();
      source_.clear();
      upper_ = loco_pose();
      last_state_seq_ = 0;
    }
    if (startup_profile)
      active_.clear();  // explicit new stand authorization never continues an old model
    startup_initialized_ = false;
    current_ = q;
    session_ = q.session_id;
    epoch_ = q.next_epoch;
    pending_ = true;
    started_ = false;
    status_ = nullptr;
    requested_at_ = Clock::now();
    a.accepted = true;
    a.reason = Switch::Request::OK;
    a.execution_profile = q.execution_profile;
    a.command_semantics = q.command_semantics;
    a.gain_profile = q.gain_profile;
    a.detail = "Accepted for a future frame boundary";
    journal_.record(q, a);
    auto& s = statuses_[uuid_string(q.session_id) + uuid_string(q.request_id)];
    s.schema_version = schema_version;
    s.schema_id = schema_id;
    s.session_id = q.session_id;
    s.mc_instance_id = instance_;
    s.request_id = q.request_id;
    s.epoch = q.next_epoch;
    s.requested_model = q.target_model;
    s.gain_profile = q.gain_profile;
    s.execution_profile = q.execution_profile;
    s.command_semantics = q.command_semantics;
    s.accepted = true;
    s.state = Status::PREPARING;
    status_ = &s;
  }
  void publish_statuses() {
    // Discovery is independent of the retained transaction journal. A fresh env
    // session must discover this instance without accepting an old session status.
    Status discovery;
    discovery.header.stamp = now();
    discovery.schema_version = schema_version;
    discovery.schema_id = schema_id;
    discovery.mc_instance_id = instance_;
    discovery.detail = "Instance discovery; session authorization is separate";
    status_pub_->publish(discovery);
    for (auto& item : statuses_) {
      item.second.header.stamp = now();
      status_pub_->publish(item.second);
    }
  }
  void fail(const std::string& why) {
    if (status_) {
      status_->state = Status::FAILED;
      status_->detail = why;
      status_->target_active = false;
      status_pub_->publish(*status_);
    }
    pending_ = false;
    active_.clear();
  }
  void tick() {
    const auto t = Clock::now();
    // Missed scheduled periods are skipped. Inference never creates a catch-up burst.
    const auto planned_deadline = next_tick_ + 20ms;
    if (t > planned_deadline) {
      while (next_tick_ <= t) next_tick_ += 20ms;
      return;
    }
    next_tick_ += 20ms;
    if (!status_) return;
    if (pending_ && t - requested_at_ > std::chrono::seconds(current_.timeout.sec) +
                                            std::chrono::nanoseconds(current_.timeout.nanosec)) {
      fail("Switch deadline expired; fallback requires new env epoch");
      return;
    }
    if (!latest_ || latest_->session_id != session_ || latest_->epoch != epoch_) return;
    if (t - received_ > 40ms) {
      if (started_) fail("State reception stale");
      return;
    }
    if (status_->state == Status::FAILED) return;
    if (pending_ && t - requested_at_ > std::chrono::seconds(current_.timeout.sec) +
                                            std::chrono::nanoseconds(current_.timeout.nanosec)) {
      fail("Switch deadline expired; fallback requires new env epoch");
      return;
    }
    const auto& f = *latest_;
    if (f.gain_profile != current_.gain_profile ||
        f.execution_profile != current_.execution_profile ||
        f.command_semantics != current_.command_semantics) {
      fail("State gain profile differs from transaction");
      return;
    }
    if (f.state_seq < current_.source_state_seq || f.state_seq <= last_state_seq_) return;
    last_state_seq_ = f.state_seq;
    try {
      if (!started_) {
        source_ = active_;
        policies_.at(current_.target_model)->reset();
        frame_count_ = 0;
        if (f.joints.name.size() != 29 || f.joints.position.size() != 29)
          throw std::runtime_error("Initial measured pose unavailable");
        (void)named_indices(f.joints.name);
        if (!source_.empty() && source_ != current_.target_model)
          timeline_.begin(current_.target_model == "asap_cr7", upper_,
                          current_.target_model == "asap_cr7" ? mimic_init() : loco_pose());
        started_ = true;
      }
      auto& target = *policies_.at(current_.target_model);
      const bool switching = !source_.empty() && source_ != current_.target_model;
      const bool entering_mimic = switching && current_.target_model == "asap_cr7";
      if (switching)
        upper_ = timeline_.upper();
      else
        upper_ = loco_pose();
      status_->target_ready = switching ? timeline_.target_ready() : frame_count_ >= 10;
      if (!switching && frame_count_ < 10) {
        target.observe(f, upper_);
        ++frame_count_;
        status_->source_state_seq = f.state_seq;
        status_pub_->publish(*status_);
        return;
      }
      const bool use_target = !switching || timeline_.target_controls();
      const std::string producing = use_target ? current_.target_model : source_;
      auto& producing_policy = *policies_.at(producing);
      const bool report_motion_done = producing_policy.motion_done();
      auto result = producing_policy.infer(f, upper_);
      const Positions policy_q_ref = result.q_ref;
      if (producing == "asap_loco")
        for (auto i : upper_override_indices()) result.q_ref[i] = upper_[i];
      const Positions mux_goal = result.q_ref;
      const bool startup_profile = current_.transition_profile == "g1_startup_v1";
      if (startup_profile) {
        if (!startup_initialized_) {
          Positions measured{};
          const auto map = named_indices(f.joints.name);
          for (size_t i = 0; i < 29; ++i) measured[map[i]] = f.joints.position[i];
          startup_.begin(measured);
          startup_initialized_ = true;
        }
        result.q_ref = startup_.apply(result.q_ref);
      }
      auto post_step = [&] {
        producing_policy.post_step();
        if (startup_profile) startup_.post_step();
        if (switching && !use_target) {
          Positions warmup_reference = entering_mimic ? upper_ : Positions{};
          target.observe(f, warmup_reference);
        }
        if (switching) {
          Positions measured{};
          auto map = named_indices(f.joints.name);
          for (size_t i = 0; i < 29; ++i) measured[map[i]] = f.joints.position[i];
          timeline_.post_step(measured);
        }
        ++frame_count_;
      };
      // A source mimic has no continuation permission in this profile. Warm it and
      // the target independently while env locally stops during the bounded wait.
      const bool may_emit =
          use_target || (current_.allow_source_continuation && source_ == "asap_loco");
      if (!may_emit) {
        post_step();
        status_->state = Status::PREPARING;
        status_->source_state_seq = f.state_seq;
        status_->source_continuation = false;
        status_->transition_entry = false;
        status_->target_active = false;
        status_pub_->publish(*status_);
        return;
      }
      const uint8_t phase =
          startup_profile && !startup_.complete() ? Target::TRANSITION_ENTRY
          : !switching || timeline_.complete()
              ? Target::TARGET_ACTIVE
              : (frame_count_ == 0 ? Target::SOURCE_CONTINUATION : Target::TRANSITION_ENTRY);
      if (Clock::now() > planned_deadline) {
        fail("Planned inference deadline missed");
        return;
      }
      Target cmd;
      cmd.header.stamp = now();
      cmd.schema_version = schema_version;
      cmd.schema_id = schema_id;
      cmd.session_id = session_;
      cmd.mc_instance_id = instance_;
      cmd.request_id = current_.request_id;
      cmd.epoch = epoch_;
      cmd.seq = ++seq_;
      cmd.source_state_seq = f.state_seq;
      cmd.model_id = producing;
      cmd.gain_profile = current_.gain_profile;
      cmd.execution_profile = current_.execution_profile;
      cmd.command_semantics = current_.command_semantics;
      cmd.validity = 1;
      cmd.valid_for.nanosec = 40000000;
      cmd.phase = phase;
      cmd.target.header = cmd.header;
      cmd.target.joint_names = joint_names();
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions.assign(result.q_ref.begin(), result.q_ref.end());
      cmd.target.points.push_back(point);
      pub_->publish(cmd);  // Completion below confirms publication only.
      post_step();
      if (trace_) {
        trace_ << "{\"state_seq\":" << f.state_seq << ",\"sdk_tick\":" << f.sdk_tick
               << ",\"epoch\":" << epoch_ << ",\"target_seq\":" << seq_
               << ",\"phase\":" << static_cast<int>(phase)
               << ",\"model\":" << std::quoted(producing)
               << ",\"execution_profile\":" << std::quoted(cmd.execution_profile)
               << ",\"command_semantics\":" << static_cast<int>(cmd.command_semantics);
        const auto vector = [this](const char* name, const auto& values) {
          trace_ << ",\"" << name << "\":[";
          bool first = true;
          for (auto value : values) {
            if (!first) trace_ << ',';
            first = false;
            trace_ << value;
          }
          trace_ << ']';
        };
        vector("observation", result.observation);
        vector("raw_action", result.raw_action);
        vector("policy_q_ref", policy_q_ref);
        vector("mux_goal", mux_goal);
        vector("published_q_ref", result.q_ref);
        vector("measured_q", f.joints.position);
        vector("measured_dq", f.joints.velocity);
        trace_ << "}\n";
      }
      const size_t p = phase == Target::SOURCE_CONTINUATION ? 0
                       : phase == Target::TRANSITION_ENTRY  ? 1
                                                            : 2;
      if (status_->phase_first_seq[p] == 0) {
        status_->phase_first_seq[p] = seq_;
      }
      status_->phase_last_seq[p] = seq_;
      status_->published_phase_mask |= phase;
      status_->source_state_seq = f.state_seq;
      status_->active_model = producing;
      status_->source_continuation = phase == Target::SOURCE_CONTINUATION;
      status_->transition_entry = phase == Target::TRANSITION_ENTRY;
      status_->target_active = phase == Target::TARGET_ACTIVE;
      status_->motion_done = report_motion_done;
      status_->state = phase == Target::TARGET_ACTIVE ? Status::ACTIVE : Status::TRANSITIONING;
      if (f.mc_instance_seen == instance_ && f.request_id_seen == current_.request_id &&
          f.target_seq_seen <= seq_) {
        for (size_t k = 0; k < 3; ++k) {
          const uint8_t bit = 1u << k;
          if ((f.phase_mask_seen & bit) && status_->phase_first_seq[k] &&
              f.target_seq_seen >= status_->phase_first_seq[k])
            status_->env_seen_phase_mask |= bit;
        }
      }
      if (f.sdk_published_mc_instance_id == instance_ &&
          f.sdk_published_request_id == current_.request_id && f.sdk_published_epoch == epoch_ &&
          f.sdk_published_target_seq <= seq_) {
        for (size_t k = 0; k < 3; ++k) {
          const uint8_t bit = 1u << k;
          if ((f.sdk_published_phase_mask & bit) && status_->phase_first_seq[k] &&
              f.sdk_published_target_seq >= status_->phase_first_seq[k])
            status_->env_sdk_published_phase_mask |= bit;
        }
      }
      if (phase == Target::TARGET_ACTIVE) {
        pending_ = false;
        active_ = current_.target_model;
      }
      status_->header.stamp = now();
      status_pub_->publish(*status_);
    } catch (const std::exception& e) {
      fail(e.what());
    }
  }
};
}  // namespace anakin::mc
int main(int argc, char** argv) {
  try {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<anakin::mc::Node>());
    rclcpp::shutdown();
    return 130;
  } catch (const std::exception& e) {
    std::cerr << "anakin_mc: " << e.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
