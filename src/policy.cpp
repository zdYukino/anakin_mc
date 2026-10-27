#include <algorithm>
#include <anakin_mc/policy.hpp>
#include <anakin_sdk/contract.hpp>
#include <cmath>
#include <numeric>
#include <stdexcept>
namespace anakin::mc {
const Positions& defaults() {
  static const Positions q = {-.1, 0, 0, .3, -.2, 0, -.1, 0, 0, .3, -.2, 0, 0, 0, 0,
                              0,   0, 0, 0,  0,   0, 0,   0, 0, 0,  0,   0, 0, 0};
  return q;
}
Positions loco_pose() {
  auto p = defaults();
  p[16] = .3;
  p[18] = 1;
  p[23] = -.3;
  p[25] = 1;
  return p;
}
std::vector<size_t> mimic_indices() {
  std::vector<std::string> n;
  for (size_t i = 0; i < 29; ++i)
    if (i != 19 && i != 20 && i != 21 && i != 26 && i != 27 && i != 28)
      n.push_back(joint_names()[i]);
  return named_indices(n);
}
const std::vector<size_t>& upper_override_indices(){
 static const auto indices=named_indices({"waist_yaw_joint","left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint","right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint"});return indices;
}
Positions mimic_init() {
  auto p = defaults();
  const double upper[] = {.19964170455932617, .07710712403059006, -.2882401943206787,
                          .21672365069389343, .15629297494888306, -.5167576670646667,
                          .5782126784324646,  .25740593671798706, -.2504104673862457,
                          .22500675916671753, .5127624273300171};
  auto idx = mimic_indices();
  for (size_t i = 0; i < 11; ++i) p[idx[12 + i]] = upper[i];
  return p;
}
Policy::Policy(std::string id, const std::string& path) : id_(std::move(id)) {
  if (std::string(OrtGetApiBase()->GetVersionString()) != "1.20.1")
    throw std::runtime_error("Native ORT runtime version mismatch");
  if (id_ != "asap_loco" && id_ != "asap_cr7") throw std::invalid_argument("Unregistered model");
  options_.SetIntraOpNumThreads(1);
  options_.SetInterOpNumThreads(1);
  options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  session_ = std::make_unique<Ort::Session>(runtime_, path.c_str(), options_);
  Ort::AllocatorWithDefaultOptions a;
  if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1)
    throw std::runtime_error("Expected exactly one input and output");
  auto in = session_->GetInputNameAllocated(0, a), out = session_->GetOutputNameAllocated(0, a);
  auto it = session_->GetInputTypeInfo(0), ot = session_->GetOutputTypeInfo(0);
  auto ti = it.GetTensorTypeAndShapeInfo(), to = ot.GetTensorTypeAndShapeInfo();
  const int64_t obs = id_ == "asap_loco" ? 500 : 380, act = id_ == "asap_loco" ? 12 : 23;
  if (std::string(in.get()) != "actor_obs" || std::string(out.get()) != "action" ||
      ti.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      to.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      ti.GetShape() != std::vector<int64_t>{1, obs} ||
      to.GetShape() != std::vector<int64_t>{1, act})
    throw std::runtime_error("Model signature mismatch: " + id_);
  if (id_ == "asap_loco") {
    indices_.resize(29);
    std::iota(indices_.begin(), indices_.end(), 0);
  } else
    indices_ = mimic_indices();
  reset();
}
void Policy::reset() {
  steps_ = 0;
  motion_done_ = false;
  last_.assign(id_ == "asap_loco" ? 12 : 23, 0);
  history_.clear();
  const std::vector<size_t> dims = id_ == "asap_loco"
                                       ? std::vector<size_t>{12, 3, 1, 1, 2, 1, 1, 29, 29, 3, 17, 1}
                                       : std::vector<size_t>{23, 3, 23, 23, 3, 1};
  Groups blank;
  for (auto n : dims) blank.emplace_back(n, 0);
  for (int i = 0; i < 4; ++i) history_.push_back(blank);
}
std::vector<float> Policy::observe(const Frame& f, const Positions& ref) {
  if (f.schema_version != schema_version || f.schema_id != schema_id || (f.validity & 3) != 3)
    throw std::runtime_error("State schema or validity mismatch");
  if (f.joints.name.size() != 29 || f.joints.position.size() != 29 ||
      f.joints.velocity.size() != 29)
    throw std::runtime_error("State joint dimensions");
  auto map = named_indices(f.joints.name);
  Positions q{}, dq{};
  for (size_t i = 0; i < 29; ++i) {
    q[map[i]] = f.joints.position[i];
    dq[map[i]] = f.joints.velocity[i];
    if (!std::isfinite(q[map[i]]) || !std::isfinite(dq[map[i]]))
      throw std::runtime_error("Nonfinite joint state");
  }
  const auto& v = f.imu.orientation;
  const double norm = v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
  if (!std::isfinite(norm) || std::abs(norm - 1) > .01)
    throw std::runtime_error("Invalid IMU quaternion");
  std::vector<float> gravity = {static_cast<float>(2 * (v.w * v.y - v.x * v.z)),
                                static_cast<float>(-2 * (v.w * v.x + v.y * v.z)),
                                static_cast<float>(-1 + 2 * (v.x * v.x + v.y * v.y))};
  Groups g;
  g.push_back(last_);
  g.push_back({static_cast<float>(.25 * f.imu.angular_velocity.x),
               static_cast<float>(.25 * f.imu.angular_velocity.y),
               static_cast<float>(.25 * f.imu.angular_velocity.z)});
  double phase = 0;
  if (id_ == "asap_loco") {
    if (!(f.validity & Frame::VALID_COMMAND)) throw std::runtime_error("Invalid command");
    const double wall = f.header.stamp.sec + f.header.stamp.nanosec * 1e-9;
    phase = f.command_stand ? std::fmod(wall, .8) / .8 : 0;
    g.push_back({static_cast<float>(f.command.twist.angular.z)});
    g.push_back({static_cast<float>(2 * f.command_base_height)});
    g.push_back({static_cast<float>(f.command.twist.linear.x),
                 static_cast<float>(f.command.twist.linear.y)});
    g.push_back({f.command_stand ? 1.f : 0.f});
    g.push_back({static_cast<float>(std::cos(2 * 3.14159265358979323846 * phase))});
  }
  std::vector<float> qp, dqp;
  for (auto i : indices_) {
    qp.push_back(static_cast<float>(q[i] - defaults()[i]));
    dqp.push_back(static_cast<float>(.05 * dq[i]));
  }
  g.push_back(qp);
  g.push_back(dqp);
  const size_t history_insert = g.size();
  g.push_back(gravity);
  if (id_ == "asap_loco") {
    std::vector<float> upper;
    for (size_t i = 12; i < 29; ++i) upper.push_back(static_cast<float>(ref[i]));
    g.push_back(upper);
    g.push_back({static_cast<float>(std::sin(2 * 3.14159265358979323846 * phase))});
  } else
    g.push_back({static_cast<float>(steps_ * .02 / 3.967)});
  std::vector<float> obs;
  auto add = [&obs](const auto& values) { obs.insert(obs.end(), values.begin(), values.end()); };
  for (size_t k = 0; k < g.size(); ++k) {
    if (k == history_insert)
      for (size_t group = 0; group < g.size(); ++group)
        for (const auto& h : history_) add(h[group]);
    add(g[k]);
  }
  history_.push_front(g);
  history_.pop_back();
  for (auto x : obs)
    if (!std::isfinite(x)) throw std::runtime_error("Nonfinite observation");
  return obs;
}
Result Policy::infer(const Frame& f, const Positions& ref) {
  Result r;
  r.observation = observe(f, ref);
  std::array<int64_t, 2> shape = {1, static_cast<int64_t>(r.observation.size())};
  auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto input = Ort::Value::CreateTensor<float>(mem, r.observation.data(), r.observation.size(),
                                               shape.data(), 2);
  const char* in[] = {"actor_obs"};
  const char* out[] = {"action"};
  auto outputs = session_->Run(Ort::RunOptions{nullptr}, in, &input, 1, out, 1);
  const auto* data = outputs[0].GetTensorData<float>();
  r.raw_action.assign(data, data + last_.size());
  r.q_ref = defaults();
  for (size_t k = 0; k < r.raw_action.size(); ++k) {
    auto a = r.raw_action[k];
    if (!std::isfinite(a)) throw std::runtime_error("Nonfinite model output");
    auto clipped = std::clamp(a, -100.f, 100.f);
    last_[k] = id_ == "asap_loco" ? clipped : a;
    size_t j = id_ == "asap_loco" ? k : indices_[k];
    r.q_ref[j] = defaults()[j] + .25f * clipped;
  }
  return r;
}
void Policy::post_step() {
  if (id_ == "asap_cr7" && steps_ * .02 / 3.967 >= 1.) {
    motion_done_ = true;
  } else {
    ++steps_;
  }
}
bool Policy::motion_done() const { return motion_done_; }
}  // namespace anakin::mc
