#include <anakin_mc/policy.hpp>
#include <anakin_mc/transactions.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
int main(int argc, char** argv) {
  try {
    if (argc != 5)
      throw std::runtime_error("Usage: anakin_mc_trace MODEL_ID MODEL_PATH INPUT OUTPUT");
    anakin::mc::Policy p(argv[1], argv[2]);
    std::ifstream in(argv[3]);
    std::ofstream out(argv[4]);
    if (!in || !out) throw std::runtime_error("Fixture path unavailable");
    out << std::setprecision(17);
    size_t n = 0;
    double wall;
    while (in >> wall) {
      anakin::mc::Frame f;
      f.schema_version = anakin::schema_version;
      f.schema_id = anakin::schema_id;
      f.validity = 7;
      f.header.stamp.sec = static_cast<int32_t>(wall);
      f.header.stamp.nanosec =
          static_cast<uint32_t>(std::llround((wall - f.header.stamp.sec) * 1e9));
      f.joints.name = anakin::joint_names();
      f.joints.position.resize(29);
      f.joints.velocity.resize(29);
      for (auto& v : f.joints.position) in >> v;
      for (auto& v : f.joints.velocity) in >> v;
      in >> f.imu.orientation.x >> f.imu.orientation.y >> f.imu.orientation.z >>
          f.imu.orientation.w >> f.imu.angular_velocity.x >> f.imu.angular_velocity.y >>
          f.imu.angular_velocity.z >> f.command.twist.linear.x >> f.command.twist.linear.y >>
          f.command.twist.angular.z >> f.command_stand >> f.command_base_height;
      anakin::mc::Positions ref;
      for (auto& v : ref) in >> v;
      if (!in) throw std::runtime_error("Truncated fixture row");
      auto r = p.infer(f, ref);
      for (auto v : r.observation) out << v << ' ';
      for (auto v : r.raw_action) out << v << ' ';
      for (auto v : r.q_ref) out << v << ' ';
      out << static_cast<int>(p.motion_done()) << '\n';
      p.post_step();
      ++n;
    }
    if (n == 0) throw std::runtime_error("Empty fixture");
    // Exercise asymmetric name mapping and request conflict semantics.
    auto m = anakin::named_indices(
        {"right_elbow_joint", "left_hip_roll_joint", "right_shoulder_roll_joint"});
    if (m != std::vector<size_t>{25, 1, 23}) throw std::runtime_error("Named map mismatch");
    anakin::mc::Journal j;
    anakin::mc::Switch::Request q;
    q.session_id = anakin::new_uuid();
    q.request_id = anakin::new_uuid();
    q.target_model = "asap_loco";
    anakin::mc::Switch::Response a;
    a.accepted = true;
    j.record(q, a);
    if (!j.find(q) || !(j.find(q)->request == q)) throw std::runtime_error("Retry lost");
    q.target_model = "asap_cr7";
    if (j.find(q)->request == q) throw std::runtime_error("Conflict not distinguished");
    std::cout << "Native trace frames=" << n << " model=" << argv[1]
              << " mapping/retry/conflict=PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
