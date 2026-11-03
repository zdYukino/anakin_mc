#include <anakin_mc/policy.hpp>
#include <anakin_sdk/contract.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace anakin::mc;
void check(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("Expected loco and mimic model paths");
    Policy loco("asap_loco", argv[1]), mimic("asap_cr7", argv[2]);
    Frame frame;
    frame.schema_version = anakin::schema_version;
    frame.schema_id = anakin::schema_id;
    frame.validity = 7;
    frame.joints.name = anakin::joint_names();
    frame.joints.position.assign(defaults().begin(), defaults().end());
    frame.joints.velocity.assign(29, 0.);
    frame.imu.orientation.w = 1.;
    frame.command_base_height = .78;
    Positions measured{}, command{};
    for (size_t i = 0; i < measured.size(); ++i) {
      measured[i] = .01 * static_cast<double>(i + 1);
      command[i] = -.03 * static_cast<double>(i + 1);
    }
    const Positions loco_reference = loco.uncontrolled_reference(measured);
    const Positions mimic_reference = mimic.uncontrolled_reference(measured);
    const std::array<double, 17> standing_upper = {
        0., 0., 0., 0., .3, 0., 1., 0., 0., 0., 0., -.3, 0., 1., 0., 0., 0.};
    for (size_t i = 0; i < standing_upper.size(); ++i)
      check(loco_reference[12 + i] == standing_upper[i], "Loco standing reference changed");
    check(mimic_reference == measured, "Other policies retain their own captured reference");
    auto loco_command = command;
    loco.apply_uncontrolled_reference(loco_command, loco_reference);
    for (size_t i = 0; i < loco_command.size(); ++i)
      check(loco_command[i] == (i < 12 ? command[i] : standing_upper[i - 12]),
            "Loco policy controls its action joints and sets remaining joints to stand");
    auto mimic_command = command;
    mimic.apply_uncontrolled_reference(mimic_command, mimic_reference);
    for (size_t i = 0; i < mimic_command.size(); ++i) {
      const bool wrist = (i >= 19 && i <= 21) || (i >= 26 && i <= 28);
      check(mimic_command[i] == (wrist ? measured[i] : command[i]),
            "Mimic holds only its unowned joints");
    }

    // Running the source policy must not inject its actions into the target's
    // previous-action observation; passive history records real state frames.
    for (int i = 0; i < 5; ++i) {
      (void)mimic.infer(frame, measured);
      mimic.post_step();
      const auto observation = loco.observe(frame, loco_reference);
      check(observation.size() == 500, "Loco observation size changed");
      for (size_t action = 0; action < 12; ++action)
        check(observation[action] == 0.f, "Inactive loco received another policy's action");
      for (size_t joint = 0; joint < standing_upper.size(); ++joint)
        check(std::abs(observation[482 + joint] - static_cast<float>(standing_upper[joint])) < 1e-6f,
              "Loco current observation must contain the commanded standing reference");
      if (i == 4)
        for (size_t sample = 0; sample < 4; ++sample)
          for (size_t joint = 0; joint < standing_upper.size(); ++joint)
            check(std::abs(observation[407 + sample * 17 + joint] -
                           static_cast<float>(standing_upper[joint])) < 1e-6f,
                  "Prewarmed loco history must contain the same standing reference");
    }
    const auto first = loco.infer(frame, loco_reference);
    check(first.raw_action.size() == 12, "Loco action size changed");
    const auto second = loco.observe(frame, loco_reference);
    for (size_t i = 0; i < first.raw_action.size(); ++i)
      check(std::abs(second[i] - std::clamp(first.raw_action[i], -100.f, 100.f)) < 1e-6f,
            "Active loco history must use its own previous inference");
    std::cout << "Loco stand targets, observed reference and independent histories passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
