#include <anakin_mc/transition.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
int main(int argc, char** argv) {
  try {
    if (argc != 4) throw std::runtime_error("Usage: transition_trace enter|return INPUT OUTPUT");
    bool enter = std::string(argv[1]) == "enter";
    anakin::mc::TransitionTimeline timeline;
    timeline.begin(enter, enter ? anakin::mc::loco_pose() : anakin::mc::mimic_init(),
                   enter ? anakin::mc::mimic_init() : anakin::mc::loco_pose());
    std::ifstream in(argv[2]);
    std::ofstream out(argv[3]);
    out << std::setprecision(17);
    anakin::mc::Positions measured;
    size_t frames = 0;
    while (in >> measured[0]) {
      for (size_t i = 1; i < 29; ++i) in >> measured[i];
      if (!in) throw std::runtime_error("Truncated transition fixture");
      out << timeline.target_controls() << ' ' << timeline.complete() << ' ';
      for (auto q : timeline.upper()) out << q << ' ';
      out << '\n';
      if (!enter && frames < 25 && timeline.target_controls())
        throw std::runtime_error("Loco took control before 25 warmup steps");
      timeline.post_step(measured);
      ++frames;
    }
    if (frames < 110) throw std::runtime_error("Transition fixture too short");
    std::cout << "Post-step transition trace frames=" << frames << " direction=" << argv[1] << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
