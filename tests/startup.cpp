#include <anakin_mc/transition.hpp>
#include <cmath>
#include <iostream>
using namespace anakin::mc;
void check(bool ok, const char* why) {
  if (!ok) throw std::runtime_error(why);
}
int main() {
  try {
    Positions measured{}, target{};
    measured[0] = -.1;
    measured[3] = .3;
    measured[25] = .2;
    target[0] = .2;
    target[3] = .9;
    target[25] = 1.1;
    StartupTimeline startup;
    startup.begin(measured);
    check(startup.apply(target) == measured,
          "First startup command equals its measured source pose");
    check(!startup.complete(), "Warm model does not imply completed canonical transition");
    for (int step = 0; step < 5; ++step) startup.post_step();
    const auto half = startup.apply(target);
    check(std::abs(half[0] - .05) < 1e-12 && std::abs(half[3] - .6) < 1e-12 &&
              std::abs(half[25] - .65) < 1e-12,
          "Asymmetric canonical-position midpoint oracle");
    check(!startup.complete(), "No early active phase at midpoint");
    for (int step = 5; step < 10; ++step) startup.post_step();
    check(startup.complete() && startup.apply(target) == target,
          "Full model target only after ten publication callbacks");
    measured[25] = -.4;
    startup.begin(measured);
    check(!startup.complete() && startup.apply(target) == measured,
          "New transaction resets measured source and callback schedule");
    std::cout
        << "PASS canonical startup first/midpoint/terminal boundaries and transaction reset\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
