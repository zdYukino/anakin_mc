#pragma once
#include <anakin_sdk/contract.hpp>
#include <anakin_sdk/srv/switch_inference_mode.hpp>
#include <map>
namespace anakin::mc {
using Switch = anakin_sdk::srv::SwitchInferenceMode;
// Never evict an ID within a live session: refuse when the finite journal fills.
class Journal {
 public:
  struct Entry {
    Switch::Request request;
    Switch::Response response;
  };
  std::map<std::string, Entry> entries;
  const Entry* find(const Switch::Request& r) const {
    auto p = entries.find(uuid_string(r.session_id) + uuid_string(r.request_id));
    return p == entries.end() ? nullptr : &p->second;
  }
  bool full() const { return entries.size() >= 128; }
  void record(const Switch::Request& r, const Switch::Response& s) {
    entries.emplace(uuid_string(r.session_id) + uuid_string(r.request_id), Entry{r, s});
  }
};
}  // namespace anakin::mc
