#pragma once

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace manipulator {
// Configuration comes from the same launch/YAML values as the Python guard.
struct SpeedSafetyConfig {
  double transition_warning, transition_abort, guide_warning, sustained_speed,
      sustained_duration, instant_speed;
  void Validate() const {
    for (double v : {transition_warning, transition_abort, guide_warning,
                     sustained_speed, sustained_duration, instant_speed})
      if (!std::isfinite(v) || v <= 0) throw std::invalid_argument("invalid speed parameter");
    if (!(transition_warning <= transition_abort && transition_abort <= instant_speed &&
          guide_warning < sustained_speed && sustained_speed < instant_speed))
      throw std::invalid_argument("invalid speed threshold ordering");
  }
};

class SpeedSafety {
 public:
  explicit SpeedSafety(SpeedSafetyConfig cfg) : config(cfg) {
    config.Validate();
    since.fill(-1.);
  }
  void SetManual(const std::vector<double>& w, const std::vector<double>& s,
                 const std::vector<double>& h, double duration_seconds) {
    if (w.size()!=6 || s.size()!=6 || h.size()!=6 || !std::isfinite(duration_seconds) || duration_seconds<=0)
      throw std::invalid_argument("invalid manual speed profile");
    for (size_t i=0;i<6;++i) {
      if (!std::isfinite(w[i]) || !std::isfinite(s[i]) || !std::isfinite(h[i]) ||
          !(0<w[i] && w[i]<s[i] && s[i]<h[i])) throw std::invalid_argument("invalid manual speed order");
      manual_warning[i]=w[i]; manual_sustained[i]=s[i]; manual_instant[i]=h[i];
    }
    manual_duration=duration_seconds; manual_configured=true;
  }
  void Update(const std::vector<double>& dq, bool guide, double now, bool manual_requested=false) {
    if (fault) return;
    if (dq.size() != 6 || !std::isfinite(now) || (last >= 0 && now < last))
      throw std::invalid_argument("invalid speed sample/time");
    for (double v : dq) if (!std::isfinite(v)) throw std::invalid_argument("invalid speed sample");
    const bool manual = guide && manual_requested;
    if (manual && !manual_configured) throw std::invalid_argument("manual profile not configured");
    if (guide != guide_phase || manual != manual_phase) since.fill(-1.);
    guide_phase = guide;
    manual_phase = manual;
    last = now;
    for (size_t i = 0; i < 6; ++i) {
      const double speed = std::abs(dq[i]);
      warning[i] = speed > (manual ? manual_warning[i] : guide ? config.guide_warning : config.transition_warning);
      if (guide && speed > (manual ? manual_sustained[i] : config.sustained_speed)) {
        if (since[i] < 0) since[i] = now;
      } else since[i] = -1.;
      duration[i] = since[i] < 0 ? 0. : now - since[i];
      type[i] = guide && speed > (manual ? manual_instant[i] : config.instant_speed) ? "instantaneous" :
          guide && since[i] >= 0 && duration[i] >= (manual ? manual_duration : config.sustained_duration) ? "sustained" :
          !guide && speed > config.transition_abort ? "transition" : "";
      axes[i] = !type[i].empty();
      fault = fault || axes[i];
    }
  }
  SpeedSafetyConfig config;
  std::array<bool, 6> warning{}, axes{};
  std::array<double, 6> duration{}, since{};
  std::array<std::string, 6> type{};
  bool fault = false, guide_phase = false;
  bool manual_phase = false, manual_configured = false;
  std::array<double,6> manual_warning{}, manual_sustained{}, manual_instant{};
  double manual_duration = 0.;
  double last = -1.;
};
}  // namespace manipulator
