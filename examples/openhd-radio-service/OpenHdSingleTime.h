#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

// Local OpenHD executor mathematics. This file does not read OpenHD JSON or
// carry a schedule over the air; the OpenHD process sends one validated plan.
namespace openhd_single_time {

constexpr std::uint8_t kGroundFlag = 1;
constexpr std::uint8_t kFhssFlag = 2;
constexpr std::uint8_t kTdmaFlag = 4;
constexpr std::uint32_t kTdmaSlotUs = 20000;
constexpr std::uint32_t kTdmaGuardUs = 2000;
constexpr std::uint64_t kTdmaLeaseUs = 2000000;

struct Plan {
  std::uint32_t period_us = 0;
  std::uint32_t dwell_us = 0;
  std::uint32_t pre_guard_us = 0;
  std::uint32_t post_guard_us = 0;
  std::uint64_t seed = 0;
  std::uint32_t channel_count = 0;
  std::array<std::uint32_t, 64> frequencies_mhz{};
  std::uint32_t peer_timeout_us = 0;
  std::uint8_t own_id = 255;
  std::uint8_t flags = 0;
};

inline bool valid(const Plan& plan) {
  if (plan.period_us < 1800000000U || plan.period_us > 3600000000U ||
      plan.dwell_us < 50000U || plan.dwell_us > 1000000U ||
      plan.period_us % plan.dwell_us != 0 ||
      plan.channel_count == 0 || plan.channel_count > 64 ||
      plan.pre_guard_us >= plan.dwell_us ||
      plan.post_guard_us >= plan.dwell_us - plan.pre_guard_us ||
      plan.dwell_us - plan.pre_guard_us - plan.post_guard_us < 13000U ||
      plan.peer_timeout_us < 1000000U ||
      plan.own_id == 255 || plan.flags > 7 ||
      (((plan.flags & kFhssFlag) == 0) && plan.channel_count != 1) ||
      (((plan.flags & kFhssFlag) != 0) && plan.channel_count < 2))
    return false;
  for (std::uint32_t i = 0; i < plan.channel_count; ++i) {
    const auto frequency = plan.frequencies_mhz[i];
    if (frequency < 5000 || frequency > 6000 || frequency % 5 != 0)
      return false;
    for (std::uint32_t j = 0; j < i; ++j)
      if (plan.frequencies_mhz[j] == frequency) return false;
  }
  return true;
}

inline std::uint32_t phase_at(std::uint32_t sample_phase_us,
                              std::uint64_t elapsed_us,
                              std::uint32_t period_us) {
  if (period_us == 0) return 0;
  return static_cast<std::uint32_t>(
      (static_cast<std::uint64_t>(sample_phase_us) +
       elapsed_us % period_us) % period_us);
}

inline std::uint64_t next_random(std::uint64_t& state) {
  auto value = (state += 0x9e3779b97f4a7c15ULL);
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

inline std::uint32_t channel_index(const Plan& plan,
                                   std::uint32_t phase_us) {
  if (plan.channel_count == 0 || plan.dwell_us == 0 || plan.period_us == 0)
    return 0;
  const auto slot = (phase_us % plan.period_us) / plan.dwell_us;
  const auto group = slot / plan.channel_count;
  const auto position = slot % plan.channel_count;
  if (position == 0 || plan.channel_count == 1) return 0;
  std::array<std::uint8_t, 64> order{};
  for (std::uint32_t i = 0; i < plan.channel_count; ++i)
    order[i] = static_cast<std::uint8_t>(i);
  std::uint64_t state = plan.seed ^
      (static_cast<std::uint64_t>(group) * 0xa0761d6478bd642fULL);
  for (std::uint32_t i = plan.channel_count; i > 2; --i) {
    const auto j = 1U + static_cast<std::uint32_t>(
        next_random(state) % (i - 1));
    const auto old = order[i - 1];
    order[i - 1] = order[j];
    order[j] = old;
  }
  return order[position];
}

inline bool tdma_layout(const std::array<std::uint64_t, 4>& members,
                        std::uint8_t own_id,
                        std::uint16_t& count,
                        std::uint16_t& seat) {
  const auto has = [&members](std::uint16_t id) {
    return (members[id / 64] & (1ULL << (id % 64))) != 0;
  };
  if (own_id == 255 || has(255) || !has(own_id)) return false;
  count = 0;
  seat = 0;
  for (std::uint16_t id = 0; id < 255; ++id) {
    if (!has(id)) continue;
    if (id < own_id) ++seat;
    ++count;
  }
  return count != 0;
}

inline bool tdma_data_allowed(std::uint32_t phase_us,
                              std::uint16_t count,
                              std::uint16_t seat,
                              std::uint32_t airtime_us) {
  if (count == 0 || count > 255 || seat >= count ||
      airtime_us >= kTdmaSlotUs - 2 * kTdmaGuardUs)
    return false;
  const auto position = phase_us % (count * kTdmaSlotUs);
  const auto begin = seat * kTdmaSlotUs + kTdmaGuardUs;
  const auto end = (seat + 1) * kTdmaSlotUs - kTdmaGuardUs;
  return position >= begin && position < end && airtime_us <= end - position;
}

// OpenHD's normal timed TX uses a 13-byte NOACK TX_FLAGS+MCS radiotap header.
// Reserve airtime at the slowest HT20 rate (MCS0) plus queue margin, matching
// the conservative bound in the kernel path. Unknown formats fail closed.
inline std::optional<std::uint32_t> tx_airtime_us(
    const std::uint8_t* frame, std::size_t length) {
  if (!frame || length < 13 + 24 || length > 4096 ||
      frame[0] != 0 || frame[1] != 0 || frame[2] != 13 || frame[3] != 0 ||
      frame[4] != 0 || frame[5] != 0x80 ||
      frame[6] != 0x08 || frame[7] != 0 ||
      frame[8] != 0x08 || frame[9] != 0 ||
      !(frame[10] & 0x02) || frame[12] > 7)
    return std::nullopt;
  const auto mpdu_bytes = static_cast<std::uint32_t>(length - 13);
  return 500U + ((mpdu_bytes + 64U) * 80U + 64U) / 65U;
}

enum class RunState : std::uint32_t {
  Fixed = 0,
  WaitSync = 2,
  Running = 3,
  Recovery = 4,
  Fault = 5,
};

struct Status {
  std::uint32_t configured = 0;
  std::uint32_t synchronized = 0;
  std::uint32_t tx_gated = 1;
  std::uint32_t target_frequency_mhz = 0;
  std::uint32_t slot = 0;
  RunState state = RunState::Fixed;
};

struct Retune {
  std::uint32_t frequency_mhz = 0;
  std::uint32_t slot = 0;
};

// One local clock/slot state machine. The service performs any requested RF
// operation and calls retune_complete; the state machine never owns USB I/O.
class Scheduler {
 public:
  bool configure(const Plan& plan, std::uint32_t tuned_frequency_mhz) {
    if (!valid(plan) || tuned_frequency_mhz != plan.frequencies_mhz[0])
      return false;
    plan_ = plan;
    configured_ = true;
    synchronized_ = false;
    sample_local_us_ = 0;
    sample_phase_us_ = 0;
    peer_received_us_ = 0;
    members_updated_us_ = 0;
    members_ = {};
    active_frequency_mhz_ = tuned_frequency_mhz;
    active_slot_ = 0;
    retune_end_us_ = 0;
    fault_ = false;
    return true;
  }

  void stop() { *this = Scheduler{}; }

  bool set_phase(std::uint64_t now_us, std::uint64_t sample_local_us,
                 std::uint32_t phase_us) {
    if (!configured_ || sample_local_us == 0 || sample_local_us > now_us ||
        now_us - sample_local_us > 1000000ULL ||
        phase_us >= plan_.period_us ||
        (synchronized_ && sample_local_us < sample_local_us_))
      return false;
    sample_local_us_ = sample_local_us;
    sample_phase_us_ = phase_us;
    synchronized_ = true;
    return true;
  }

  bool set_peer_lease(std::uint64_t now_us,
                      std::uint64_t received_local_us) {
    if (!configured_ || received_local_us == 0 ||
        received_local_us > now_us ||
        now_us - received_local_us > 1000000ULL)
      return false;
    if (received_local_us > peer_received_us_)
      peer_received_us_ = received_local_us;
    return true;
  }

  bool set_members(std::uint64_t now_us,
                   const std::array<std::uint64_t, 4>& members) {
    if (!configured_ || !((plan_.flags & kTdmaFlag) != 0)) return false;
    std::uint16_t count = 0, seat = 0;
    if (!tdma_layout(members, plan_.own_id, count, seat)) return false;
    members_ = members;
    members_updated_us_ = now_us;
    return true;
  }

  bool wait_first() {
    if (!configured_ || !(plan_.flags & kFhssFlag) ||
        (plan_.flags & kGroundFlag))
      return false;
    synchronized_ = false;
    peer_received_us_ = 0;
    return true;
  }

  std::optional<Retune> step(std::uint64_t now_us) {
    if (!configured_ || !(plan_.flags & kFhssFlag)) return std::nullopt;
    if (fault_) return std::nullopt;
    if (synchronized_ && !(plan_.flags & kGroundFlag) &&
        (peer_received_us_ == 0 || now_us < peer_received_us_ ||
         now_us - peer_received_us_ > plan_.peer_timeout_us)) {
      synchronized_ = false;
    }
    if (!synchronized_ || now_us < sample_local_us_) {
      if (active_frequency_mhz_ != plan_.frequencies_mhz[0])
        return Retune{plan_.frequencies_mhz[0], 0};
      return std::nullopt;
    }
    const auto phase = phase_at(sample_phase_us_,
                                now_us - sample_local_us_, plan_.period_us);
    const auto slot = phase / plan_.dwell_us;
    const auto position = phase % plan_.dwell_us;
    auto target_slot = slot;
    if (plan_.pre_guard_us &&
        position >= plan_.dwell_us - plan_.pre_guard_us)
      target_slot = (slot + 1) % (plan_.period_us / plan_.dwell_us);
    const auto frequency = plan_.frequencies_mhz[
        channel_index(plan_, target_slot * plan_.dwell_us)];
    if (active_frequency_mhz_ != frequency ||
        active_slot_ != target_slot)
      return Retune{frequency, target_slot};
    return std::nullopt;
  }

  void retune_complete(const Retune& request, std::uint64_t now_us,
                       bool success) {
    if (!configured_) return;
    if (fault_) return;
    if (!success) {
      fault_ = true;
      return;
    }
    active_frequency_mhz_ = request.frequency_mhz;
    active_slot_ = request.slot;
    retune_end_us_ = now_us;
    fault_ = false;
  }

  Status status(std::uint64_t now_us) const {
    Status out;
    out.configured = configured_ ? 1U : 0U;
    out.synchronized = synchronized_ ? 1U : 0U;
    out.target_frequency_mhz = active_frequency_mhz_;
    out.slot = active_slot_;
    if (!configured_) return out;
    if (fault_) {
      out.state = RunState::Fault;
      return out;
    }
    if (!(plan_.flags & kFhssFlag)) {
      out.state = RunState::Fixed;
      out.tx_gated = 0;
      return out;
    }
    if (!synchronized_ || now_us < sample_local_us_ ||
        (!(plan_.flags & kGroundFlag) &&
         (peer_received_us_ == 0 || now_us < peer_received_us_ ||
          now_us - peer_received_us_ > plan_.peer_timeout_us))) {
      out.state = RunState::WaitSync;
      return out;
    }
    out.state = RunState::Running;
    const auto phase = phase_at(sample_phase_us_,
                                now_us - sample_local_us_, plan_.period_us);
    const auto slot = phase / plan_.dwell_us;
    const auto position = phase % plan_.dwell_us;
    if (slot == active_slot_ &&
        active_frequency_mhz_ ==
            plan_.frequencies_mhz[channel_index(plan_,
                                                slot * plan_.dwell_us)] &&
        position >= plan_.post_guard_us &&
        position < plan_.dwell_us - plan_.pre_guard_us &&
        now_us >= retune_end_us_ + plan_.post_guard_us)
      out.tx_gated = 0;
    return out;
  }

  bool can_tx(std::uint64_t now_us, bool control_beacon,
              std::uint32_t airtime_us) const {
    const auto current = status(now_us);
    if (!configured_ || current.tx_gated) return false;
    if (plan_.flags & kFhssFlag) {
      const auto phase = phase_at(sample_phase_us_,
                                  now_us - sample_local_us_, plan_.period_us);
      const auto position = phase % plan_.dwell_us;
      if (airtime_us > plan_.dwell_us - plan_.pre_guard_us - position)
        return false;
    }
    if (control_beacon || !(plan_.flags & kTdmaFlag)) return true;
    if (!synchronized_ || members_updated_us_ == 0 ||
        now_us < members_updated_us_ ||
        now_us - members_updated_us_ > kTdmaLeaseUs)
      return false;
    std::uint16_t count = 0, seat = 0;
    if (!tdma_layout(members_, plan_.own_id, count, seat)) return false;
    const auto phase = phase_at(sample_phase_us_,
                                now_us - sample_local_us_, plan_.period_us);
    return tdma_data_allowed(phase, count, seat, airtime_us);
  }

  bool configured() const { return configured_; }
  bool hopping() const { return configured_ && (plan_.flags & kFhssFlag); }

 private:
  Plan plan_{};
  bool configured_ = false;
  bool synchronized_ = false;
  bool fault_ = false;
  std::uint64_t sample_local_us_ = 0;
  std::uint32_t sample_phase_us_ = 0;
  std::uint64_t peer_received_us_ = 0;
  std::array<std::uint64_t, 4> members_{};
  std::uint64_t members_updated_us_ = 0;
  std::uint32_t active_frequency_mhz_ = 0;
  std::uint32_t active_slot_ = 0;
  std::uint64_t retune_end_us_ = 0;
};

}  // namespace openhd_single_time
