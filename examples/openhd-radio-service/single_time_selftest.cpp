#include "OpenHdSingleTime.h"

#include <array>
#include <cassert>
#include <cstdint>

int main() {
  openhd_single_time::Plan plan{};
  plan.period_us = 1800000000U;
  plan.dwell_us = 100000U;
  plan.pre_guard_us = 8000U;
  plan.post_guard_us = 3000U;
  plan.seed = 20260927U;
  plan.channel_count = 4;
  plan.frequencies_mhz = {5180U, 5200U, 5220U, 5240U};
  plan.peer_timeout_us = 10000000U;
  plan.own_id = 1;
  plan.flags = openhd_single_time::kFhssFlag |
               openhd_single_time::kTdmaFlag;
  assert(openhd_single_time::valid(plan));
  // Fixed vectors also checked against OpenHD's current driver schedule.
  constexpr std::array<std::uint32_t, 24> expected{
      0, 2, 1, 3, 0, 3, 1, 2, 0, 3, 2, 1,
      0, 3, 2, 1, 0, 3, 2, 1, 0, 2, 3, 1};
  for (std::size_t slot = 0; slot < expected.size(); ++slot)
    assert(openhd_single_time::channel_index(
               plan, static_cast<std::uint32_t>(slot * plan.dwell_us)) ==
           expected[slot]);
  assert(openhd_single_time::phase_at(1799999000U, 2000U,
                                     plan.period_us) == 1000U);

  std::array<std::uint64_t, 4> members{};
  members[0] = (1ULL << 1) | (1ULL << 6) | (1ULL << 32);
  std::uint16_t count = 0, seat = 0;
  assert(openhd_single_time::tdma_layout(members, 6, count, seat));
  assert(count == 3 && seat == 1);
  assert(openhd_single_time::tdma_data_allowed(22000U, count, seat, 1000U));
  assert(!openhd_single_time::tdma_data_allowed(21999U, count, seat, 1000U));
  assert(!openhd_single_time::tdma_data_allowed(22000U, count, seat, 16000U));
  std::array<std::uint8_t, 1013> ht_frame{};
  const std::array<std::uint8_t, 13> rtap{
      0x00, 0x00, 0x0d, 0x00, 0x00, 0x80, 0x08,
      0x00, 0x08, 0x00, 0x37, 0x30, 0x03};
  for (std::size_t i = 0; i < rtap.size(); ++i) ht_frame[i] = rtap[i];
  const auto reserved = openhd_single_time::tx_airtime_us(
      ht_frame.data(), ht_frame.size());
  assert(reserved && *reserved == 1810U);
  ht_frame[12] = 9;
  assert(!openhd_single_time::tx_airtime_us(
      ht_frame.data(), ht_frame.size()));
  ht_frame[12] = 3;
  ht_frame[8] = 0;
  assert(!openhd_single_time::tx_airtime_us(
      ht_frame.data(), ht_frame.size()));
  ht_frame[8] = 0x08;

  using openhd_single_time::RunState;
  using openhd_single_time::Scheduler;
  Scheduler air;
  assert(air.configure(plan, 5180U));
  assert(air.status(1000000U).state == RunState::WaitSync);
  assert(!air.can_tx(1000000U, true, 1000U));
  assert(!air.set_phase(1000000U, 0U, 0U));
  assert(air.set_phase(1000000U, 1000000U, 0U));
  assert(air.set_peer_lease(1000000U, 1000000U));
  assert(air.set_members(1000000U, members));
  assert(air.can_tx(1004000U, true, 1000U));
  assert(air.can_tx(1004000U, false, 1000U));
  assert(!air.can_tx(1023000U, false, 1000U));
  assert(!air.can_tx(1017000U, false, 2000U));
  const auto next = air.step(1093000U);
  assert(next && next->frequency_mhz == 5220U && next->slot == 1);
  assert(air.status(1093000U).tx_gated);
  air.retune_complete(*next, 1093500U, false);
  assert(air.status(1093600U).state == RunState::Fault);
  assert(!air.can_tx(1093600U, true, 100U));
  assert(!air.step(1093700U));
  air.retune_complete(*next, 1094000U, true);
  assert(air.status(1104000U).state == RunState::Fault);
  assert(air.configure(plan, 5180U));
  assert(air.set_phase(1100000U, 1100000U, 100000U));
  assert(air.set_peer_lease(1100000U, 1100000U));
  assert(air.set_members(1100000U, members));
  const auto resumed = air.step(1100000U);
  assert(resumed && resumed->frequency_mhz == 5220U);
  air.retune_complete(*resumed, 1100500U, true);
  assert(air.status(1104000U).state == RunState::Running);
  assert(air.can_tx(1104000U, true, 1000U));
  assert(!air.can_tx(1104000U, false, 1000U));
  assert(air.status(12000000U).state == RunState::WaitSync);
  assert(!air.can_tx(12000000U, true, 100U));
  const auto first = air.step(12000000U);
  assert(first && first->frequency_mhz == 5180U);
  air.retune_complete(*first, 12002000U, true);
  assert(!air.can_tx(12010000U, true, 100U));

  plan.flags = openhd_single_time::kTdmaFlag |
               openhd_single_time::kGroundFlag;
  plan.channel_count = 1;
  Scheduler fixed;
  assert(fixed.configure(plan, 5180U));
  assert(fixed.can_tx(1000000U, true, 1000U));
  assert(!fixed.can_tx(1000000U, false, 1000U));
  assert(fixed.set_phase(1000000U, 1000000U, 0U));
  assert(fixed.set_members(1000000U, members));
  assert(fixed.can_tx(1004000U, false, 1000U));
  assert(!fixed.can_tx(4000000U, false, 1000U));
  fixed.stop();
  assert(!fixed.can_tx(4000000U, true, 1000U));

  // All four selectable modes accept the same plan shape with only the
  // FHSS/TDMA flags and frequency count changed. An unsupported combination
  // must be rejected before USB TX, not silently downgraded.
  for (const bool fhss : {false, true}) {
    for (const bool tdma : {false, true}) {
      auto mode = plan;
      mode.flags = openhd_single_time::kGroundFlag |
                   (fhss ? openhd_single_time::kFhssFlag : 0) |
                   (tdma ? openhd_single_time::kTdmaFlag : 0);
      mode.channel_count = fhss ? 2 : 1;
      mode.own_id = 1;
      assert(openhd_single_time::valid(mode));
      Scheduler executor;
      assert(executor.configure(mode, mode.frequencies_mhz[0]));
      if (fhss || tdma)
        assert(executor.set_phase(1000000U, 1000000U, 0U));
      if (tdma)
        assert(executor.set_members(1000000U, members));
      assert(executor.can_tx(1004000U, true, 1000U));
      assert(executor.can_tx(1004000U, false, 1000U));
    }
  }

  // Reject malformed plans at the executor boundary, including combinations
  // that could otherwise look like a valid fixed-frequency fallback.
  auto invalid = plan;
  invalid.flags = openhd_single_time::kGroundFlag;
  invalid.channel_count = 2;
  assert(!openhd_single_time::valid(invalid));
  invalid = plan;
  invalid.flags = openhd_single_time::kFhssFlag;
  invalid.channel_count = 1;
  assert(!openhd_single_time::valid(invalid));
  invalid = plan;
  invalid.flags = openhd_single_time::kFhssFlag;
  invalid.channel_count = 2;
  invalid.frequencies_mhz[1] = invalid.frequencies_mhz[0];
  assert(!openhd_single_time::valid(invalid));
  invalid = plan;
  invalid.dwell_us = 0;
  assert(!openhd_single_time::valid(invalid));
  invalid = plan;
  invalid.pre_guard_us = invalid.dwell_us - 10000U;
  assert(!openhd_single_time::valid(invalid));
  invalid = plan;
  invalid.frequencies_mhz[0] = 4995U;
  assert(!openhd_single_time::valid(invalid));
  return 0;
}
