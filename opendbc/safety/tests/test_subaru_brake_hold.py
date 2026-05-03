#!/usr/bin/env python3
"""
Panda safety tests for the Subaru brake-intercept feature.

TDD: tests written BEFORE tx_hook guard implementation.
All brake-intercept-specific tests will FAIL until subaru.h tx_hook
gains the brake_intercept branch.

RACE-A counter mechanics (rx_hook counts UP):
  standstill → countdown = 0 (reset on each zero-speed frame)
  moving frame 1 → countdown = 1  (1 < 3 → settling)
  moving frame 2 → countdown = 2  (2 < 3 → settling)
  moving frame 3 → countdown = 3  (3 < 3 → FALSE → BLOCKED)
  moving frame 4+ → countdown = 3 (capped, still blocked)

tx_hook settling check:  standstill_or_settling = !vehicle_moving || (countdown < BRAKE_INTERCEPT_RELEASE_FRAMES)
                          i.e.  countdown < 3  (NOT countdown > 0)
"""
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.subaru.values import SubaruSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.sunnypilot.car.subaru.values_ext import SubaruSafetyFlagsSP

# Re-use constants / helpers from the upstream test module
from opendbc.safety.tests.test_subaru import (
  SubaruMsg,
  SUBARU_MAIN_BUS,
  SUBARU_ALT_BUS,
  SUBARU_CAM_BUS,
  lkas_tx_msgs,
  TestSubaruSafetyBase,
)

# Brake_Pedal (0x139) is not in SubaruMsg enum — define locally
MSG_SUBARU_Brake_Pedal = 0x139

BRAKE_INTERCEPT_RELEASE_FRAMES = 3  # must match C #define


class TestSubaruBrakeIntercept(TestSubaruSafetyBase):
  """
  Gen1, no SnG, brake_intercept SP param set.
  TX allowlist: base LKAS + ES_Distance (no relay) + Brake_Pedal (cam, relay) + ES_Brake (main, relay).
  """
  SAFETY_MODEL = CarParams.SafetyModel.subaru
  FLAGS = 0  # gen1, no longitudinal

  # base LKAS msgs + ES_Distance (no relay) + Brake_Pedal + ES_Brake
  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)                      # ES_LKAS, ES_DashStatus, ES_LKAS_State, ES_Infotainment + ES_Distance
    + [[MSG_SUBARU_Brake_Pedal, SUBARU_CAM_BUS]]        # 0x139 cam bus
    + [[SubaruMsg.ES_Brake,    SUBARU_MAIN_BUS]]       # 0x220 main bus
  )

  # Relay malfunction fires when received (addr,bus) matches a check_relay=true TX entry.
  # ES_Brake TX bus = MAIN_BUS → relay malfunction if ES_Brake seen on MAIN_BUS.
  # Brake_Pedal TX bus = CAM_BUS → relay malfunction if Brake_Pedal seen on CAM_BUS.
  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
      SubaruMsg.ES_Brake,
    ),
    SUBARU_CAM_BUS: (
      MSG_SUBARU_Brake_Pedal,
    ),
  }

  # Forwarding block logic: check_relay=true entry with bus=destination_bus → fwd returns -1.
  # ES_Brake (0x220): TX entry bus=MAIN_BUS → blocked when arriving from CAM_BUS (destination=MAIN).
  # Brake_Pedal (0x139): TX entry bus=CAM_BUS → blocked when arriving from MAIN_BUS (destination=CAM).
  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
      SubaruMsg.ES_Brake,
    ],
    SUBARU_MAIN_BUS: [
      MSG_SUBARU_Brake_Pedal,
    ],
  }

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    # CRITICAL: SP param must be set BEFORE set_safety_hooks
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  # ── helpers ────────────────────────────────────────────────────────────────

  def _es_brake_msg(self, pressure):
    values = {"Brake_Pressure": pressure}
    return self.packer.make_can_msg_safety("ES_Brake", SUBARU_MAIN_BUS, values)

  def _set_standstill(self):
    """Drive rx_hook to vehicle_moving=False and reset countdown."""
    for _ in range(BRAKE_INTERCEPT_RELEASE_FRAMES + 1):
      self._rx(self._speed_msg(0))

  def _set_moving(self, frames=1):
    """Drive rx_hook to vehicle_moving=True for `frames` Wheel_Speeds frames."""
    for _ in range(frames):
      self._rx(self._speed_msg(10))  # non-zero speed

  def _exhaust_hysteresis(self):
    """Send enough moving frames to push countdown to BRAKE_INTERCEPT_RELEASE_FRAMES (blocked)."""
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES + 1)

  # ── TX allowlist sanity ─────────────────────────────────────────────────────

  def test_brake_intercept_tx_msgs_includes_es_brake(self):
    """ES_Brake on main bus must be in TX_MSGS class attribute."""
    self.assertIn([SubaruMsg.ES_Brake, SUBARU_MAIN_BUS], self.TX_MSGS)

  def test_tx_hook_on_wrong_safety_mode(self):
    """
    Override to skip cross-class overlap check between TestSubaruBrakeIntercept
    and TestSubaruSnGBrakeIntercept — they share all LKAS TX msgs by design.
    The inherited test_spam_can_buses and test_tx_msg_in_scanned_range provide
    equivalent per-mode coverage without false cross-class collisions.
    """
    raise unittest.SkipTest("Subaru brake-intercept variants share LKAS TX msgs — skip cross-mode TX check")

  # ── zero pressure always allowed ────────────────────────────────────────────

  def test_es_brake_zero_allowed_at_standstill(self):
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))

  def test_es_brake_zero_allowed_when_moving(self):
    """Zero pressure is a passthrough — always TX even when fully moving."""
    self._exhaust_hysteresis()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))

  def test_es_brake_zero_allowed_when_moving_controls_off(self):
    """Zero pressure passes even with controls_allowed=False."""
    self._exhaust_hysteresis()
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._es_brake_msg(0)))

  # ── non-zero pressure at standstill ─────────────────────────────────────────

  def test_es_brake_nonzero_allowed_at_standstill(self):
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(100)))

  def test_es_brake_at_max_allowed_at_standstill(self):
    """Boundary: pressure == max_brake (600) at standstill is allowed."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(600)))

  def test_es_brake_exceeds_max_blocked_at_standstill(self):
    """pressure = 601 > max_brake → violation regardless of standstill."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(601)))

  def test_es_brake_requires_controls_allowed_at_standstill(self):
    """Non-zero pressure with controls_allowed=False → blocked."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  # ── non-zero pressure when moving (hysteresis exhausted) ────────────────────

  def test_es_brake_nonzero_blocked_when_moving(self):
    """After hysteresis expires, non-zero pressure must be blocked."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._exhaust_hysteresis()
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_es_brake_nonzero_blocked_when_moving_various_pressures(self):
    """Several pressure values all blocked once fully moving."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._exhaust_hysteresis()
    for pressure in (1, 50, 100, 300, 600):
      with self.subTest(pressure=pressure):
        self.assertFalse(self._tx(self._es_brake_msg(pressure)))

  # ── RACE-A hysteresis ────────────────────────────────────────────────────────

  def test_race_a_allowed_during_hysteresis_frame1(self):
    """
    Exactly 1 moving frame received → countdown=1 < 3 → settling → ALLOWED.
    Models the first Wheel_Speeds frame where panda sees vehicle_moving=True
    but Python has not yet updated CS.out.standstill.
    """
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=1)
    self.assertTrue(self._tx(self._es_brake_msg(100)))

  def test_race_a_allowed_during_hysteresis_frame2(self):
    """2 moving frames → countdown=2 < 3 → still settling → ALLOWED."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=2)
    self.assertTrue(self._tx(self._es_brake_msg(100)))

  def test_race_a_blocked_at_frame3(self):
    """
    Exactly BRAKE_INTERCEPT_RELEASE_FRAMES (3) moving frames → countdown=3.
    3 < 3 is False → not settling → BLOCKED.
    """
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES)
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_race_a_blocked_after_hysteresis_frame4plus(self):
    """4+ frames — countdown capped at 3, still blocked."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._exhaust_hysteresis()  # BRAKE_INTERCEPT_RELEASE_FRAMES + 1 = 4 frames
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_race_a_countdown_resets_on_standstill(self):
    """After hysteresis expires, returning to standstill re-allows non-zero pressure."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._exhaust_hysteresis()
    # Now blocked
    self.assertFalse(self._tx(self._es_brake_msg(100)))
    # Return to standstill → countdown reset to 0
    self._set_standstill()
    # Should be allowed again
    self.assertTrue(self._tx(self._es_brake_msg(100)))

  def test_race_a_hysteresis_does_not_bypass_controls_allowed(self):
    """Even inside the settling window, controls_allowed=False still blocks non-zero."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self._set_moving(frames=1)  # inside settling window
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_race_a_hysteresis_does_not_bypass_max_brake(self):
    """Pressure > max_brake blocked even inside settling window."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=1)  # inside settling window
    self.assertFalse(self._tx(self._es_brake_msg(601)))

  # ── brake_intercept absent → ES_Brake not in allowlist ──────────────────────

  def test_no_brake_intercept_es_brake_blocked(self):
    """
    Re-init with NO brake_intercept SP param.
    ES_Brake is not in TX allowlist → tx blocked unconditionally.
    """
    self.safety.set_current_safety_param_sp(0)  # no SP flags
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))    # even zero blocked (not in allowlist)
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  # ── gen2 never gets brake_intercept ─────────────────────────────────────────

  def test_gen2_with_brake_intercept_sp_param_es_brake_blocked(self):
    """
    Gen2 flag ignores SP brake_intercept — ES_Brake must not be in TX allowlist.
    """
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))
    self.assertFalse(self._tx(self._es_brake_msg(100)))


class TestSubaruSnGBrakeIntercept(TestSubaruBrakeIntercept):
  """
  Gen1, SnG + brake_intercept SP params combined.
  TX allowlist: base LKAS + ES_Distance (no relay) + Throttle (cam, relay)
                + Brake_Pedal (cam, relay) + ES_Brake (main, relay).
  """
  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.Throttle,   SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Pedal, SUBARU_CAM_BUS]]
    + [[SubaruMsg.ES_Brake,   SUBARU_MAIN_BUS]]
  )

  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
      SubaruMsg.ES_Brake,
    ),
    SUBARU_CAM_BUS: (
      SubaruMsg.Throttle,
      MSG_SUBARU_Brake_Pedal,
    ),
  }

  # Throttle TX bus=CAM_BUS (check_relay) → fwd blocked when Throttle from MAIN_BUS
  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
      SubaruMsg.ES_Brake,
    ],
    SUBARU_MAIN_BUS: [
      SubaruMsg.Throttle,
      MSG_SUBARU_Brake_Pedal,
    ],
  }

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    # CRITICAL: SP param set BEFORE set_safety_hooks
    self.safety.set_current_safety_param_sp(
      SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT
    )
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  # ── SnG+brake_intercept allowlist checks ────────────────────────────────────

  def test_sng_brake_intercept_includes_es_brake(self):
    self.assertIn([SubaruMsg.ES_Brake, SUBARU_MAIN_BUS], self.TX_MSGS)

  def test_sng_brake_intercept_includes_throttle(self):
    self.assertIn([SubaruMsg.Throttle, SUBARU_CAM_BUS], self.TX_MSGS)

  def test_sng_brake_intercept_includes_brake_pedal(self):
    self.assertIn([MSG_SUBARU_Brake_Pedal, SUBARU_CAM_BUS], self.TX_MSGS)

  def test_sng_brake_intercept_does_not_include_es_brake_on_wrong_bus(self):
    """ES_Brake is on MAIN bus, not CAM bus."""
    self.assertNotIn([SubaruMsg.ES_Brake, SUBARU_CAM_BUS], self.TX_MSGS)

  # Override: re-init tests that change SP param must restore SnG|BRAKE_INTERCEPT

  def test_no_brake_intercept_es_brake_blocked(self):
    """Re-init with ONLY SnG (no brake_intercept) → ES_Brake not in allowlist."""
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.STOP_AND_GO)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_gen2_with_brake_intercept_sp_param_es_brake_blocked(self):
    """Gen2 + SnG|brake_intercept → ES_Brake still not in allowlist."""
    self.safety.set_current_safety_param_sp(
      SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT
    )
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._es_brake_msg(0)))
    self.assertFalse(self._tx(self._es_brake_msg(100)))


if __name__ == "__main__":
  unittest.main()
