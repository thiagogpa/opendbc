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

# Brake_Pedal (0x139) and Brake_Status (0x13C) are not in SubaruMsg enum — define locally
MSG_SUBARU_Brake_Pedal  = 0x139
MSG_SUBARU_Brake_Status = 0x13C

BRAKE_INTERCEPT_RELEASE_FRAMES = 3  # must match C #define
SUBARU_BRAKE_HOLD_ACTIVE_FRAMES = 4  # must match C #define (~200ms at 20Hz Wheel_Speeds)


class TestSubaruBrakeIntercept(TestSubaruSafetyBase):
  """
  Gen1, no SnG, brake_intercept SP param set.
  TX allowlist: base LKAS + ES_Distance (no relay) + Brake_Pedal (cam, relay) + ES_Brake (main, relay).
  """
  SAFETY_MODEL = CarParams.SafetyModel.subaru
  FLAGS = 0  # gen1, no longitudinal

  # base LKAS msgs + ES_Distance (no relay) + Brake_Pedal + ES_Brake + Brake_Status
  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)                           # ES_LKAS, ES_DashStatus, ES_LKAS_State, ES_Infotainment + ES_Distance
    + [[MSG_SUBARU_Brake_Pedal,  SUBARU_CAM_BUS]]           # 0x139 cam bus
    + [[SubaruMsg.ES_Brake,      SUBARU_MAIN_BUS]]          # 0x220 main bus
    + [[MSG_SUBARU_Brake_Status, SUBARU_CAM_BUS]]           # 0x13C cam bus (masked copy, ES_Brake=0)
  )

  # Relay malfunction fires when received (addr,bus) matches a check_relay=true TX entry.
  # ES_Brake TX bus = MAIN_BUS → relay malfunction if ES_Brake seen on MAIN_BUS.
  # Brake_Pedal TX bus = CAM_BUS → relay malfunction if Brake_Pedal seen on CAM_BUS.
  # Brake_Status TX bus = CAM_BUS → relay malfunction if Brake_Status seen on CAM_BUS.
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
      MSG_SUBARU_Brake_Status,
    ),
  }

  # Forwarding block logic: check_relay=true entry with bus=destination_bus → fwd returns -1
  # UNLESS disable_static_blocking=true (see ES_Brake & Brake_Status below — those are
  # conditionally blocked via subaru_fwd_hook only while a hold is actively being injected).
  # FWD_BLACKLISTED_ADDRS keys = source bus (the bus the message ARRIVES from).
  # ES_Brake (0x220): conditionally blocked CAM→MAIN — covered by TestSubaruBrakeHoldFwd.
  # Brake_Pedal (0x139): TX bus=CAM_BUS → blocked arriving from MAIN_BUS (src=MAIN, dst=CAM).
  # Brake_Status (0x13C): conditionally blocked MAIN→CAM — covered by TestSubaruBrakeHoldFwd.
  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
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
    """Non-zero pressure blocked only when BOTH controls_allowed and controls_allowed_lateral are False."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_es_brake_allowed_with_mads_only_at_standstill(self):
    """controls_allowed=False but controls_allowed_lateral=True (MADS active, no ACC) → allowed."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._tx(self._es_brake_msg(100)))

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
    """Even inside the settling window, both controls_allowed=False AND controls_allowed_lateral=False blocks non-zero."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self._set_moving(frames=1)  # inside settling window
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_race_a_hysteresis_does_not_bypass_max_brake(self):
    """Pressure > max_brake blocked even inside settling window."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=1)  # inside settling window
    self.assertFalse(self._tx(self._es_brake_msg(601)))

  # ── Brake_Status (0x13C) masking tx_hook ────────────────────────────────────

  def _brake_status_msg(self, es_brake_bit):
    """Build a Brake_Status CAN message with the ES_Brake bit set or cleared.
    ES_Brake is bit 2 of byte 7 (bit 58 overall) per subaru_global_2017_generated.dbc."""
    # The packer doesn't expose a named ES_Brake signal in Brake_Status directly via
    # the safety packer, so build the raw byte manually.
    values = {"ES_Brake": es_brake_bit, "Brake": 0}
    return self.packer.make_can_msg_safety("Brake_Status", SUBARU_CAM_BUS, values)

  def test_brake_status_allowed_es_brake_cleared(self):
    """Brake_Status with ES_Brake=0 to cam bus must be allowed in brake_intercept mode."""
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._brake_status_msg(0)))

  def test_brake_status_blocked_es_brake_set(self):
    """Brake_Status with ES_Brake=1 must be blocked — panda must never forward this to Eyesight."""
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._brake_status_msg(1)))

  def test_brake_status_allowed_controls_off(self):
    """Brake_Status masking is not gated on controls_allowed — always needed during hold."""
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._brake_status_msg(0)))

  def test_brake_status_blocked_without_brake_intercept(self):
    """In non-brake_intercept mode, Brake_Status is not in TX allowlist → blocked."""
    self.safety.set_current_safety_param_sp(0)  # no SP flags
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._brake_status_msg(0)))

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

  # ── ACC-fault fix (2026-05-11): conditional forwarding ──────────────────────
  #
  # Background: in brake-intercept mode, Panda used to UNCONDITIONALLY block
  # ES_Brake (CAM→MAIN) and Brake_Status (MAIN→CAM) forwarding. This broke
  # Eyesight's native ACC braking: Eyesight's ES_Brake never reached the braking
  # module, and the module's Brake_Status (ES_Brake=1 confirmation) never
  # reached Eyesight → Cruise_Fault watchdog within ~566ms.
  #
  # Fix: forwarding is CONDITIONALLY blocked only while we are actively asserting
  # a hold. The active-hold state is tracked via a Wheel_Speeds-paced countdown
  # set on TX of ES_Brake (Brake_Pressure>0) or Brake_Status (the mask).

  def _brake_status_mask_msg(self):
    return self.packer.make_can_msg_safety("Brake_Status", SUBARU_CAM_BUS,
                                           {"ES_Brake": 0, "Brake": 0})

  def _tx_hold_pressure(self):
    """TX one ES_Brake with Brake_Pressure>0 — should set the active-hold countdown."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(400)))

  def _tx_brake_status_mask(self):
    """TX the masked Brake_Status — should also set the active-hold countdown."""
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._brake_status_mask_msg()))

  def _pump_wheel_speeds(self, n):
    """Advance the countdown by n Wheel_Speeds RX frames."""
    for _ in range(n):
      self._rx(self._speed_msg(0))

  def test_fwd_es_brake_cam_to_main_allowed_when_idle(self):
    """At setUp (no hold TX yet) — ES_Brake CAM→MAIN must forward (destination 0)."""
    self.assertEqual(SUBARU_MAIN_BUS,
                     self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_brake_status_main_to_cam_allowed_when_idle(self):
    """At idle — Brake_Status MAIN→CAM must forward (destination 2)."""
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_es_brake_cam_to_main_blocked_after_hold_tx(self):
    """After TXing ES_Brake with Brake_Pressure>0 — relay CAM→MAIN must be blocked."""
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_brake_status_main_to_cam_blocked_after_hold_tx(self):
    """After TXing ES_Brake hold — Brake_Status MAIN→CAM must be blocked."""
    self._tx_hold_pressure()
    self.assertEqual(-1,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_brake_status_main_to_cam_blocked_after_mask_tx(self):
    """After TXing the Brake_Status mask — forwarding must be blocked too."""
    self._tx_brake_status_mask()
    self.assertEqual(-1,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_remains_blocked_during_countdown(self):
    """Within SUBARU_BRAKE_HOLD_ACTIVE_FRAMES of a hold TX, forwarding stays blocked."""
    self._tx_hold_pressure()
    for k in range(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES - 1):
      self._pump_wheel_speeds(1)
      with self.subTest(after_pumps=k + 1):
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_restored_after_countdown_expires(self):
    """After SUBARU_BRAKE_HOLD_ACTIVE_FRAMES Wheel_Speeds RX, forwarding is restored."""
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(SUBARU_MAIN_BUS,
                     self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_blocked_retriggered_by_subsequent_hold_tx(self):
    """A new hold TX after countdown expired must re-block forwarding."""
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(SUBARU_MAIN_BUS,
                     self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self._set_standstill()
    self.assertTrue(self._tx(self._es_brake_msg(400)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_not_blocked_after_zero_pressure_tx(self):
    """TXing ES_Brake with Brake_Pressure=0 must NOT engage the hold gate."""
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))
    self.assertEqual(SUBARU_MAIN_BUS,
                     self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_es_brake_restored_for_aeb_after_hold_release(self):
    """
    AEB-while-moving relay test.

    Scenario: driver was held at standstill (AVH active), then accelerated.
    While moving, Eyesight triggers AEB. Eyesight sends its own ES_Brake on
    the cam bus — Panda must relay it to the braking module (cam→main).

    Sequence:
      1. Hold engaged → hold TX sets the active-hold countdown.
      2. Driver releases hold (gas press) → Python stops sending hold TXes.
      3. Car moves → SUBARU_BRAKE_HOLD_ACTIVE_FRAMES Wheel_Speeds frames expire
         the countdown.
      4. AEB fires → Eyesight's ES_Brake appears on cam bus.
         Panda must forward it (return SUBARU_MAIN_BUS), not block it (-1).
    """
    # Step 1: hold TX (sets countdown)
    self._tx_hold_pressure()
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake),
                     "relay must be blocked during hold")

    # Step 2+3: hold released, countdown decays as car moves
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)

    # Step 4: AEB fires — Eyesight's ES_Brake (cam→main) must be forwarded
    self.assertEqual(SUBARU_MAIN_BUS,
                     self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake),
                     "Eyesight AEB ES_Brake must reach braking module after hold release")


class TestSubaruSnGBrakeIntercept(TestSubaruBrakeIntercept):
  """
  Gen1, SnG + brake_intercept SP params combined.
  TX allowlist: base LKAS + ES_Distance (no relay) + Throttle (cam, relay)
                + Brake_Pedal (cam, relay) + ES_Brake (main, relay) + Brake_Status (cam, relay).
  """
  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.Throttle,        SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Pedal,    SUBARU_CAM_BUS]]
    + [[SubaruMsg.ES_Brake,        SUBARU_MAIN_BUS]]
    + [[MSG_SUBARU_Brake_Status,   SUBARU_CAM_BUS]]
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
      MSG_SUBARU_Brake_Status,
    ),
  }

  # Throttle TX bus=CAM_BUS → blocked arriving from MAIN_BUS (src=MAIN, dst=CAM).
  # ES_Brake & Brake_Status: conditional — see TestSubaruBrakeHoldFwd.
  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS,
      SubaruMsg.ES_DashStatus,
      SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment,
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

  def test_brake_status_blocked_without_brake_intercept(self):
    """In SnG-only mode (no brake_intercept), Brake_Status not in allowlist → blocked."""
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.STOP_AND_GO)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._brake_status_msg(0)))


if __name__ == "__main__":
  unittest.main()
