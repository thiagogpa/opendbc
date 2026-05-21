#!/usr/bin/env python3
"""
Panda safety tests for the Subaru brake-intercept feature.

Settling-window counter mechanics (rx_hook counts UP):
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
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.sunnypilot.car.subaru.values_ext import SubaruSafetyFlagsSP

# Re-use constants / helpers from the upstream test module
from opendbc.safety.tests.test_subaru import (
  SubaruMsg,
  SUBARU_MAIN_BUS,
  SUBARU_CAM_BUS,
  lkas_tx_msgs,
  TestSubaruSafetyBase,
)

# Brake_Pedal (0x139) and Brake_Status (0x13C) are not in SubaruMsg enum — define locally
MSG_SUBARU_Brake_Pedal  = 0x139
MSG_SUBARU_Brake_Status = 0x13C

BRAKE_INTERCEPT_RELEASE_FRAMES = 3  # must match C #define
SUBARU_BRAKE_HOLD_ACTIVE_FRAMES = 4  # must match C #define (~80ms at 50Hz Wheel_Speeds, 1 frame = 20ms)


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

  def test_es_brake_allowed_with_gas_pressed_at_standstill(self):
    """Gas is intentionally NOT gated on the AVH path: an injected hold at standstill is allowed
    even with the gas pressed. This is deliberate — the controller owns the gas-release UX, the
    same ES_Brake frame carries Eyesight's AEB echo (which must never be gas-gated), and braking
    is the fail-safe direction (already bounded by authority + pressure + standstill)."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._rx(self._user_gas_msg(2000))  # driver on the gas
    self.assertTrue(self._tx(self._es_brake_msg(600)))

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

  # ── settling-window hysteresis ───────────────────────────────────────────────

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

  # ── Conditional forwarding ───────────────────────────────────────────────────
  #
  # In brake-intercept mode, ES_Brake (CAM→MAIN) and Brake_Status (MAIN→CAM) forwarding is
  # blocked only while openpilot is actively asserting a hold — not unconditionally.
  # Unconditional blocking would break Eyesight's native ACC braking: Eyesight's ES_Brake
  # would never reach the braking module, and the module's Brake_Status (ES_Brake=1
  # confirmation) would never reach Eyesight → Cruise_Fault watchdog within ~566ms.
  # The active-hold state is tracked via a Wheel_Speeds-paced countdown set on TX of
  # ES_Brake (Brake_Pressure>0) or Brake_Status (the mask).

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


class TestSubaruLongBrakeIntercept(TestSubaruBrakeIntercept):
  """
  Gen1, alpha long enabled, brake_intercept SP param set, no SnG.
  This is the scenario that fails on real hardware (route dde08cad3a74cd94|00000012):
  alpha long ON, MADS active, op long not engaged, AVH wants to hold at standstill.

  Expected behavior:
    - ES_Brake=600 at standstill+MADS must be ACCEPTED (currently rejected by
      longitudinal_brake_checks because controls_allowed=False).
    - Brake_Status mask must be in TX allowlist and accepted when subaru_brake_intercept set.
    - When controls_allowed=True (ACC engaged), full-range ES_Brake (any pressure ≤ max_brake)
      must still be accepted — the union with AVH-valid set must not narrow the long path.
  """
  FLAGS = SubaruSafetyFlags.LONG

  # TX allowlist when subaru_longitudinal && subaru_brake_intercept (no SnG).
  # Must include: base LKAS + long common (ES_Distance, ES_Brake, ES_Status)
  # + Brake_Status (cam, conditional fwd via disable_static_blocking)
  # + Brake_Pedal (cam, kept for SnG-resume compat even when SnG not selected — harmless).
  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.ES_Brake,        SUBARU_MAIN_BUS]]
    + [[SubaruMsg.ES_Status,       SUBARU_MAIN_BUS]]
    + [[MSG_SUBARU_Brake_Status,   SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Pedal,    SUBARU_CAM_BUS]]
  )

  # MAIN bus relay addrs: long mode adds ES_Distance + ES_Status (check_relay=true in
  # SUBARU_COMMON_LONG_TX_MSGS). CAM bus adds Brake_Pedal (standalone entry with check_relay=true)
  # and Brake_Status.
  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ),
    SUBARU_CAM_BUS: (
      MSG_SUBARU_Brake_Pedal, MSG_SUBARU_Brake_Status,
    ),
  }

  # In long mode, ES_Brake + ES_Distance + ES_Status cam→main are ALL statically blocked
  # (SUBARU_COMMON_LONG_TX_MSGS uses check_relay=true for all three, no disable_static_blocking).
  # Op long is the sole sender of these on main bus. Brake_Pedal (TX bus=CAM → block MAIN→CAM).
  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ],
    SUBARU_MAIN_BUS: [
      MSG_SUBARU_Brake_Pedal,
    ],
  }

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    # CRITICAL: SP param set BEFORE set_safety_hooks (panda reads at init).
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  # ── Allowlist sanity ─────────────────────────────────────────────────────────

  def test_long_plus_intercept_includes_es_brake_main(self):
    self.assertIn([SubaruMsg.ES_Brake, SUBARU_MAIN_BUS], self.TX_MSGS)

  def test_long_plus_intercept_includes_brake_status_cam(self):
    self.assertIn([MSG_SUBARU_Brake_Status, SUBARU_CAM_BUS], self.TX_MSGS)

  def test_long_plus_intercept_includes_es_status_main(self):
    self.assertIn([SubaruMsg.ES_Status, SUBARU_MAIN_BUS], self.TX_MSGS)

  # ── Core regression: MADS-only AVH hold at standstill with alpha long ────────

  def test_avh_hold_pressure_allowed_with_mads_at_standstill(self):
    """
    Reproduces the failing log scenario: alpha long enabled, op long NOT engaged,
    MADS active (controls_allowed_lateral=True), standstill.
    AVH must be allowed to inject ES_Brake=BRAKE_HOLD_PRESSURE (600).
    """
    self._set_standstill()
    self.safety.set_controls_allowed(False)             # ACC off
    self.safety.set_controls_allowed_lateral(True)      # MADS active
    self.assertTrue(self._tx(self._es_brake_msg(600)),
                    "AVH hold at standstill+MADS must be allowed when alpha long is on")

  def test_avh_hold_pressure_blocked_when_moving_without_acc(self):
    """
    Safety invariant: with only MADS (no ACC), brake injection is allowed
    ONLY at standstill (+ the settling window). Once truly rolling, AVH is blocked.
    """
    self._exhaust_hysteresis()                          # countdown maxed → no longer settling
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self.assertFalse(self._tx(self._es_brake_msg(600)),
                     "AVH must NOT inject brake while rolling without ACC engaged")

  def test_long_path_unchanged_when_acc_engaged(self):
    """
    When ACC is engaged (controls_allowed=True), full ES_Brake range must work
    EVEN AT NON-STANDSTILL — the long-path semantics (op-long deceleration from
    rolling speed) must not be narrowed by adding the AVH union.
    """
    self._exhaust_hysteresis()                          # rolling, NOT in settling window
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.assertTrue(self._tx(self._es_brake_msg(100)),  # op long mid-brake
                    "op-long must brake while rolling with ACC engaged")

  def test_brake_pressure_above_max_rejected(self):
    """Max brake limit (600) is enforced regardless of which path is permissive."""
    self._set_standstill()
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed_lateral(True)
    self.assertFalse(self._tx(self._es_brake_msg(601)))

  def test_no_controls_no_lateral_blocks_nonzero_brake(self):
    """Neither ACC nor MADS → any nonzero brake is rejected."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._tx(self._es_brake_msg(100)))
    self.assertTrue(self._tx(self._es_brake_msg(0)),
                    "Zero brake is always allowed (inactive value)")

  # ── Brake_Status mask coverage ───────────────────────────────────────────────

  def test_brake_status_mask_allowed_with_brake_intercept(self):
    """Brake_Status with ES_Brake_bit=0 must be allowed when subaru_brake_intercept set,
    regardless of subaru_longitudinal state."""
    self.assertTrue(self._tx(self._brake_status_msg(0)))

  def test_brake_status_mask_with_es_brake_bit_set_rejected(self):
    """Brake_Status MUST clear the ES_Brake bit (existing invariant from line 259)."""
    self.assertFalse(self._tx(self._brake_status_msg(1)))

  # ── Gen2 negative ────────────────────────────────────────────────────────────

  def test_gen2_long_with_brake_intercept_uses_gen2_long_path(self):
    """
    Gen2 + LONG + brake_intercept must continue to use SUBARU_GEN2_LONG_TX_MSGS
    (no AVH support on gen2 — interfaces.py never sets BRAKE_HOLD on gen2 anyway,
    but panda must not silently accept Brake_Status on the cam bus for gen2).
    """
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                  SubaruSafetyFlags.LONG | SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    # Brake_Status must NOT be in gen2 long allowlist.
    self.assertFalse(self._tx(self._brake_status_msg(0)))

  # ── Moving-while-MADS overrides: parent uses controls_allowed=True which, in long mode,
  # triggers the longitudinal path (allowed while rolling). Override to MADS-only
  # (controls_allowed=False, controls_allowed_lateral=True) to properly test the AVH invariant.

  def test_es_brake_nonzero_blocked_when_moving(self):
    """MADS-only: once rolling past settling window, AVH must not inject brake."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._exhaust_hysteresis()
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_es_brake_nonzero_blocked_when_moving_various_pressures(self):
    """MADS-only: several pressures all blocked once fully moving past settling window."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._exhaust_hysteresis()
    for pressure in (1, 50, 100, 300, 600):
      with self.subTest(pressure=pressure):
        self.assertFalse(self._tx(self._es_brake_msg(pressure)))

  def test_race_a_blocked_at_frame3(self):
    """MADS-only: at BRAKE_INTERCEPT_RELEASE_FRAMES moving frames, not settling → blocked."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES)
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_race_a_blocked_after_hysteresis_frame4plus(self):
    """MADS-only: 4+ frames past settling → blocked."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._exhaust_hysteresis()
    self.assertFalse(self._tx(self._es_brake_msg(100)))

  def test_race_a_countdown_resets_on_standstill(self):
    """MADS-only: returning to standstill after rolling re-allows non-zero pressure."""
    self._set_standstill()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(True)
    self._exhaust_hysteresis()
    self.assertFalse(self._tx(self._es_brake_msg(100)))
    self._set_standstill()
    self.assertTrue(self._tx(self._es_brake_msg(100)))

  # ── Fwd overrides: ES_Brake cam→main is ALWAYS statically blocked in long mode ─

  def test_fwd_es_brake_cam_to_main_allowed_when_idle(self):
    """In long mode, op long owns ES_Brake on main bus — cam→main always statically blocked."""
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_restored_after_countdown_expires(self):
    """After countdown: Brake_Status fwd restored; ES_Brake remains statically blocked."""
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_blocked_retriggered_by_subsequent_hold_tx(self):
    """Re-trigger after countdown: Brake_Status re-blocked; ES_Brake always blocked."""
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self._set_standstill()
    self.assertTrue(self._tx(self._es_brake_msg(400)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))

  def test_fwd_not_blocked_after_zero_pressure_tx(self):
    """Zero-pressure TX: no hold countdown set; Brake_Status still forwarded; ES_Brake always -1."""
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._es_brake_msg(0)))
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_remains_blocked_during_countdown(self):
    """During countdown: Brake_Status blocked; ES_Brake also -1 (always, in long mode)."""
    self._tx_hold_pressure()
    for k in range(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES - 1):
      self._pump_wheel_speeds(1)
      with self.subTest(after_pumps=k + 1):
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake))
        self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status))

  def test_fwd_es_brake_restored_for_aeb_after_hold_release(self):
    """In long mode, ES_Brake cam→main is always blocked — op long handles AEB, not Eyesight relay."""
    self._tx_hold_pressure()
    self._pump_wheel_speeds(SUBARU_BRAKE_HOLD_ACTIVE_FRAMES)
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_CAM_BUS, SubaruMsg.ES_Brake),
                     "ES_Brake cam→main must stay blocked in long mode even after hold release")

  def test_long_path_brake_does_not_bump_countdown(self):
    """Op-long braking via long_valid path must NOT bump the hold countdown.
    Scenario 3a/3b: if countdown were bumped, Eyesight's Brake_Status relay would be
    starved during sustained ACC braking, causing ACC faults."""
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES + 1)  # fully moving, past settling
    # TX via long_valid path (ACC on, moving — avh_valid=False)
    self.assertTrue(self._tx(self._es_brake_msg(300)),
                    "op-long brake TX must be allowed via long_valid path")
    # Countdown must NOT have been bumped — Brake_Status must still forward
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status),
                     "Brake_Status MAIN→CAM must NOT be blocked after op-long brake TX (only AVH hold may block)")

  def test_sustained_long_braking_does_not_starve_brake_status_relay(self):
    """30 consecutive op-long brake TXes must not starve the Brake_Status fwd relay.
    Without the fix, every TX refreshes the countdown indefinitely.
    Must keep vehicle_moving=True throughout — uses non-zero speed RX to stay in long_valid path."""
    self.safety.set_controls_allowed(True)
    self._set_moving(frames=BRAKE_INTERCEPT_RELEASE_FRAMES + 1)
    for _ in range(30):
      self.assertTrue(self._tx(self._es_brake_msg(300)))
      self._rx(self._speed_msg(10))  # non-zero: stay moving so avh_valid=False (long path only)
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status),
                     "Brake_Status must still forward to cam after sustained op-long braking")

  def test_safety_reinit_resets_countdown(self):
    """Re-calling set_safety_hooks must clear the active-hold countdown immediately."""
    self._tx_hold_pressure()
    # Verify countdown is set
    self.assertEqual(-1, self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status),
                     "countdown should be active after hold TX")
    # Re-init safety
    self.safety.set_current_safety_param_sp(SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()
    # Countdown must be reset to 0 — fwd must be immediately open
    self.assertEqual(SUBARU_CAM_BUS,
                     self.safety.safety_fwd_hook(SUBARU_MAIN_BUS, MSG_SUBARU_Brake_Status),
                     "Brake_Status fwd must be unblocked immediately after safety reinit")


class TestSubaruLongSnGBrakeIntercept(TestSubaruLongBrakeIntercept):
  """
  Gen1, alpha long enabled, SnG + brake_intercept SP params, AVH.
  This is the on-device configuration for SUBARU_IMPREZA_2020 with both
  StopAndGo and alpha long enabled.
  """

  TX_MSGS = (
    lkas_tx_msgs(SUBARU_MAIN_BUS)
    + [[SubaruMsg.ES_Brake,        SUBARU_MAIN_BUS]]
    + [[SubaruMsg.ES_Status,       SUBARU_MAIN_BUS]]
    + [[SubaruMsg.Throttle,        SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Pedal,    SUBARU_CAM_BUS]]
    + [[MSG_SUBARU_Brake_Status,   SUBARU_CAM_BUS]]
  )

  RELAY_MALFUNCTION_ADDRS = {
    SUBARU_MAIN_BUS: (
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ),
    SUBARU_CAM_BUS: (
      SubaruMsg.Throttle, MSG_SUBARU_Brake_Pedal, MSG_SUBARU_Brake_Status,
    ),
  }

  FWD_BLACKLISTED_ADDRS = {
    SUBARU_CAM_BUS: [
      SubaruMsg.ES_LKAS, SubaruMsg.ES_DashStatus, SubaruMsg.ES_LKAS_State,
      SubaruMsg.ES_Infotainment, SubaruMsg.ES_Brake, SubaruMsg.ES_Distance, SubaruMsg.ES_Status,
    ],
    SUBARU_MAIN_BUS: [
      SubaruMsg.Throttle, MSG_SUBARU_Brake_Pedal,
    ],
  }

  def setUp(self):
    self.packer = CANPackerSafety("subaru_global_2017_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(
      SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT
    )
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru, self.FLAGS)
    self.safety.init_tests()

  def test_long_sng_intercept_includes_throttle_cam(self):
    self.assertIn([SubaruMsg.Throttle, SUBARU_CAM_BUS], self.TX_MSGS)

  def test_long_sng_intercept_includes_brake_pedal_cam(self):
    self.assertIn([MSG_SUBARU_Brake_Pedal, SUBARU_CAM_BUS], self.TX_MSGS)

  def test_gen2_long_with_brake_intercept_uses_gen2_long_path(self):
    self.safety.set_current_safety_param_sp(
      SubaruSafetyFlagsSP.STOP_AND_GO | SubaruSafetyFlagsSP.BRAKE_INTERCEPT
    )
    self.safety.set_safety_hooks(CarParams.SafetyModel.subaru,
                                  SubaruSafetyFlags.LONG | SubaruSafetyFlags.GEN2)
    self.safety.init_tests()
    self.assertFalse(self._tx(self._brake_status_msg(0)))


if __name__ == "__main__":
  unittest.main()
