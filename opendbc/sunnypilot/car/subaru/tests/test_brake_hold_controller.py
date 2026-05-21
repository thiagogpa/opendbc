"""Tests for the Subaru AVH brake-hold logic in CarController.update().

Covers the velocity-primed latch state machine (`_update_brake_hold_state`) — priming,
holding, AEB echo, and release — plus the ES_Brake / Brake_Status arbitration in `update()`
(when AVH owns ES_Brake vs. yields to op long, and when the Brake_Status mask is sent).

Most tests patch `create_es_brake_hold` and assert on its call args; `test_real_packer_*`
runs the real CANPacker end-to-end and decodes the emitted frame to catch controller↔packer
drift the mocks can't.
"""

import pytest
from unittest.mock import MagicMock, patch

from opendbc.can import CANParser
from opendbc.car import structs, Bus
from opendbc.car.subaru.values import DBC, CAR, SubaruFlags, CarControllerParams
from opendbc.car.subaru.carcontroller import CarController
from opendbc.car.interfaces import GearShifter

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_DUMMY_MSG = ("ES_Brake", bytes(8), 0)
_DUMMY_BS_MSG = ("Brake_Status", bytes(8), 2)
_DBC_NAMES = DBC[CAR.SUBARU_IMPREZA_2020.value]


def make_CP(brake_hold: bool = True, long_control: bool = False) -> structs.CarParams:
  CP = structs.CarParams()
  CP.carFingerprint = CAR.SUBARU_IMPREZA_2020.value
  CP.openpilotLongitudinalControl = long_control
  if brake_hold:
    CP.flags = SubaruFlags.BRAKE_HOLD.value | SubaruFlags.STEER_RATE_LIMITED.value
  else:
    CP.flags = SubaruFlags.STEER_RATE_LIMITED.value
  return CP


def make_CC(lat_active: bool = False, long_active: bool = False, enabled: bool = False, cancel: bool = False):
  """Returns a capnp reader (needed so actuators.as_builder() works in update())."""
  cc_b = structs.CarControl()
  cc_b.enabled = enabled
  cc_b.latActive = lat_active
  cc_b.longActive = long_active
  cc_b.cruiseControl.cancel = cancel
  return cc_b.as_reader()


def make_CC_SP(mads_enabled: bool = True, mads_active: bool = True) -> structs.CarControlSP:
  CC_SP = structs.CarControlSP()
  CC_SP.mads.enabled = mads_enabled
  CC_SP.mads.active = mads_active
  return CC_SP


def make_CS(
  vEgoRaw: float = 0.0,
  brakePressed: bool = False,
  gasPressed: bool = False,
  standstill: bool = True,
  gear: GearShifter = GearShifter.drive,
  aeb_status: int = 0,
  es_brake_msg=None,
):
  CS = MagicMock()
  CS.out = MagicMock()
  CS.out.vEgoRaw = vEgoRaw
  CS.out.brakePressed = brakePressed
  CS.out.gasPressed = gasPressed
  CS.out.standstill = standstill
  CS.out.gearShifter = gear
  CS.out.steeringTorque = 0
  CS.out.steeringRateDeg = 0
  CS.es_distance_msg = {"COUNTER": 0, "Cruise_Cancel": False, "Cruise_Throttle": 0,
                         "Close_Distance": 0.0}
  CS.es_dashstatus_msg = {}
  CS.es_lkas_state_msg = {}
  CS.es_infotainment_msg = {}
  CS.es_status_msg = {}
  if es_brake_msg is None:
    CS.es_brake_msg = {
      "AEB_Status": aeb_status, "CHECKSUM": 0, "Signal1": 0,
      "Brake_Pressure": 0, "Cruise_Brake_Lights": 0,
      "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 0,
      "Cruise_Activated": 0, "Signal3": 0,
    }
  else:
    CS.es_brake_msg = es_brake_msg
  CS.brake_status_msg = {
    "CHECKSUM": 0, "COUNTER": 0, "Signal1": 0, "ES_Brake": 0,
    "Signal2": 0, "Brake": 0, "Signal3": 0,
  }
  CS.cruise_button = 0
  # SnGCarController fields (unused unless SnG enabled on CP_SP)
  CS.throttle_msg = {}
  CS.brake_pedal_msg = {}
  return CS


def make_ctrl(brake_hold: bool = True, long_control: bool = False) -> CarController:
  CP = make_CP(brake_hold=brake_hold, long_control=long_control)
  CP_SP = structs.CarParamsSP()  # no SnG flags → SnGCarController disabled
  return CarController(_DBC_NAMES, CP, CP_SP)


# Patch targets for functions unrelated to brake hold that would fail on mock CS data.
_STEERING_PATCH = patch("opendbc.car.subaru.subarucan.create_steering_control", return_value=_DUMMY_MSG)
_DASHSTATUS_PATCH = patch("opendbc.car.subaru.subarucan.create_es_dashstatus", return_value=_DUMMY_MSG)
_LKAS_STATE_PATCH = patch("opendbc.car.subaru.subarucan.create_es_lkas_state", return_value=_DUMMY_MSG)
_BRAKE_HOLD_PATCH = "opendbc.car.subaru.subarucan.create_es_brake_hold"
_BRAKE_STATUS_HOLD_PATCH = "opendbc.sunnypilot.car.subaru.subarucan_ext.create_brake_status_hold"


def run_update(ctrl, CC=None, CC_SP=None, CS=None):
  """Run one ctrl.update() call with all subarucan side-effect patches active.

  Returns the mock for create_es_brake_hold so callers can inspect calls.
  """
  CC = CC or make_CC()
  CC_SP = CC_SP or make_CC_SP()
  CS = CS or make_CS()
  with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
       patch(_BRAKE_HOLD_PATCH, return_value=_DUMMY_MSG) as mock_bh, \
       patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG):
    ctrl.update(CC, CC_SP, CS, 0)
  return mock_bh


def run_update_capture_both(ctrl, CC=None, CC_SP=None, CS=None):
  """Run one ctrl.update() and return (mock_es_brake_hold, mock_brake_status_hold)."""
  CC = CC or make_CC()
  CC_SP = CC_SP or make_CC_SP()
  CS = CS or make_CS()
  with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
       patch(_BRAKE_HOLD_PATCH, return_value=_DUMMY_MSG) as mock_bh, \
       patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG) as mock_bsh:
    ctrl.update(CC, CC_SP, CS, 0)
  return mock_bh, mock_bsh


# ---------------------------------------------------------------------------
# TestBrakeHoldController
# ---------------------------------------------------------------------------

class TestBrakeHoldController:

  # -----------------------------------------------------------------------
  # 1. Init
  # -----------------------------------------------------------------------

  def test_primed_false_on_init(self):
    """_brake_hold_primed must be False immediately after construction."""
    ctrl = make_ctrl()
    assert ctrl._brake_hold_primed is False

  # -----------------------------------------------------------------------
  # 2–6. Priming conditions
  # -----------------------------------------------------------------------

  @pytest.mark.parametrize("mads_enabled,vego,brake,gear,expected", [
    (False, 0.5, True, GearShifter.drive, False),  # mads disabled
    (True, 0.5, False, GearShifter.drive, False),  # brake not pressed
    (True, 2.0, True, GearShifter.drive, False),  # speed too high (>= 1.5)
    (True, 0.5, True, GearShifter.park, False),  # park
    (True, 0.5, True, GearShifter.reverse, False),  # reverse
    (True, 0.5, True, GearShifter.drive, True),  # all conditions met -> primed
  ])
  def test_priming_conditions(self, mads_enabled, vego, brake, gear, expected):
    """Priming requires mads.enabled AND vEgoRaw<1.5 AND brakePressed AND gear not park/reverse."""
    ctrl = make_ctrl()
    ctrl.frame = 1  # odd frame: priming runs but frame%5 != 0 (isolates priming from hold/reset)
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=mads_enabled),
               CS=make_CS(vEgoRaw=vego, brakePressed=brake, gear=gear))
    assert ctrl._brake_hold_primed is expected

  # -----------------------------------------------------------------------
  # 7–14. Holding conditions (verified via create_es_brake_hold call args)
  #        Tests set ctrl.frame = 0 so frame%5 == 0 path is taken.
  #        For tests that need primed=True we manually set the flag.
  # -----------------------------------------------------------------------

  def _get_brake_value(self, mock_bh):
    """Extract the brake_value positional arg (5th arg) from the mock call."""
    assert mock_bh.called, "create_es_brake_hold was not called"
    return mock_bh.call_args[0][4]  # (packer, frame, es_brake_msg, long_enabled, brake_value)

  def test_holding_requires_primed(self):
    """primed=False, standstill=True → no create_es_brake_hold call (stay out of Eyesight's way)."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = False
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False, gasPressed=False))
    assert not mock_bh.called

  def test_holding_requires_standstill(self):
    """primed=True, standstill=False → no create_es_brake_hold call (ACC may be braking)."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=False, brakePressed=False, gasPressed=False))
    assert not mock_bh.called

  def test_holding_active_while_brake_pressed(self):
    """primed=True, standstill=True, brakePressed=True → hold activates (seamless hold, foot still on pedal)."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=True, gasPressed=False))
    assert self._get_brake_value(mock_bh) == CarControllerParams.BRAKE_HOLD_PRESSURE

  def test_holding_blocked_if_gas_pressed(self):
    """primed=True, standstill=True, gasPressed=True → no create_es_brake_hold call."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False, gasPressed=True))
    assert not mock_bh.called

  def test_holding_blocked_in_park(self):
    """primed=True, standstill=True, gear=park → no create_es_brake_hold call."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False,
                                    gasPressed=False, gear=GearShifter.park))
    assert not mock_bh.called

  def test_holding_succeeds(self):
    """All hold conditions met → brake_value == BRAKE_HOLD_PRESSURE (non-zero)."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False,
                                    gasPressed=False, gear=GearShifter.drive, aeb_status=0))
    assert self._get_brake_value(mock_bh) == CarControllerParams.BRAKE_HOLD_PRESSURE

  @pytest.mark.parametrize("primed,standstill,vego,aeb_status,eyesight_pressure,expected", [
    (True, True, 0.0, 8, 450, 450),  # AEB overrides hold -> echo Eyesight pressure
    (True, True, 0.0, 0, 0, CarControllerParams.BRAKE_HOLD_PRESSURE),  # no AEB, holding -> hold pressure
    (False, False, 8.0, 4, 600, 600),  # AEB while moving, not holding -> passthrough
  ])
  def test_aeb_vs_hold_brake_value(self, primed, standstill, vego, aeb_status, eyesight_pressure, expected):
    """AEB echo (AEB_Status != 0) passes through Eyesight's Brake_Pressure; otherwise holding emits BRAKE_HOLD_PRESSURE."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = primed
    es_msg = {"AEB_Status": aeb_status, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": eyesight_pressure,
              "Cruise_Brake_Lights": 0, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 0,
              "Cruise_Activated": 0, "Signal3": 0}
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=standstill, vEgoRaw=vego, brakePressed=False,
                                    gasPressed=False, es_brake_msg=es_msg))
    assert self._get_brake_value(mock_bh) == expected

  # -----------------------------------------------------------------------
  # 15–17. Latch reset (inside frame%5 == 0)
  #
  # Pattern:
  #   frame=1 (non-multiple): prime the latch (brakePressed=True, vEgoRaw=0.5)
  #   frame=5 (multiple of 5): trigger reset condition, verify primed=False
  # -----------------------------------------------------------------------

  def test_reset_on_gas_pressed(self):
    """gasPressed=True inside frame%5==0 block → _brake_hold_primed reset to False."""
    ctrl = make_ctrl()

    # Frame 1: prime
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True))
    assert ctrl._brake_hold_primed is True, "precondition: must be primed"

    # Frame 5: gasPressed=True triggers reset
    ctrl.frame = 5
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=False, gasPressed=True, standstill=True))
    assert ctrl._brake_hold_primed is False

  def test_reset_on_mads_disabled(self):
    """mads.enabled=False inside frame%5==0 block → primed reset to False."""
    ctrl = make_ctrl()

    # Frame 1: prime with mads enabled
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True))
    assert ctrl._brake_hold_primed is True, "precondition: must be primed"

    # Frame 5: mads.enabled=False triggers reset
    ctrl.frame = 5
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=False),
               CS=make_CS(vEgoRaw=0.5, brakePressed=False, standstill=True))
    assert ctrl._brake_hold_primed is False

  def test_reset_on_vego_above_threshold(self):
    """vEgoRaw > 0.5 inside frame%5==0 block → primed reset to False."""
    ctrl = make_ctrl()

    # Frame 1: prime
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True))
    assert ctrl._brake_hold_primed is True, "precondition: must be primed"

    # Frame 5: speed > 0.5 triggers reset
    ctrl.frame = 5
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=1.0, brakePressed=False, standstill=False))
    assert ctrl._brake_hold_primed is False

  # -----------------------------------------------------------------------
  # 18. Guard: es_brake_msg=None → no call
  # -----------------------------------------------------------------------

  def test_none_guard_no_send(self):
    """CS.es_brake_msg=None → create_es_brake_hold must NOT be called."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    CS = make_CS()
    CS.es_brake_msg = None  # explicitly None — bypasses make_CS default dict
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True), CS=CS)
    assert not mock_bh.called

  # -----------------------------------------------------------------------
  # Guard: longActive yields ES_Brake to op long — covered by
  # TestAlphaLongCoexistence.test_avh_yields_when_alpha_long_active (stronger:
  # also asserts create_es_brake runs).
  # -----------------------------------------------------------------------

  # -----------------------------------------------------------------------
  # 20. Guard: BRAKE_HOLD flag absent → no call
  # -----------------------------------------------------------------------

  def test_brake_hold_flag_required(self):
    """CP.flags without SubaruFlags.BRAKE_HOLD → create_es_brake_hold NOT called."""
    ctrl = make_ctrl(brake_hold=False)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True))
    assert not mock_bh.called

  # -----------------------------------------------------------------------
  # 21–23. Frame modulo gating
  # -----------------------------------------------------------------------

  def test_hold_only_at_frame_mod5(self):
    """At a frame where frame%5 != 0 → create_es_brake_hold NOT called."""
    ctrl = make_ctrl()
    ctrl.frame = 3  # 3 % 5 = 3 != 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False))
    assert not mock_bh.called

  def test_hold_at_frame_mod5_zero(self):
    """At frame=0 (0 % 5 == 0) → create_es_brake_hold IS called."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False))
    assert mock_bh.called

  def test_frame_counter_passed_correctly(self):
    """At frame=5 → create_es_brake_hold called with frame_count arg == 5 // 5 == 1."""
    ctrl = make_ctrl()
    ctrl.frame = 5
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False))
    assert mock_bh.called
    # Second positional arg is frame_count = self.frame // 5
    frame_count_arg = mock_bh.call_args[0][1]
    assert frame_count_arg == 1  # 5 // 5

  # -----------------------------------------------------------------------
  # 24. Cadence decoupling: Brake_Status mask (every 2) vs ES_Brake (every 5)
  # -----------------------------------------------------------------------

  def test_mask_sends_without_es_brake_at_frame2(self):
    """At frame=2 the Brake_Status mask (frame%2==0) fires while the ES_Brake hold
    (frame%5==0) does not. _update_brake_hold_state recomputes only on frame%5==0,
    so _brake_hold_active carries over from a prior holding frame."""
    ctrl = make_ctrl()
    ctrl._brake_hold_primed = True
    ctrl._brake_hold_active = True  # carried over from a prior frame%5==0 hold
    ctrl.frame = 2  # 2%5 != 0 (no ES_Brake hold) but 2%2 == 0 (mask sends)
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    assert not mock_bh.called   # frame%5 != 0
    assert mock_bsh.called      # frame%2 == 0 and brake_hold_active carried over

  # -----------------------------------------------------------------------
  # 25. Integration: real CANPacker (create_es_brake_hold NOT patched)
  # -----------------------------------------------------------------------

  def test_real_packer_emits_hold_pressure(self):
    """Run update() through the REAL create_es_brake_hold + CANPacker and decode the
    emitted ES_Brake frame — Brake_Pressure must equal BRAKE_HOLD_PRESSURE. Catches
    controller<->packer drift (e.g. a wrong DBC signal name) the mock-based tests cannot."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    # patch only the unrelated send fns that choke on the mock CS; leave create_es_brake_hold REAL
    with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
         patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG):
      _, can_sends = ctrl.update(
        make_CC(), make_CC_SP(mads_enabled=True),
        make_CS(standstill=True, brakePressed=False, gasPressed=False), 0,
      )
    es_brake = [m for m in can_sends if isinstance(m[0], int) and m[0] == 0x220]
    assert len(es_brake) == 1, "exactly one real ES_Brake frame expected"
    parser = CANParser(_DBC_NAMES[Bus.pt], [("ES_Brake", 0)], 0)
    parser.update([0, es_brake])
    assert parser.vl["ES_Brake"]["Brake_Pressure"] == CarControllerParams.BRAKE_HOLD_PRESSURE


# ---------------------------------------------------------------------------
# TestACCInterferenceRegression
#
# Regression coverage for the accFaulted bug observed in route
# dde08cad3a74cd94/0000004d--8360486eb1 seg 3 (2026-05-11): the brake-hold
# carcontroller was unconditionally injecting ES_Brake=0 on bus 0 and
# Brake_Status mask on cam bus while Eyesight ACC was actively braking from
# rolling speed. This overrode Eyesight's brake command and hid the braking
# module's feedback, triggering Eyesight's ~566ms Cruise_Fault watchdog.
#
# Fix invariant: when we are NOT actively holding AND NOT echoing AEB, the
# carcontroller must transmit NEITHER 0x220 (create_es_brake_hold) NOR 0x13C
# (create_brake_status_hold) — let Eyesight's own ES_Brake reach the braking
# module and the module's Brake_Status reach Eyesight.
# ---------------------------------------------------------------------------

class TestACCInterferenceRegression:

  def test_brake_hold_active_false_on_init(self):
    """`_brake_hold_active` flag must exist and be False after construction."""
    ctrl = make_ctrl()
    assert hasattr(ctrl, "_brake_hold_active")
    assert ctrl._brake_hold_active is False

  def test_no_es_brake_hold_when_acc_braking_not_holding(self):
    """ACC actively braking from rolling speed, not holding → no create_es_brake_hold call.

    Reproduces the failing scenario: longActive=False (Eyesight ACC), BRAKE_HOLD flag set,
    standstill=False, AEB_Status=0, Brake_Pressure>0 (Eyesight is requesting braking).
    Old behavior overrode this with brake_value=0; new behavior must stay out of the way.
    """
    ctrl = make_ctrl(long_control=False)
    ctrl.frame = 0
    ctrl._brake_hold_primed = False
    acc_braking_msg = {
      "AEB_Status": 0, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 40,
      "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
      "Cruise_Activated": 1, "Signal3": 0,
    }
    CC = make_CC(enabled=True, long_active=False)
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC=CC, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(vEgoRaw=2.32, standstill=False, brakePressed=False,
                 gasPressed=False, es_brake_msg=acc_braking_msg),
    )
    assert not mock_bh.called, "create_es_brake_hold must not be called during ACC braking when not holding"
    assert not mock_bsh.called, "create_brake_status_hold must not be called during ACC braking when not holding"

  def test_no_brake_status_mask_when_not_holding(self):
    """Idle (not holding, not AEB): create_brake_status_hold must NOT be called.

    The mask is only valid while we are actively asserting brake hold; otherwise
    Eyesight must see the real Brake_Status (ES_Brake feedback) via Panda relay.
    """
    ctrl = make_ctrl()
    ctrl.frame = 0  # frame%2 == 0 — would otherwise trigger the mask send
    ctrl._brake_hold_primed = False
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=False, brakePressed=False, gasPressed=False, vEgoRaw=5.0),
    )
    assert not mock_bh.called
    assert not mock_bsh.called

  def test_holding_path_unchanged_regression(self):
    """When actively holding, both 0x220 (with BRAKE_HOLD_PRESSURE) and 0x13C mask must still be sent."""
    ctrl = make_ctrl()
    ctrl.frame = 0  # frame%5==0 and frame%2==0
    ctrl._brake_hold_primed = True
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    assert mock_bh.called
    # Args: (packer, frame, es_brake_msg, long_enabled, brake_value)
    assert mock_bh.call_args[0][3] is False  # stock Eyesight ACC — forward fault verbatim
    assert mock_bh.call_args[0][4] == CarControllerParams.BRAKE_HOLD_PRESSURE
    assert mock_bsh.called

  def test_aeb_passthrough_keeps_both_messages(self):
    """AEB active (non-zero AEB_Status), not standstill → echo Brake_Pressure AND send mask.

    AEB counts as active intercept: we DO want our ES_Brake echo to reach the
    braking module to preserve full AEB authority, and we DO want the mask to
    keep Eyesight from seeing redundant ES_Brake feedback during AEB.
    """
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = False
    aeb_msg = {
      "AEB_Status": 4, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 600,
      "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
      "Cruise_Activated": 0, "Signal3": 0,
    }
    mock_bh, mock_bsh = run_update_capture_both(
      ctrl, CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=False, vEgoRaw=8.0, brakePressed=False,
                 gasPressed=False, es_brake_msg=aeb_msg),
    )
    assert mock_bh.called
    assert mock_bh.call_args[0][4] == 600
    assert mock_bsh.called, "AEB must keep the Brake_Status mask active"

  def test_brake_hold_active_tracks_state(self):
    """`_brake_hold_active` flag must be True iff (holding or AEB)."""
    ctrl = make_ctrl()
    ctrl.frame = 0

    # Case 1: idle ACC braking (regression scenario) → False
    ctrl._brake_hold_primed = False
    acc_msg = {"AEB_Status": 0, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 40,
               "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
               "Cruise_Activated": 1, "Signal3": 0}
    run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                            CS=make_CS(standstill=False, vEgoRaw=2.3, es_brake_msg=acc_msg))
    assert ctrl._brake_hold_active is False

    # Case 2: holding → True
    ctrl._brake_hold_primed = True
    ctrl.frame = 0
    run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                            CS=make_CS(standstill=True, brakePressed=False, gasPressed=False))
    assert ctrl._brake_hold_active is True

    # Case 3: AEB → True
    ctrl._brake_hold_primed = False
    ctrl.frame = 0
    aeb_msg = {"AEB_Status": 4, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 600,
               "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
               "Cruise_Activated": 0, "Signal3": 0}
    run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                            CS=make_CS(standstill=False, vEgoRaw=8.0, es_brake_msg=aeb_msg))
    assert ctrl._brake_hold_active is True

  def test_brake_status_msg_none_guard(self):
    """CS.brake_status_msg=None → create_brake_status_hold must not be called even while holding."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CS = make_CS(standstill=True, brakePressed=False, gasPressed=False)
    CS.brake_status_msg = None
    mock_bh, mock_bsh = run_update_capture_both(ctrl, CC_SP=make_CC_SP(mads_enabled=True), CS=CS)
    assert mock_bh.called  # ES_Brake hold still goes out
    assert not mock_bsh.called  # mask gated on brake_status_msg presence


# ---------------------------------------------------------------------------
# TestAlphaLongCoexistence
#
# After Task 3 the brake-hold injection must work whenever:
#   CP.openpilotLongitudinalControl=True AND CC.longActive=False
# And must yield (no create_es_brake_hold call; create_es_brake instead) when:
#   CP.openpilotLongitudinalControl=True AND CC.longActive=True
# ---------------------------------------------------------------------------

_ES_BRAKE_PATCH = "opendbc.car.subaru.subarucan.create_es_brake"
_ES_STATUS_PATCH = "opendbc.car.subaru.subarucan.create_es_status"
_ES_DISTANCE_PATCH = "opendbc.car.subaru.subarucan.create_es_distance"


def _run_long_branch(ctrl, CC, CC_SP=None, CS=None):
  """Run ctrl.update() with all op-long-branch send fns patched.

  Returns (mock_es_brake, mock_es_brake_hold, mock_brake_status_hold).
  """
  CC_SP = CC_SP or make_CC_SP()
  CS = CS or make_CS()
  with _STEERING_PATCH, _DASHSTATUS_PATCH, _LKAS_STATE_PATCH, \
       patch(_ES_STATUS_PATCH, return_value=_DUMMY_MSG), \
       patch(_ES_DISTANCE_PATCH, return_value=_DUMMY_MSG), \
       patch(_ES_BRAKE_PATCH, return_value=_DUMMY_MSG) as mock_eb, \
       patch(_BRAKE_HOLD_PATCH, return_value=_DUMMY_MSG) as mock_bh, \
       patch(_BRAKE_STATUS_HOLD_PATCH, return_value=_DUMMY_BS_MSG) as mock_bsh:
    ctrl.update(CC, CC_SP, CS, 0)
  return mock_eb, mock_bh, mock_bsh


class TestAlphaLongCoexistence:

  def test_avh_fires_when_alpha_long_enabled_but_not_active(self):
    """openpilotLongitudinalControl=True, CC.longActive=False, holding conditions met
    → create_es_brake_hold MUST be called (AVH takes over ES_Brake)."""
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=False, long_active=False)
    mock_eb, mock_bh, _ = _run_long_branch(
      ctrl, CC,
      CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    assert mock_bh.called, "create_es_brake_hold must run when alpha long enabled but inactive"
    assert not mock_eb.called, "create_es_brake must NOT run when AVH is asserting ES_Brake"
    # Args: (packer, frame, es_brake_msg, long_enabled, brake_value)
    assert mock_bh.call_args[0][3] is True  # alpha long enabled — clear the latched fault
    assert mock_bh.call_args[0][4] == CarControllerParams.BRAKE_HOLD_PRESSURE

  def test_avh_yields_when_alpha_long_active(self):
    """openpilotLongitudinalControl=True, CC.longActive=True
    → create_es_brake_hold MUST NOT be called; create_es_brake runs instead."""
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=True, long_active=True)
    mock_eb, mock_bh, _ = _run_long_branch(
      ctrl, CC,
      CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    assert not mock_bh.called, "create_es_brake_hold must yield to op long when long_active=True"
    assert mock_eb.called, "create_es_brake must run when long is actively in control"

  def test_brake_status_mask_under_alpha_long_when_avh_active(self):
    """openpilotLongitudinalControl=True, AVH actively holding → Brake_Status mask must apply.

    Mask runs at frame % 2 == 0 and is independent of the long-control branch.
    """
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=False, long_active=False)
    _, _, mock_bsh = _run_long_branch(
      ctrl, CC,
      CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    assert mock_bsh.called, "Brake_Status mask must run when AVH active under alpha long"

  def test_brake_status_mask_yields_under_alpha_long_active(self):
    """openpilotLongitudinalControl=True, CC.longActive=True → no Brake_Status mask
    (op long owns the brake message exchange)."""
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    CC = make_CC(enabled=True, long_active=True)
    _, _, mock_bsh = _run_long_branch(
      ctrl, CC,
      CC_SP=make_CC_SP(mads_enabled=True),
      CS=make_CS(standstill=True, brakePressed=False, gasPressed=False),
    )
    assert not mock_bsh.called, "Brake_Status mask must not run when long is active"
