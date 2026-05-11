"""
Tests for brake hold logic in CarController.update().

Logic under test (opendbc_repo/opendbc/car/subaru/carcontroller.py, inside `else` branch):

  if self.CP.flags & SubaruFlags.BRAKE_HOLD and CS.es_brake_msg is not None:
      # velocity-primed latch
      if CC_SP.mads.enabled and CS.out.vEgoRaw < 1.5 and CS.out.brakePressed:
          if CS.out.gearShifter not in (GearShifter.park, GearShifter.reverse):
              self._brake_hold_primed = True

      if self.frame % 5 == 0:
          holding = (self._brake_hold_primed
                     and CS.out.standstill
                     and not CS.out.gasPressed
                     and CS.out.gearShifter not in (GearShifter.park, GearShifter.reverse))

          if CS.out.gasPressed or not CC_SP.mads.enabled or CS.out.vEgoRaw > 0.5:
              self._brake_hold_primed = False

          # AEB safety: echo EyeSight's own Brake_Pressure when it asserts AEB.
          if CS.es_brake_msg["AEB_Status"] != 0:
              brake_value = CS.es_brake_msg["Brake_Pressure"]
          elif holding:
              brake_value = CarControllerParams.BRAKE_HOLD_PRESSURE
          else:
              brake_value = 0

          can_sends.append(subarucan.create_es_brake_hold(
              self.packer, self.frame // 5, CS.es_brake_msg,
              brake_value
          ))
"""

import pytest
from unittest.mock import MagicMock, patch, call

from opendbc.car import structs
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

  def test_priming_requires_mads_enabled(self):
    """mads.enabled=False → primed stays False even if speed/brake conditions met."""
    ctrl = make_ctrl()
    ctrl.frame = 1  # not a frame%5 boundary to isolate priming from hold logic
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=False),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True))
    assert ctrl._brake_hold_primed is False

  def test_priming_requires_brake_pressed(self):
    """brakePressed=False → primed stays False."""
    ctrl = make_ctrl()
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=False))
    assert ctrl._brake_hold_primed is False

  def test_priming_requires_low_speed(self):
    """vEgoRaw >= 1.5 m/s → primed stays False."""
    ctrl = make_ctrl()
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=2.0, brakePressed=True))
    assert ctrl._brake_hold_primed is False

  def test_priming_blocked_in_park(self):
    """Gear=park → primed stays False even if all other conditions met."""
    ctrl = make_ctrl()
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True, gear=GearShifter.park))
    assert ctrl._brake_hold_primed is False

  def test_priming_blocked_in_reverse(self):
    """Gear=reverse → primed stays False."""
    ctrl = make_ctrl()
    ctrl.frame = 1
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True, gear=GearShifter.reverse))
    assert ctrl._brake_hold_primed is False

  def test_priming_succeeds(self):
    """All priming conditions met → _brake_hold_primed becomes True."""
    ctrl = make_ctrl()
    ctrl.frame = 1  # odd frame: priming runs but frame%5 != 0 so no reset
    run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
               CS=make_CS(vEgoRaw=0.5, brakePressed=True, gear=GearShifter.drive))
    assert ctrl._brake_hold_primed is True

  # -----------------------------------------------------------------------
  # 7–14. Holding conditions (verified via create_es_brake_hold call args)
  #        Tests set ctrl.frame = 0 so frame%5 == 0 path is taken.
  #        For tests that need primed=True we manually set the flag.
  # -----------------------------------------------------------------------

  def _get_brake_value(self, mock_bh):
    """Extract the brake_value positional arg (4th arg) from the mock call."""
    assert mock_bh.called, "create_es_brake_hold was not called"
    return mock_bh.call_args[0][3]  # (packer, frame, es_brake_msg, brake_value)

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

  def test_aeb_overrides_hold(self):
    """AEB_Status != 0 → brake_value passes through EyeSight's Brake_Pressure, not hold pressure."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    aeb_es_msg = {"AEB_Status": 8, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 450,
                  "Cruise_Brake_Lights": 0, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 0,
                  "Cruise_Activated": 0, "Signal3": 0}
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False,
                                    gasPressed=False, es_brake_msg=aeb_es_msg))
    assert self._get_brake_value(mock_bh) == 450

  def test_aeb_zero_does_not_override(self):
    """AEB_Status=0 → normal hold logic, brake_value == BRAKE_HOLD_PRESSURE."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=True, brakePressed=False,
                                    gasPressed=False, aeb_status=0))
    assert self._get_brake_value(mock_bh) == CarControllerParams.BRAKE_HOLD_PRESSURE

  def test_aeb_while_moving_passthrough(self):
    """AEB fires while car is moving (standstill=False, not holding) → passthrough EyeSight's Brake_Pressure."""
    ctrl = make_ctrl()
    ctrl.frame = 0
    ctrl._brake_hold_primed = False
    aeb_es_msg = {"AEB_Status": 4, "CHECKSUM": 0, "Signal1": 0, "Brake_Pressure": 600,
                  "Cruise_Brake_Lights": 1, "Cruise_Brake_Fault": 0, "Cruise_Brake_Active": 1,
                  "Cruise_Activated": 0, "Signal3": 0}
    mock_bh = run_update(ctrl, CC_SP=make_CC_SP(mads_enabled=True),
                         CS=make_CS(standstill=False, brakePressed=False,
                                    gasPressed=False, vEgoRaw=8.0, es_brake_msg=aeb_es_msg))
    assert self._get_brake_value(mock_bh) == 600

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
  # 19. Guard: openpilotLongitudinalControl=True → no call
  # -----------------------------------------------------------------------

  def test_not_in_long_branch(self):
    """openpilotLongitudinalControl=True puts update in a different branch.

    The brake hold code lives inside `else` (longControl=False).
    When longControl=True, create_es_brake_hold must NOT be called.
    """
    ctrl = make_ctrl(long_control=True, brake_hold=True)
    ctrl.frame = 0
    ctrl._brake_hold_primed = True
    # When longControl=True, subarucan.create_es_brake and create_es_status are
    # called instead. Patch those to avoid KeyErrors on mock CS.
    with patch("opendbc.car.subaru.subarucan.create_steering_control", return_value=_DUMMY_MSG), \
         patch("opendbc.car.subaru.subarucan.create_es_dashstatus", return_value=_DUMMY_MSG), \
         patch("opendbc.car.subaru.subarucan.create_es_lkas_state", return_value=_DUMMY_MSG), \
         patch("opendbc.car.subaru.subarucan.create_es_status", return_value=_DUMMY_MSG), \
         patch("opendbc.car.subaru.subarucan.create_es_brake", return_value=_DUMMY_MSG), \
         patch("opendbc.car.subaru.subarucan.create_es_distance", return_value=_DUMMY_MSG), \
         patch(_BRAKE_HOLD_PATCH, return_value=_DUMMY_MSG) as mock_bh:
      ctrl.update(make_CC(), make_CC_SP(), make_CS(), 0)
    assert not mock_bh.called

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
  # Bonus: verify BRAKE_HOLD_PRESSURE is a non-zero constant (sanity check)
  # -----------------------------------------------------------------------

  def test_brake_hold_pressure_nonzero(self):
    """CarControllerParams.BRAKE_HOLD_PRESSURE must be a positive integer."""
    assert isinstance(CarControllerParams.BRAKE_HOLD_PRESSURE, int)
    assert CarControllerParams.BRAKE_HOLD_PRESSURE > 0


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
    # Fourth positional arg is brake_value
    assert mock_bh.call_args[0][3] == CarControllerParams.BRAKE_HOLD_PRESSURE
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
    assert mock_bh.call_args[0][3] == 600
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
