"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.sunnypilot.car.subaru.brake_hold import BrakeHoldController, _State


def _ctrl(**overrides):
  """Default kwargs for controller update — car moving, MADS off, no pedals."""
  defaults = dict(mads_active=False, standstill=False, brake_pressed=False,
                  gas_pressed=False, v_ego=10.0, brake_pedal_raw=0)
  defaults.update(overrides)
  return defaults


class TestBrakeHoldControllerColdStart:
  def test_no_spurious_hold_at_zero_mph_without_mads(self):
    """Car stationary at boot with no MADS — must stay IDLE."""
    ctrl = BrakeHoldController()
    for _ in range(10):
      result = ctrl.update(**_ctrl(mads_active=False, standstill=True, v_ego=0.0))
    assert ctrl.state == _State.IDLE
    assert result is False

  def test_no_hold_without_prior_brake_press(self):
    """MADS active + standstill but driver never braked to stop — no hold."""
    ctrl = BrakeHoldController()
    result = ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=False,
                                 brake_pedal_raw=0, v_ego=0.0))
    assert ctrl.state == _State.IDLE
    assert result is False

  def test_no_hold_while_moving(self):
    """MADS active, brake pressed, but car still moving — must not latch."""
    ctrl = BrakeHoldController()
    result = ctrl.update(**_ctrl(mads_active=True, standstill=False, brake_pressed=True,
                                 brake_pedal_raw=96, v_ego=2.0))
    assert ctrl.state == _State.IDLE
    assert result is False


class TestBrakeHoldControllerEngagement:
  def test_hold_engages_after_mads_brake_standstill(self):
    """Happy path: MADS + brake-to-stop → HOLDING."""
    ctrl = BrakeHoldController()
    # Approach: braking while still moving (pedal value recorded)
    ctrl.update(**_ctrl(mads_active=True, standstill=False, brake_pressed=True,
                        brake_pedal_raw=96, v_ego=0.5))
    # Reach standstill with brake still pressed
    result = ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                                 brake_pedal_raw=96, v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    assert result is True

  def test_hold_maintained_when_driver_releases_brake_at_standstill(self):
    """Driver releases brake pedal at standstill while HOLDING — hold continues."""
    ctrl = BrakeHoldController()
    ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                        brake_pedal_raw=96, v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    # Driver releases brake — still holding (this is the whole point)
    result = ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=False,
                                 brake_pedal_raw=0, v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    assert result is True

  def test_hold_maintained_multiple_frames(self):
    """Hold is stable across many frames without release trigger."""
    ctrl = BrakeHoldController()
    ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                        brake_pedal_raw=96, v_ego=0.0))
    for _ in range(50):
      result = ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=False,
                                   brake_pedal_raw=0, v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    assert result is True


class TestBrakeHoldControllerRelease:
  def _reach_holding(self) -> BrakeHoldController:
    ctrl = BrakeHoldController()
    ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                        brake_pedal_raw=96, v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    return ctrl

  def test_gas_press_releases_hold(self):
    """Gas pressed in HOLDING → RELEASING within one frame."""
    ctrl = self._reach_holding()
    ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=False,
                        gas_pressed=True, v_ego=0.0))
    assert ctrl.state == _State.RELEASING

  def test_mads_deactivation_releases_hold(self):
    """MADS turned off mid-hold → RELEASING."""
    ctrl = self._reach_holding()
    ctrl.update(**_ctrl(mads_active=False, standstill=True, v_ego=0.0))
    assert ctrl.state == _State.RELEASING

  def test_speed_above_threshold_releases_hold(self):
    """Car exceeds RELEASE_SPEED_THRESHOLD while in HOLDING → RELEASING."""
    ctrl = self._reach_holding()
    ctrl.update(**_ctrl(mads_active=True, standstill=False,
                        v_ego=BrakeHoldController.RELEASE_SPEED_THRESHOLD + 0.1))
    assert ctrl.state == _State.RELEASING

  def test_releasing_clears_to_idle_when_moving(self):
    """RELEASING → IDLE when car starts moving (standstill cleared)."""
    ctrl = self._reach_holding()
    # Trigger release
    ctrl.update(**_ctrl(mads_active=True, gas_pressed=True, standstill=True, v_ego=0.0))
    assert ctrl.state == _State.RELEASING
    # Car moves
    ctrl.update(**_ctrl(mads_active=True, standstill=False, v_ego=1.0))
    assert ctrl.state == _State.IDLE

  def test_releasing_clears_to_idle_above_threshold(self):
    """RELEASING → IDLE when speed exceeds threshold (no standstill change needed)."""
    ctrl = self._reach_holding()
    ctrl.update(**_ctrl(mads_active=True, gas_pressed=True, standstill=True, v_ego=0.0))
    assert ctrl.state == _State.RELEASING
    ctrl.update(**_ctrl(mads_active=True, standstill=False,
                        v_ego=BrakeHoldController.RELEASE_SPEED_THRESHOLD + 0.5))
    assert ctrl.state == _State.IDLE

  def test_full_cycle_re_engages(self):
    """After full release cycle, can engage hold again on next stop."""
    ctrl = self._reach_holding()
    ctrl.update(**_ctrl(mads_active=True, gas_pressed=True, standstill=False, v_ego=1.0))
    # May still be RELEASING if standstill not yet cleared
    ctrl.update(**_ctrl(mads_active=True, standstill=False, v_ego=2.0))
    assert ctrl.state == _State.IDLE
    # New stop
    ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                        brake_pedal_raw=80, v_ego=0.0))
    assert ctrl.state == _State.HOLDING


class TestBrakeHoldControllerBug4Regression:
  """Bug 4 from backup/implemented-features: hold must survive latActive=False at standstill.

  At v=0.18 m/s → 0 m/s, openpilot sets latActive=False momentarily. Previous attempt
  incorrectly used latActive instead of mads.active as the hold gate, causing spurious
  release. mads.active remains True through this transition — confirmed by Phase 0 drive log.
  """

  def test_hold_survives_low_speed_approach(self):
    """Approach at 0.18 m/s (latActive=False window) must not drop last_pedal_raw."""
    ctrl = BrakeHoldController()
    # At 0.18 m/s — below latActive threshold but mads is still active
    ctrl.update(**_ctrl(mads_active=True, standstill=False, brake_pressed=True,
                        brake_pedal_raw=96, v_ego=0.18))
    # Car reaches full stop
    result = ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                                 brake_pedal_raw=96, v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    assert result is True

  def test_hold_does_not_use_latactive(self):
    """Controller has no latActive parameter — mads_active drives hold, not lat control."""
    ctrl = BrakeHoldController()
    # Reach HOLDING
    ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                        brake_pedal_raw=96, v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    # Simulate latActive=False scenario: mads still active, lat just disengaged temporarily
    # (mads_active=True regardless of lat — this mirrors the actual MADS state machine)
    result = ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=False,
                                 v_ego=0.0))
    assert ctrl.state == _State.HOLDING
    assert result is True


class TestBrakeHoldControllerProperties:
  def test_should_hold_property_matches_state(self):
    ctrl = BrakeHoldController()
    assert ctrl.should_hold is False
    ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                        brake_pedal_raw=96, v_ego=0.0))
    assert ctrl.should_hold is True

  def test_state_property_accessible(self):
    ctrl = BrakeHoldController()
    assert ctrl.state == _State.IDLE

  def test_update_returns_should_hold(self):
    ctrl = BrakeHoldController()
    result = ctrl.update(**_ctrl(mads_active=True, standstill=True, brake_pressed=True,
                                 brake_pedal_raw=96, v_ego=0.0))
    assert result is ctrl.should_hold
