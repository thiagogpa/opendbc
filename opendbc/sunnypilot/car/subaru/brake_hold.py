"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import IntEnum

from opendbc.car import structs
from opendbc.sunnypilot.car.subaru import subarucan_ext
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP

# Brake_Pedal (0x139) runs at 50 Hz — send every 2 control frames (100 Hz loop)
_BRAKE_HOLD_FRAME_DIVISOR = 2


class _State(IntEnum):
  IDLE = 0
  HOLDING = 1
  RELEASING = 2


class BrakeHoldController:
  """
  Pure logic state machine for Subaru manual brake hold (AVH).

  Transitions:
    IDLE     → HOLDING  : mads_active AND brakePressed AND standstill
    HOLDING  → RELEASING: gasPressed OR NOT mads_active
    RELEASING→ IDLE     : NOT standstill (car started moving)
    HOLDING  stays      : standstill AND mads_active (brake can be released by driver — hold maintained)

  Emits should_hold=True only in HOLDING state.
  CAN packing is handled by BrakeHoldCarController.
  """

  RELEASE_SPEED_THRESHOLD = 0.5  # m/s — above this, clear any residual hold

  def __init__(self) -> None:
    self._state = _State.IDLE
    self._last_pedal_raw: int = 0  # last non-zero Brake_Pedal value seen at standstill

  @property
  def state(self) -> _State:
    return self._state

  @property
  def should_hold(self) -> bool:
    return self._state == _State.HOLDING

  @property
  def last_pedal_raw(self) -> int:
    return self._last_pedal_raw

  def update(self, mads_active: bool, standstill: bool, brake_pressed: bool,
             gas_pressed: bool, v_ego: float, brake_pedal_raw: int) -> bool:
    """Update state machine. Returns should_hold."""

    # Track the last pedal value while driver is braking at standstill
    if standstill and brake_pressed and brake_pedal_raw > 0:
      self._last_pedal_raw = brake_pedal_raw

    if self._state == _State.IDLE:
      # Engage: MADS active + driver at a complete stop (brake released OR still pressed)
      if mads_active and standstill and (brake_pressed or self._last_pedal_raw > 0):
        # Only latch when we've actually seen the driver brake to a stop
        if self._last_pedal_raw > 0:
          self._state = _State.HOLDING

    elif self._state == _State.HOLDING:
      if gas_pressed or not mads_active or v_ego > self.RELEASE_SPEED_THRESHOLD:
        self._state = _State.RELEASING
        self._last_pedal_raw = 0

    elif self._state == _State.RELEASING:
      if not standstill or v_ego > self.RELEASE_SPEED_THRESHOLD:
        self._state = _State.IDLE
        self._last_pedal_raw = 0

    return self.should_hold


class BrakeHoldCarController:
  """Mixin for CarController — manages BrakeHoldController and produces cam-bus CAN sends."""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self._CP = CP
    self._enabled = bool(CP_SP.flags & SubaruFlagsSP.BRAKE_HOLD)
    self._controller = BrakeHoldController()

  @property
  def is_holding(self) -> bool:
    return self._enabled and self._controller.should_hold

  def create_brake_hold(self, packer, frame: int, CC, CC_SP, CS) -> list:
    """Call once per CarController.update() frame. Returns CAN messages (may be empty)."""
    if not self._enabled:
      return []

    # CS.brake_pedal_msg is a dict populated by SnGCarState from the pt-bus Brake_Pedal frame.
    # Fall back to empty dict at startup (before first CAN frame arrives).
    brake_pedal_msg: dict = getattr(CS, 'brake_pedal_msg', {})
    brake_pedal_raw = int(brake_pedal_msg.get("Brake_Pedal", 0))

    self._controller.update(
      mads_active=CC_SP.mads.active,
      standstill=CS.out.standstill,
      brake_pressed=CS.out.brakePressed,
      gas_pressed=CS.out.gasPressed,
      v_ego=CS.out.vEgoRaw,
      brake_pedal_raw=brake_pedal_raw,
    )

    if not self.is_holding:
      return []

    # No template yet — brake_pedal_msg empty at startup, skip until populated
    if not brake_pedal_msg:
      return []

    # Send at 50 Hz (every 2 frames of the 100 Hz control loop)
    if frame % _BRAKE_HOLD_FRAME_DIVISOR != 0:
      return []

    return [subarucan_ext.create_brake_hold_pedal(
      packer, brake_pedal_msg, self._controller.last_pedal_raw
    )]
