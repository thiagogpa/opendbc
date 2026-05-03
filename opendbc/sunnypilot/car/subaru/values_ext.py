"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import IntFlag


class SubaruSafetyFlagsSP:
  STOP_AND_GO = 1      # bit 0 — existing
  BRAKE_INTERCEPT = 4  # bit 2 — NEW; bit 1 intentionally unused


class SubaruFlagsSP(IntFlag):
  STOP_AND_GO = 1
  STOP_AND_GO_MANUAL_PARKING_BRAKE = 2
  BRAKE_HOLD = 4
