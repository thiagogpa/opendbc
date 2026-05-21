"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tests that the SubaruAutoVehicleHold param gates the brake-hold (AVH) flag and
the BRAKE_INTERCEPT safety param in _initialize_brake_intercept, exercised via
the public setup_interfaces() entry point.
"""
from unittest.mock import MagicMock

from opendbc.car import structs
from opendbc.car.subaru.values import SubaruFlags
from opendbc.sunnypilot.car.interfaces import setup_interfaces
from opendbc.sunnypilot.car.subaru.values_ext import SubaruSafetyFlagsSP


def _run_setup(param_value: str, alpha_long_available: bool = True):
  CP = structs.CarParams()
  CP.brand = 'subaru'
  CP.alphaLongitudinalAvailable = alpha_long_available
  CP_SP = structs.CarParamsSP()
  CI = MagicMock()
  setup_interfaces(CI, CP, CP_SP, params_list=[{"SubaruAutoVehicleHold": param_value}])
  return CP, CP_SP


class TestBrakeInterceptInit:
  def test_flag_not_set_when_param_off(self):
    CP, CP_SP = _run_setup("0")
    assert not (CP.flags & SubaruFlags.BRAKE_HOLD)
    assert not (CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)

  def test_flag_set_when_param_on(self):
    CP, CP_SP = _run_setup("1")
    assert CP.flags & SubaruFlags.BRAKE_HOLD
    assert CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT

  def test_flag_not_set_when_ineligible_even_if_param_on(self):
    CP, CP_SP = _run_setup("1", alpha_long_available=False)
    assert not (CP.flags & SubaruFlags.BRAKE_HOLD)
    assert not (CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)

  def test_brake_intercept_preserves_sng_bit(self):
    """SnG + AVH both on → BRAKE_INTERCEPT is OR-added without clobbering the SnG safety bit."""
    CP = structs.CarParams()
    CP.brand = 'subaru'
    CP.alphaLongitudinalAvailable = True
    CP_SP = structs.CarParamsSP()
    CI = MagicMock()
    setup_interfaces(CI, CP, CP_SP, params_list=[{"SubaruStopAndGo": "1", "SubaruAutoVehicleHold": "1"}])
    assert CP_SP.safetyParam & SubaruSafetyFlagsSP.STOP_AND_GO      # SnG bit preserved
    assert CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT  # AVH bit added

  def test_non_subaru_brand_untouched(self):
    """A non-Subaru brand must not get the AVH flag or safety bit even with the param on."""
    CP = structs.CarParams()
    CP.brand = 'toyota'
    CP.alphaLongitudinalAvailable = True
    CP_SP = structs.CarParamsSP()
    CI = MagicMock()
    setup_interfaces(CI, CP, CP_SP, params_list=[{"SubaruAutoVehicleHold": "1"}])
    assert not (CP.flags & SubaruFlags.BRAKE_HOLD)
    assert not (CP_SP.safetyParam & SubaruSafetyFlagsSP.BRAKE_INTERCEPT)
