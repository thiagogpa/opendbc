from unittest.mock import MagicMock
import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.subaru.values import DBC, CAR, CanBus
from opendbc.car.subaru import subarucan


def make_packer():
  packer = MagicMock()
  packer.make_can_msg.return_value = ("ES_Brake", b'\x00' * 8, 0)
  return packer


def make_es_brake_msg(**overrides):
  base = {
    "CHECKSUM": 0,
    "Signal1": 0,
    "Brake_Pressure": 0,
    "AEB_Status": 0,
    "Cruise_Brake_Lights": 0,
    "Cruise_Brake_Fault": 0,
    "Cruise_Brake_Active": 0,
    "Cruise_Activated": 0,
    "Signal3": 0,
  }
  base.update(overrides)
  return base


def get_values(packer):
  _, _, values = packer.make_can_msg.call_args[0]
  return values


# --- Brake_Pressure / Cruise_Brake_Active / Cruise_Brake_Lights ---

@pytest.mark.parametrize("brake_value,active,lights", [
  (0, False, False),
  (50, True, False),   # active >0, below 70-unit lights threshold
  (70, True, True),    # lights threshold is inclusive
  (100, True, True),
  (600, True, True),
])
def test_brake_field_packing(brake_value, active, lights):
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=brake_value)
  v = get_values(packer)
  assert v["Brake_Pressure"] == brake_value
  assert bool(v["Cruise_Brake_Active"]) == active
  assert bool(v["Cruise_Brake_Lights"]) == lights


def test_stock_path_packs_brake_fields():
  # long_enabled=False still derives Active/Lights from brake_value; only fault handling differs
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=False, brake_value=100)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 100
  assert v["Cruise_Brake_Active"]
  assert v["Cruise_Brake_Lights"]


# --- Cruise_Brake_Fault: cleared on op-long, forwarded on stock ---

def test_cruise_brake_fault_cleared_when_long_enabled():
  # Eyesight latches Cruise_Brake_Fault=1 after op-long ACC engages; forwarding it poisoned the
  # AVH hold (car rolled), so op-long forces it to 0. Route dde08cad3a74cd94/00000014--7d3ff4c569.
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=True, brake_value=100)
  assert get_values(packer)["Cruise_Brake_Fault"] == 0


def test_cruise_brake_fault_forwarded_on_stock():
  # stock Eyesight ACC: the fault is genuinely Eyesight's, forward verbatim
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=False, brake_value=100)
  assert get_values(packer)["Cruise_Brake_Fault"] == 1


def test_hold_pressure_with_eyesight_fault_latched():
  # regression: 600-unit hold must still go out with fault cleared when Eyesight has latched the fault
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=True, brake_value=600)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 600
  assert v["Cruise_Brake_Active"]
  assert v["Cruise_Brake_Fault"] == 0


# --- Passthrough fields ---

def test_cruise_activated_passthrough():
  # forwarded verbatim — NOT overridden, unlike create_es_brake
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Activated=1), long_enabled=True, brake_value=100)
  assert get_values(packer)["Cruise_Activated"] == 1


@pytest.mark.parametrize("signal,value", [("AEB_Status", 8), ("Signal1", 5), ("Signal3", 3)])
def test_signal_passthrough(signal, value):
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(**{signal: value}), long_enabled=True, brake_value=0)
  assert get_values(packer)[signal] == value


# --- COUNTER ---

@pytest.mark.parametrize("frame,counter", [(0, 0), (15, 15), (16, 0)])  # 16 % 16 == 0 wraps
def test_counter(frame, counter):
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=frame, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
  assert get_values(packer)["COUNTER"] == counter


# --- Real CANPacker smoke test (catches DBC signal-name drift; also covers msg name + bus) ---

def test_real_packer_accepts_signals():
  packer = CANPacker(DBC[CAR.SUBARU_IMPREZA_2020.value][Bus.pt])
  addr, dat, bus = subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(),
                                                  long_enabled=True, brake_value=600)
  assert addr == 0x220
  assert bus == CanBus.main
  assert len(dat) == 8
