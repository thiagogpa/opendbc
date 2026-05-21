from unittest.mock import MagicMock
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
  """Extract values dict passed to make_can_msg."""
  call_args = packer.make_can_msg.call_args
  msg_name, bus, values = call_args[0]
  return values


# --- Brake_Pressure / Cruise_Brake_Active / Cruise_Brake_Lights ---

def test_zero_brake_value():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 0
  assert not v["Cruise_Brake_Active"]
  assert not v["Cruise_Brake_Lights"]


def test_100_brake_value():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=100)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 100
  assert v["Cruise_Brake_Active"]
  assert v["Cruise_Brake_Lights"]  # 100 >= 70


def test_below_lights_threshold():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=50)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 50
  assert v["Cruise_Brake_Active"]    # 50 > 0
  assert not v["Cruise_Brake_Lights"]  # 50 < 70


def test_at_lights_threshold():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=70)
  v = get_values(packer)
  assert v["Cruise_Brake_Lights"]  # exactly 70


def test_max_brake_value():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=600)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 600
  assert v["Cruise_Brake_Active"]
  assert v["Cruise_Brake_Lights"]


# --- Passthrough fields ---

def test_cruise_activated_passthrough():
  """Cruise_Activated forwarded verbatim — NOT overridden (unlike create_es_brake)."""
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Activated=1), long_enabled=True, brake_value=100)
  v = get_values(packer)
  assert v["Cruise_Activated"] == 1


def test_cruise_brake_fault_cleared_when_long_enabled():
  """With op long enabled, Cruise_Brake_Fault is forced to 0 — the car ignores a brake command
  that carries a fault flag. Eyesight latches Cruise_Brake_Fault=1 after op-long ACC engages once;
  forwarding it verbatim poisoned the AVH hold (car rolled). See route dde08cad3a74cd94/00000014--7d3ff4c569."""
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=True, brake_value=100)
  v = get_values(packer)
  assert v["Cruise_Brake_Fault"] == 0


def test_cruise_brake_fault_forwarded_on_stock():
  """With op long NOT enabled (stock Eyesight ACC), Cruise_Brake_Fault is forwarded verbatim.
  The latch only occurs once alpha-long ACC engages, which forces openpilotLongitudinalControl=True;
  on the stock path the fault is genuinely Eyesight's, so we don't fidget with it."""
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=False, brake_value=100)
  v = get_values(packer)
  assert v["Cruise_Brake_Fault"] == 1


def test_hold_pressure_with_eyesight_fault_latched():
  """Regression: post-alpha-long-ACC, Eyesight es_brake_msg has Cruise_Brake_Fault=1.
  With op long enabled the 600-unit hold must still go out with fault cleared so the car honors it."""
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Cruise_Brake_Fault=1), long_enabled=True, brake_value=600)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 600
  assert v["Cruise_Brake_Active"]
  assert v["Cruise_Brake_Fault"] == 0


def test_aeb_status_passthrough():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(AEB_Status=8), long_enabled=True, brake_value=0)
  v = get_values(packer)
  assert v["AEB_Status"] == 8


def test_signal1_passthrough():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Signal1=5), long_enabled=True, brake_value=0)
  v = get_values(packer)
  assert v["Signal1"] == 5


def test_signal3_passthrough():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(Signal3=3), long_enabled=True, brake_value=0)
  v = get_values(packer)
  assert v["Signal3"] == 3


# --- COUNTER ---

def test_counter_frame0():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
  v = get_values(packer)
  assert v["COUNTER"] == 0


def test_counter_wrap():
  """frame=16 wraps to COUNTER=0 (16 % 16 == 0)."""
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=16, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
  v = get_values(packer)
  assert v["COUNTER"] == 0


def test_counter_frame15():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=15, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
  v = get_values(packer)
  assert v["COUNTER"] == 15


# --- Message name and bus ---

def test_message_name():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
  call_args = packer.make_can_msg.call_args
  msg_name = call_args[0][0]
  assert msg_name == "ES_Brake"


def test_bus_is_main():
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=True, brake_value=0)
  call_args = packer.make_can_msg.call_args
  bus = call_args[0][1]
  assert bus == CanBus.main


# --- Stock-path (long_enabled=False) brake-field packing ---

def test_stock_path_sets_brake_active_and_lights():
  """long_enabled=False (stock Eyesight ACC): Cruise_Brake_Active / Cruise_Brake_Lights are still
  derived from brake_value. Only Cruise_Brake_Fault handling differs between the long/stock paths."""
  packer = make_packer()
  subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(), long_enabled=False, brake_value=100)
  v = get_values(packer)
  assert v["Brake_Pressure"] == 100
  assert v["Cruise_Brake_Active"]
  assert v["Cruise_Brake_Lights"]  # 100 >= 70


# --- Real CANPacker smoke test (catches DBC signal-name drift) ---

def test_real_packer_accepts_signals():
  """Pack with a REAL CANPacker so a typo'd / removed DBC signal name would raise, unlike the
  MagicMock packer used elsewhere. Asserts a valid ES_Brake frame (0x220, main bus, 8 bytes)."""
  packer = CANPacker(DBC[CAR.SUBARU_IMPREZA_2020.value][Bus.pt])
  addr, dat, bus = subarucan.create_es_brake_hold(packer, frame=0, es_brake_msg=make_es_brake_msg(),
                                                  long_enabled=True, brake_value=600)
  assert addr == 0x220
  assert bus == CanBus.main
  assert len(dat) == 8
