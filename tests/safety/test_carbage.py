#!/usr/bin/env python3
import numpy as np
import random
import unittest
import itertools

from panda import Panda
from panda.tests.libpanda import libpanda_py
import panda.tests.safety.common as common
from panda.tests.safety.common import CANPackerPanda

TOYOTA_COMMON_TX_MSGS = [[0x2E4, 0], [0x191, 0], [0x412, 0], [0x343, 0], [0x1D2, 0]]  # LKAS + LTA + ACC & PCM cancel cmds
TOYOTA_SECOC_TX_MSGS = [[0x131, 0]] + TOYOTA_COMMON_TX_MSGS
TOYOTA_COMMON_LONG_TX_MSGS = [[0x283, 0], [0x2E6, 0], [0x2E7, 0], [0x33E, 0], [0x344, 0], [0x365, 0], [0x366, 0], [0x4CB, 0],  # DSU bus 0
                              [0x128, 1], [0x141, 1], [0x160, 1], [0x161, 1], [0x470, 1],  # DSU bus 1
                              [0x411, 0],  # PCS_HUD
                              [0x750, 0]]  # radar diagnostic address


class TestCarbageSafety(common.PandaCarSafetyTest, common.LongitudinalAccelSafetyTest):

  TX_MSGS = TOYOTA_COMMON_TX_MSGS + TOYOTA_COMMON_LONG_TX_MSGS
  STANDSTILL_THRESHOLD = 0  # kph
  RELAY_MALFUNCTION_ADDRS = {0: (0x2E4, 0x343)}
  FWD_BLACKLISTED_ADDRS = {2: [0x2E4, 0x412, 0x191, 0x343]}
  FWD_BUS_LOOKUP = {0: 2, 2: 0}
  EPS_SCALE = 73

  packer: CANPackerPanda
  safety: libpanda_py.Panda

  MAX_RATE_UP = 15
  MAX_RATE_DOWN = 25
  MAX_TORQUE = 1500
  MAX_RT_DELTA = 450
  RT_INTERVAL = 250000
  MAX_TORQUE_ERROR = 350
  TORQUE_MEAS_TOLERANCE = 1  # toyota safety adds one to be conservative for rounding

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 18
  MAX_INVALID_STEERING_FRAMES = 1
  MIN_VALID_STEERING_RT_INTERVAL = 170000  # a ~10% buffer, can send steer up to 110Hz

  def setUp(self):
    self.packer = CANPackerPanda("carbagepilot")
    self.safety = libpanda_py.libpanda
    self.safety.set_safety_hooks(Panda.SAFETY_TOYOTA, self.EPS_SCALE)
    self.safety.init_tests()

  @classmethod
  def setUpClass(cls):
    if cls.__name__.endswith("Base"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def _torque_meas_msg(self, torque: int, driver_torque: int | None = None):
    values = {"STEER_TORQUE_EPS": (torque / self.EPS_SCALE) * 100.}
    if driver_torque is not None:
      values["STEER_TORQUE_DRIVER"] = driver_torque
    return self.packer.make_can_msg_panda("STEER_TORQUE_SENSOR", 0, values)

  # Both torque and angle safety modes test with each other's steering commands
  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEER_TORQUE_CMD": torque, "STEER_REQUEST": steer_req}
    return self.packer.make_can_msg_panda("STEERING_LKA", 0, values)

  def _angle_meas_msg(self, angle: float, steer_angle_initializing: bool = False):
    # This creates a steering torque angle message. Not set on all platforms,
    # relative to init angle on some older TSS2 platforms. Only to be used with LTA
    # values = {"STEER_ANGLE": angle, "STEER_ANGLE_INITIALIZING": int(steer_angle_initializing)}
    # return self.packer.make_can_msg_panda("STEER_TORQUE_SENSOR", 0, values)

    values = {"STEER_ANGLE": angle}
    return self.packer.make_can_msg_panda("STEER_ANGLE_SENSOR", 0, values)

  def _accel_msg(self, accel, cancel_req=0):
    values = {"ACCEL_CMD": accel, "CANCEL_REQ": cancel_req}
    return self.packer.make_can_msg_panda("ACC_CONTROL", 0, values)

  def _speed_msg(self, speed):
    values = {("WHEEL_SPEED_%s" % n): speed * 3.6 for n in ["FR", "FL", "RR", "RL"]}
    return self.packer.make_can_msg_panda("WHEEL_SPEEDS", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_panda("BRAKE_MODULE", 0, values)

  def _user_gas_msg(self, gas):
    cruise_active = self.safety.get_controls_allowed()
    values = {"GAS_RELEASED": not gas, "CRUISE_ACTIVE": cruise_active}
    return self.packer.make_can_msg_panda("PCM_CRUISE", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"CRUISE_ACTIVE": enable}
    return self.packer.make_can_msg_panda("PCM_CRUISE", 0, values)

  def test_rx_hook(self):
    # checksum checks
    for msg in ["trq", "pcm"]:
      self.safety.set_controls_allowed(1)
      if msg == "trq":
        to_push = self._torque_meas_msg(0)
      if msg == "pcm":
        to_push = self._pcm_status_msg(True)
      self.assertTrue(self._rx(to_push))
      to_push[0].data[4] = 0
      to_push[0].data[5] = 0
      to_push[0].data[6] = 0
      to_push[0].data[7] = 0
      self.assertFalse(self._rx(to_push))
      self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
