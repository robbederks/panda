#pragma once

#include "safety_declarations.h"

#define EPS_TORQUE_FACTOR 88 // corolla
#define TOYOTA_LTA_MAX_ANGLE 1657 // EPS only accepts up to 94.9461

static void carbage_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  // steering angle + torque
  if ((bus == 0) && (addr == 0x260)) {
    int torque_meas_new = (GET_BYTE(to_push, 5) << 8) | GET_BYTE(to_push, 6);
    torque_meas_new = to_signed(torque_meas_new, 16);

    // scale by dbc_factor
    torque_meas_new = (torque_meas_new * EPS_TORQUE_FACTOR) / 100;

    // update array of sample
    update_sample(&torque_meas, torque_meas_new);

    // increase torque_meas by 1 to be conservative on rounding
    torque_meas.min--;
    torque_meas.max++;

    // driver torque for angle limiting
    int torque_driver_new = (GET_BYTE(to_push, 1) << 8) | GET_BYTE(to_push, 2);
    torque_driver_new = to_signed(torque_driver_new, 16);
    update_sample(&torque_driver, torque_driver_new);

    // LTA request angle should match current angle while inactive, clipped to max accepted angle.
    // note that angle can be relative to init angle on some TSS2 platforms, LTA has the same offset
    bool steer_angle_initializing = GET_BIT(to_push, 3U);
    if (!steer_angle_initializing) {
      int angle_meas_new = (GET_BYTE(to_push, 3) << 8U) | GET_BYTE(to_push, 4);
      angle_meas_new = CLAMP(to_signed(angle_meas_new, 16), -TOYOTA_LTA_MAX_ANGLE, TOYOTA_LTA_MAX_ANGLE);
      update_sample(&angle_meas, angle_meas_new);
    }
  }

  // CBP_status
  if ((bus == 0) && (addr == 0x100)) {
    float speed = ((GET_BYTE(to_push, 1) << 8) | GET_BYTE(to_push, 0)) / 36.0;
    vehicle_moving = ABS(speed) > 1;
    UPDATE_VEHICLE_SPEED(speed);

    bool cruise_engaged = (GET_BYTE(to_push, 5) & 0x1) != 0;
    pcm_cruise_check(cruise_engaged);

    gas_pressed = ((GET_BYTE(to_push, 5) >> 1) & 0x1) != 0;
  }

  // IBST_private2
  if ((bus == 2) && (addr == 0x38F)) {
    brake_pressed = ((GET_BYTE(to_push, 2) & 0x1) != 0);
  }

  generic_rx_checks(false);
}


static bool carbage_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int bus = GET_BUS(to_send);
  int addr = GET_ADDR(to_send);

  const SteeringLimits TOYOTA_STEERING_LIMITS = {
    .max_steer = 1500,
    .max_rate_up = 15,          // ramp up slow
    .max_rate_down = 25,        // ramp down fast
    .max_torque_error = 350,    // max torque cmd in excess of motor torque
    .max_rt_delta = 450,        // the real time limit is 1800/sec, a 20% buffer
    .max_rt_interval = 250000,
    .type = TorqueMotorLimited,

    // the EPS faults when the steering angle rate is above a certain threshold for too long. to prevent this,
    // we allow setting STEER_REQUEST bit to 0 while maintaining the requested torque value for a single frame
    .min_valid_request_frames = 18,
    .max_invalid_request_frames = 1,
    .min_valid_request_rt_interval = 170000,  // 170ms; a ~10% buffer on cutting every 19 frames
    .has_steer_req_tolerance = true,

    // LTA angle limits
    // factor for STEER_TORQUE_SENSOR->STEER_ANGLE and STEERING_LTA->STEER_ANGLE_CMD (1 / 0.0573)
    .angle_deg_to_can = 17.452007,
    .angle_rate_up_lookup = {
      {5., 25., 25.},
      {0.3, 0.15, 0.15}
    },
    .angle_rate_down_lookup = {
      {5., 25., 25.},
      {0.36, 0.26, 0.26}
    },
  };

  if ((bus == 0) && (addr == 0x2E4)) {
    int desired_torque = (GET_BYTE(to_send, 1) << 8) | GET_BYTE(to_send, 2);
    desired_torque = to_signed(desired_torque, 16);
    bool steer_req = GET_BIT(to_send, 0U);
    if (steer_torque_cmd_checks(desired_torque, steer_req, TOYOTA_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // ServoControl
  if ((bus == 0) && (addr == 0x200)) {
    uint16_t desired_servo_percent = ((GET_BYTE(to_send, 1) & 0xF) << 8) | GET_BYTE(to_send, 0);
    if (desired_servo_percent > 100) {
      tx = false;
    }

    if ((!controls_allowed) && (desired_servo_percent != 0)) {
      tx = false;
    }
  }

  // BrakeControl
  if ((bus == 0) && (addr == 0x201)) {
    uint16_t desired_brake_raw = ((GET_BYTE(to_send, 1) & 0xF) << 8) | GET_BYTE(to_send, 0);

    // 0mm == 320 raw
    if ((!controls_allowed) && (desired_brake_raw != 320U)) {
      tx = false;
    }
  }

  return tx;
}

static int carbage_fwd_hook(int bus_num, int addr) {
  UNUSED(bus_num); UNUSED(addr);
  return -1;
}

static safety_config carbage_init(uint16_t param) {
  UNUSED(param);

  static const CanMsg CARBAGE_TX_MSGS[] = {
    {0x2E4, 0, 8},  // STEERING_LKA
    {0x200, 0, 3},  // STEERING_SERVO
    {0x201, 0, 3},  // STEERING_BRAKE
  };

  static RxCheck carbage_rx_checks[] = {
    {.msg = {{0x260, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x100, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},  // CBP_status
    {.msg = {{0x38F, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},  // IBST_private2
  };

  safety_config ret = BUILD_SAFETY_CFG(carbage_rx_checks, CARBAGE_TX_MSGS);

  return ret;
}

const safety_hooks carbage_hooks = {
  .init = carbage_init,
  .rx = carbage_rx_hook,
  .tx = carbage_tx_hook,
  .fwd = carbage_fwd_hook,
};
