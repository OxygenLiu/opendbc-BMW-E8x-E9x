#!/usr/bin/env python3
import numpy as np
from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car import get_safety_config
from opendbc.car.bmw.values import CanBus, BmwFlags, CarControllerParams
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.bmw.carcontroller import CarController
from opendbc.car.bmw.carstate import CarState

TransmissionType = structs.CarParams.TransmissionType


# certain driver intervention can be distinguished from maximum estimated wheel turning force
def detect_stepper_override(steer_cmd, steer_act, v_ego, centering_coeff, steer_friction_torque):
  # when steering released (or lost steps), what angle will it return to
  # if we are above that angle, we can detect things
  release_angle = steer_friction_torque / (max(v_ego, 1) ** 2 * centering_coeff)

  override = False
  margin_value = 1
  # For higher angles steering will not move outward by itself with stepper on
  if abs(steer_cmd) > release_angle:
    if steer_cmd > 0:
      override |= steer_act - steer_cmd > margin_value  # driver overrode from right to more right
      override |= steer_act < 0  # releaseAngle -3  # driver overrode from right to opposite direction
    else:
      override |= steer_act - steer_cmd < -margin_value  # driver overrode from left to more left
      override |= steer_act > 0  # -releaseAngle +3 # driver overrode from left to opposite direction
  # else:
    # Driver overrode to an angle where steering will not go by itself
    # override |= abs(steerAct) > releaseAngle + marginVal
  return override


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP, *args, **kwargs):
    super().__init__(CP, *args, **kwargs)

    # BMW Variable Steer Ratio support
    # Enable for all BMW E90s based on route analysis showing 30% variation
    from opendbc.car.bmw.values import CAR
    self.variable_steer_ratio_enabled = (CP.carFingerprint == CAR.BMW_E90)
    self.default_steer_ratio = CP.steerRatio  # Store default ratio from CarParams


  #   self.cp_F = self.CS.get_F_can_parser(CP)
  #   self.can_parsers.append(self.cp_F)
  #   self.cp_aux = self.CS.get_actuator_can_parser(CP)
  #   self.can_parsers.append(self.cp_aux)

  @staticmethod
  # servotronic is a bit more lighter in general and especially at low speeds https://www.spoolstreet.com/threads/servotronic-on-a-335i.1400/page-13#post-117705
  def get_steer_feedforward_servotronic(desired_angle, v_ego): # accounts for steering rack ratio and/or caster nonlinearities https://www.spoolstreet.com/threads/servotronic-on-a-335i.1400/page-15#post-131271
    angle_bp = [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0] # deg
    hold_torque_v  = [-6, -2.85, -2.5, -2.25, -2, -1.65, -1, 1, 1.65, 2, 2.25, 2.5, 2.85, 6] # Nm
    hold_torque = np.interp(desired_angle, angle_bp, hold_torque_v)
    return hold_torque # todo add speed component

  @staticmethod
  def get_steer_feedforward(desired_angle, v_ego):
    angle_bp = [-40.0, -6.0, -4.0, -3.0, -2.0, -1.0, -0.5,  0.5,  1.0,  2.0,  3.0,  4.0,  6.0, 40.0] # deg
    hold_torque_v  = [-6, -2.85, -2.5, -2.25, -2, -1.65, -1, 1, 1.65, 2, 2.25, 2.5, 2.85, 6] # Nm
    hold_torque = np.interp(desired_angle, angle_bp, hold_torque_v)
    return hold_torque # todo add speed component

  @staticmethod
  def get_modelv2_velocity_index(actuator_delay_s):
    """
    Calculate optimal ModelV2 velocity index for actuator delay compensation

    Args:
        actuator_delay_s: longitudinalActuatorDelay in seconds

    Returns:
        int: ModelV2 velocity array index for actuator delay compensation
    """
    from selfdrive.modeld.constants import index_function, ModelConstants

    if actuator_delay_s <= 0:
        return 0

    # Generate ModelV2 time indices
    T_IDXS = [index_function(idx, max_val=10.0) for idx in range(ModelConstants.IDX_N)]

    if actuator_delay_s > T_IDXS[-1]:
        return len(T_IDXS) - 1

    # Find closest time index to actuator delay
    best_idx = 0
    min_error = float('inf')

    for i, t in enumerate(T_IDXS):
        error = abs(t - actuator_delay_s)
        if error < min_error:
            min_error = error
            best_idx = i

    return best_idx

  def get_steer_feedforward_function(self):
    if self.CP.flags & BmwFlags.SERVOTRONIC:
      return self.get_steer_feedforward_servotronic
    else:
      return self.get_steer_feedforward


  def get_variable_steer_ratio(self, steering_angle_deg, speed_ms):
    """
    BMW E90 Variable Steer Ratio with linear interpolation, hysteresis, and speed adjustment
    Based on measured data from route 00000048--891b50d865 (59,629 samples)
    """
    abs_angle = abs(steering_angle_deg)

    # Apply simple hysteresis by slightly adjusting the angle for interpolation
    hysteresis_offset = 1.0  # degrees of smoothing

    # If angle is increasing (turning more), use raw angle
    # If angle is decreasing (straightening), add slight offset for smoothing
    if hasattr(self, '_last_abs_angle'):
        if abs_angle < self._last_abs_angle:  # Angle decreasing
            smoothed_angle = abs_angle + hysteresis_offset
        else:  # Angle increasing or same
            smoothed_angle = abs_angle
    else:
        smoothed_angle = abs_angle

    # Store for next iteration
    self._last_abs_angle = abs_angle

    # Linear interpolation for smooth ratio transitions
    # Define angle breakpoints and corresponding ratios
    angle_breakpoints = [0, 10, 45, 90, 180]  # degrees
    ratio_values = [22.0, 22.0, 21.7, 18.5, 16.8]  # corresponding ratios

    # Use linear interpolation for smooth transitions
    import numpy as np
    base_ratio = float(np.interp(smoothed_angle, angle_breakpoints, ratio_values))

    # Speed-dependent adjustment with linear interpolation
    speed_kph = speed_ms * 3.6
    speed_breakpoints = [0, 30, 60, 100, 200]  # km/h
    speed_modifiers = [0.90, 0.90, 0.95, 1.0, 1.05]  # corresponding modifiers
    speed_modifier = float(np.interp(speed_kph, speed_breakpoints, speed_modifiers))

    # Apply speed modification
    adjusted_ratio = base_ratio * speed_modifier

    return adjusted_ratio


  def get_current_variable_steer_ratio(self):
    """
    Get the current speed-dependent variable steer ratio
    Calculates on-demand to avoid timing issues with update()
    """
    if not self.variable_steer_ratio_enabled:
      return self.default_steer_ratio  # Use CarParams default ratio

    # If we have recent cached data from update(), use it
    if hasattr(self, '_last_carstate') and self._last_carstate:
      cs = self._last_carstate
      # Calculate fresh ratio using cached CarState
      target_ratio = self.get_variable_steer_ratio(
        cs['angle'], cs['speed']
      )
      return target_ratio

    # Fallback: return default ratio
    return self.default_steer_ratio

  def update(self, can_packets):
    """Update CarInterface with CarState caching for on-demand variable steer ratio"""
    ret = super().update(can_packets)

    # Cache current CarState for on-demand variable steer ratio calculation
    if self.variable_steer_ratio_enabled:
      self._last_carstate = {
        'angle': ret.steeringAngleDeg,
        'speed': ret.vEgo
      }

    return ret

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    ret.brand = "bmw"

    # Runtime cruise control detection - PT-CAN (bus 0) messages
    has_normal_cruise = 0x200 in fingerprint.get(CanBus.PT_CAN, {})
    has_dynamic_cruise = 0x193 in fingerprint.get(CanBus.PT_CAN, {})
    has_ldm = 0x0D5 in fingerprint.get(CanBus.PT_CAN, {})

    # Runtime stepper servo detection - can be on SERVO_CAN (bus 1) or AUX_CAN (bus 2)
    # BMW panda safety accepts STEERING_STATUS (0x22F) on either bus
    if (0x22F in fingerprint.get(CanBus.SERVO_CAN, {}) or
        0x22F in fingerprint.get(CanBus.AUX_CAN, {})):
      ret.flags |= BmwFlags.STEPPER_SERVO_CAN.value

    ret.openpilotLongitudinalControl = True
    ret.radarUnavailable = True
    ret.pcmCruise = False # use OP speed tracking because we control speed using stock cruise speed setpoint or stock cruise is disabled

    ret.autoResumeSng = False
    if has_normal_cruise:   # Engine controls speed and reports cruise control status
      ret.flags |= BmwFlags.NORMAL_CRUISE_CONTROL.value # openpilot will inject cruise stalk +/- requests
    elif has_dynamic_cruise:   # either DSC or LDM reports cruise control status
      if not has_ldm:                                   # DSC itself applies brakes
        ret.flags |= BmwFlags.DYNAMIC_CRUISE_CONTROL.value # openpilot will inject cruise stalk +/- requests on F-CAN
      else: # LDM sends brake commands
        ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_ACC.value # openpilot will switch between OP and LDM
        ret.autoResumeSng = True #! hopefully
    else: # DSC/DME not sending cruise status and LDM not present - openpilot will be the only requester
      ret.flags |= BmwFlags.ACTIVE_CRUISE_CONTROL_NO_LDM.value
      ret.autoResumeSng = True #! hopefully

    # Runtime transmission detection - PT-CAN (bus 0) messages
    if 0xb8 in fingerprint.get(CanBus.PT_CAN, {}) or 0xb5 in fingerprint.get(CanBus.PT_CAN, {}):
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual

    # Detect all wheel drive BMW E90 XI - PT-CAN (bus 0) messages
    if 0xbc in fingerprint.get(CanBus.PT_CAN, {}): # XI has a transfer case
      ret.steerRatio = 18.5 # XI has slower steering rack

    if ret.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:  # DCC imperial has higher threshold
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS # if self.CS.is_metric else 20. * CV.MPH_TO_MS
    if ret.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      ret.minEnableSpeed = 30. * CV.KPH_TO_MS

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.bmw)]
    ret.safetyConfigs[0].safetyParam = 0

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.steerActuatorDelay = 0.4
    ret.steerLimitTimer = 0.4

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=2.0)

    ret.longitudinalActuatorDelay = 0.6  # Fixed delay for Phase 1 validation

    ret.centerToFront = ret.wheelbase * 0.44

    ret.startAccel = 0.0

    # has_servotronic = False
    # for fw in car_fw:  # todo check JBBF firmware for $216A
    #   if fw.ecu == "eps" and b"," in fw.fwVersion:
    #     has_servotronic = True

    return ret
