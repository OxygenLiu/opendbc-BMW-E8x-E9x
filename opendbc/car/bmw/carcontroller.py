from opendbc.car import Bus, DT_CTRL, apply_dist_to_meas_limits
from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.bmwcan import SteeringModes, CruiseStalk
from opendbc.car.bmw.values import CarControllerParams, CanBus, BmwFlags, CruiseSettings
from opendbc.car.interfaces import CarControllerBase
from opendbc.can import CANPacker
from opendbc.car.common.conversions import Conversions as CV


# DO NOT CHANGE: Cruise control step size
# Cruise single click jump - always 1 - interpreted as km or miles depending on DSC or DME set units
CC_STEP = 1

# BMW Stock DCC CAN Frequencies (measured from route 000000f1--7fed5392b6)
# See: ~/driving_data/docs/dcc_calibration_mode/BMW_DCC_Data.md
CRUISE_STALK_IDLE_TICK_STOCK = 0.2    # 5Hz - stock idle (no stalk pressed)
CRUISE_STALK_SINGLE_TICK_STOCK = 0.05 # 20Hz - stock single press
CRUISE_STALK_HOLD_TICK_STOCK = 0.025  # 40Hz - stock held stalk

# Openpilot DCC Emulation - Frequency-based command rates
# Different modes use different frequencies for comfort and responsiveness
CRUISE_STALK_PLUS1_SINGLE_TICK = 0.2    # 5Hz - improved acceleration response (was 2Hz)
CRUISE_STALK_PLUS1_HOLD_TICK = 0.025    # 40Hz - rapid acceleration for large deficits (held)
CRUISE_STALK_MINUS1_HOLD_TICK = 0.025   # 40Hz - braking (minus1 held)

# BMW DCC Specifications (ideal/theoretical - see DCC_Methodology_BMW_vs_Openpilot.md)
# These are BMW's published specs measured to 80-90% of setpoint (transient phase only)
# Plus1 held: 0.4 m/s², Minus1 held: -0.6 m/s²
#
# Measured Real-World Performance (full settling to 100% of setpoint)
# Route: 000000f1--7fed5392b6 (71 segments, Normal transmission mode)
# Plus1 held: 0.208 m/s² (52% of BMW spec - real-world conditions)
# Minus1 held: -0.445 m/s² (74% of BMW spec)
# Our measurements include complete settling phase, more suitable for velocity control


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags
    self.min_cruise_speed = CP.minEnableSpeed
    # Minimum cruise setpoint to prevent disengagement (30 km/h + 5 km/h buffer = 35 km/h)
    self.min_cruise_setpoint = self.min_cruise_speed + CruiseSettings.MIN_SPEED_BUFFER * CV.KPH_TO_MS
    self.cruise_units = None

    self.cruise_cancel = False  # local cruise control cancel
    self.cruise_enabled_prev = False
    # redundant safety check with the board
    self.apply_torque_last = 0
    self.last_cruise_rx_timestamp = 0 # stock cruise buttons
    self.last_cruise_tx_timestamp = 0 # openpilot commands
    self.tx_cruise_stalk_counter_last = 0
    self.rx_cruise_stalk_counter_last = -1

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN

    self.packer = CANPacker(dbc_name[Bus.pt])

    # DCC tick-based sequence state tracking
    self.dcc_ticks_remaining = 0  # Number of minus1 ticks left in current sequence
    self.dcc_last_tick_time = 0  # Timestamp of last tick sent

    # DCC acceleration control - direct setpoint adjustment
    self.last_accel_time = 0  # Timestamp of last plus1 command

  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    self.cruise_units = (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # *** Delay-Compensated MPC Velocity Control ***
    # v_target is delay-compensated velocity from get_accel_from_plan():
    # - Extracted from MPC trajectory at action_t = longitudinalActuatorDelay + DT_MDL
    # - For BMW: action_t = 0.6s + 0.1s = 0.7s (accounts for cruise command processing delay)
    # - MPC refines ModelV2 with physics/comfort/safety constraints (A_CHANGE_COST=200, J_EGO_COST=5)
    # This is feedforward control - commands what's needed when actuator actually responds!
    v_target = actuators.speed if actuators.speed > 0 else CS.out.vEgoCluster  # Delay-compensated target

    # Use Kalman-filtered CAN speed (vEgo) instead of noisy visionSpeed
    # visionSpeed from modelV2 is more noisy than Kalman-filtered vEgo
    v_current = CS.out.vEgo  # Kalman-filtered CAN speed (stable and reliable)

    v_error = v_target - v_current  # Velocity error using delay-compensated target

    accel = actuators.accel

    # detect incoming CruiseControlStalk message by observing counter change (message arrives at only 5Hz when nothing pressed)
    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      self.tx_cruise_stalk_counter_last = CS.cruise_stalk_counter
      # stock message was sent some time in between control samples:
      self.last_cruise_rx_timestamp = now_nanos
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter

    # *** send cruise control stalk message at different rates and manage counters ***
    def cruise_cmd(cmd, tick_interval):
      time_since_cruise_sent = (now_nanos - self.last_cruise_tx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      time_since_cruise_received = (now_nanos - self.last_cruise_rx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      # Check if enough time has passed to send the next command
      send = time_since_cruise_sent > tick_interval \
        and time_since_cruise_received > CRUISE_STALK_HOLD_TICK_STOCK/2 - DT_CTRL \
        and time_since_cruise_received < CRUISE_STALK_IDLE_TICK_STOCK/2 + DT_CTRL
      if send:
        tx_cruise_stalk_counter = self.tx_cruise_stalk_counter_last + 1
        # avoid counter clash with a potential upcoming message from stock cruise
        if tx_cruise_stalk_counter == CS.cruise_stalk_counter + 1:
          # avoid clashing with upcoming stock message
          # sometimes upcoming stock message is overshadowed by us, so also avoid clashing with one after that
          tx_cruise_stalk_counter = tx_cruise_stalk_counter + 2
        tx_cruise_stalk_counter = tx_cruise_stalk_counter % 0xF
        can_sends.append(bmwcan.create_accel_command(self.packer, cmd, self.cruise_bus, tx_cruise_stalk_counter))
        self.tx_cruise_stalk_counter_last = tx_cruise_stalk_counter
        self.last_cruise_tx_timestamp = now_nanos

    # *** cruise control cancel signal ***
    # CC.cruiseControl.cancel can't be used because it is always false because pcmCruise = False because we need OP speed tracker
    # CC.enabled appears after cruiseState.enabled, so we need to check rising edge to prevent instantaneous cancel after cruise is enabled
    # This is because CC.enabled comes from controld and CS.out.cruiseState.enabled is from card threads
    if not CC.enabled and self.cruise_enabled_prev:
      self.cruise_cancel = True
    # if we need to go below cruise speed, request cancel and coast while steering turns off softly
    if (CS.out.cruiseState.speedCluster - self.min_cruise_speed) < 0.1 \
      and CS.out.vEgoCluster - self.min_cruise_speed < 0.4:
      self.cruise_cancel = True
    # keep requesting cancel until the cruise is disabled
    if not CS.out.cruiseState.enabled:
      self.cruise_cancel = False

    cruise_stalk_human_pressing = CS.cruise_stalk_resume or CS.cruise_stalk_cancel or CS.cruise_stalk_speed != 0

    # DCC Calibration Mode: When enabled, openpilot won't engage (NO_ENTRY event in selfdrived)
    # This allows manual DCC control while logging CAN data
    # The check here is defensive programming - CC.enabled should already be False
    if not cruise_stalk_human_pressing and CS.out.cruiseState.enabled:
      if self.cruise_cancel:
        cruise_cmd(CruiseStalk.cancel, CRUISE_STALK_SINGLE_TICK_STOCK)  # Use stock single press rate for cancel
        print("cancel")
      elif CC.enabled:
        # Handle driver gas override first
        if CS.out.gasPressed:
          cruise_cmd(CruiseStalk.plus1, CRUISE_STALK_PLUS1_SINGLE_TICK)
          self.dcc_ticks_remaining = 0  # Cancel any pending braking sequence
        else:
          # *** BMW DCC Setpoint Control Strategy ***
          # v_error: velocity error (v_target - v_current)
          #
          # ACCELERATION: plus1 commands raise setpoint (hold rate for large deficits)
          # BRAKING: minus1 commands lower setpoint (hold rate for large deficits)
          # DEADBAND: |v_error| <= 1 km/h → coast, no commands

          current_time = now_nanos / 1e9

          # PRE-EMPTIVE SETPOINT MAINTENANCE: When below target speed,
          # immediately cancel any in-progress braking sequence. This eliminates
          # the lag at braking→acceleration transitions where the setpoint stays
          # low while the lead vehicle is already pulling away.
          if v_error > 0 and self.dcc_ticks_remaining > 0:
            self.dcc_ticks_remaining = 0

          # ACCELERATION: v_error > 1.0 km/h and MPC requests acceleration
          # Strategy: Send plus1 commands with speed-dependent setpoint buffer
          #   - plus1 at 40Hz hold for large deficits (>= 5 km/h) for rapid recovery
          #   - plus1 at 5Hz single for fine-grained catch-up
          #   - Speed-dependent buffer allows setpoint overshoot for aggressive acceleration
          v_error_setpoint = v_target - CS.out.cruiseState.speed  # Setpoint vs target error

          # Calculate speed-dependent buffer (in km/h)
          v_ego_kph = CS.out.vEgo * 3.6
          if v_ego_kph <= 120.0:
            # Linear interpolation: 6.0 km/h @ 0 km/h → 0.0 km/h @ 120 km/h
            buffer_kph = (1.0 - v_ego_kph / 120.0) * 6.0
          else:
            # above 120km/h, 0.0 km/h
            buffer_kph = 0.0

          if v_error > 1.0/3.6 and accel > 0 and v_error_setpoint > -buffer_kph/3.6:
            v_error_kmh = v_error * 3.6
            setpoint_increase_needed = int(round(v_error_kmh))

            time_since_last_accel = current_time - self.last_accel_time

            if time_since_last_accel >= CRUISE_STALK_PLUS1_SINGLE_TICK and setpoint_increase_needed > 0:
              # Use hold rate (40Hz) for large deficits to rapidly recover setpoint
              if setpoint_increase_needed >= 3:
                cruise_cmd(CruiseStalk.plus1, CRUISE_STALK_PLUS1_HOLD_TICK)
              else:
                cruise_cmd(CruiseStalk.plus1, CRUISE_STALK_PLUS1_SINGLE_TICK)
              self.last_accel_time = current_time
              self.dcc_ticks_remaining = 0  # Cancel any pending braking

          # BRAKING: v_error < -1.0 km/h
          # Strategy: minus1 held at 40Hz with v_error-based tick count
          #   - Ticks = setpoint km/h to drop (1 tick = 1 km/h)
          #   - Tick system prevents over-braking regardless of rate
          elif v_error < -1.0/3.6 and accel < 0 and CS.out.cruiseState.speed > self.min_cruise_setpoint:
            if self.dcc_ticks_remaining == 0:
              # Clamp ticks to not drop setpoint below min_cruise_setpoint floor
              # This prevents overshoot due to 5Hz CAN feedback delay on cruiseState.speed
              v_error_ticks = int(round(-v_error * 3.6))
              max_ticks = max(0, int((CS.out.cruiseState.speed - self.min_cruise_setpoint) * 3.6))
              self.dcc_ticks_remaining = min(v_error_ticks, max_ticks)
              self.dcc_last_tick_time = current_time

            if self.dcc_ticks_remaining > 0:
              if current_time - self.dcc_last_tick_time >= CRUISE_STALK_MINUS1_HOLD_TICK:
                cruise_cmd(CruiseStalk.minus1, CRUISE_STALK_MINUS1_HOLD_TICK)
                self.dcc_ticks_remaining -= 1
                self.dcc_last_tick_time = current_time

          # DEADBAND: |v_error| <= 1.0 km/h - coast, no commands
          else:
            self.dcc_ticks_remaining = 0  # Reset braking sequence state

    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      # *** apply steering torque ***
      # CRITICAL: Always send 0x22E STEERING_COMMAND at 100Hz to prevent COMM errors
      # Stepper servo firmware expects continuous communication - gaps > 50ms trigger SOFT_OFF lockout
      # Previous bug: steer_error guard caused 0x22E to be skipped when CC.latActive=False
      # This created 1.4s gaps during lateral control disable, triggering servo COMM error protection

      if CC.enabled and CC.latActive:
        # Active steering control
        new_steer = actuators.torque * CarControllerParams.STEER_MAX
        # explicitly clip torque before sending on CAN:
        # - don't use apply_meas_steer_torque_limits() due to integer rounding
        apply_torque = apply_dist_to_meas_limits(new_steer, self.apply_torque_last, CS.out.steeringTorqueEps,
                                           CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                           CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_torque))
      elif not CS.cruise_stalk_cancel and not CS.out.brakePressed and not CS.out.gasPressed and self.apply_torque_last != 0:
        # Graceful ramp-down when disengaging
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.SoftOff, self.apply_torque_last))
        apply_torque = CS.out.steeringTorqueEps
      else:
        # Disabled - send Off mode to maintain 100Hz communication
        apply_torque = 0
        can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.Off))
      self.apply_torque_last = apply_torque

    # debug
    if CC.enabled and (self.frame % 10) == 0: #slow print
      frame_number = self.frame
      print(f"Steering req: {actuators.torque}, Speed: {CS.out.vEgoCluster}, Frame number: {frame_number}")

    self.cruise_enabled_prev = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    new_actuators.speed = v_target

    self.frame += 1
    return new_actuators, can_sends
