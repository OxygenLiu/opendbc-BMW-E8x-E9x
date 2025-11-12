from opendbc.car import Bus, DT_CTRL, apply_dist_to_meas_limits
from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.bmwcan import SteeringModes, CruiseStalk
from opendbc.car.bmw.values import CarControllerParams, CanBus, BmwFlags, CruiseSettings
from opendbc.car.interfaces import CarControllerBase
from opendbc.can import CANPacker
from opendbc.car.common.conversions import Conversions as CV
import pickle
import numpy as np
from pathlib import Path


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
CRUISE_STALK_MINUS5_HOLD_TICK = 0.01    # 100Hz - emergency braking (maximum rate)
CRUISE_STALK_MINUS1_HOLD_TICK = 0.025   # 40Hz - moderate braking (held)
CRUISE_STALK_MINUS1_SINGLE_TICK = 0.05  # 20Hz - cruise adjustment (single presses)

# BMW DCC Specifications (ideal/theoretical - see DCC_Methodology_BMW_vs_Openpilot.md)
# These are BMW's published specs measured to 80-90% of setpoint (transient phase only)
# Plus1 held: 0.4 m/s², Plus5 held: 1.2 m/s²
# Minus1 held: -0.6 m/s², Minus5 held: -1.2 m/s²
#
# Measured Real-World Performance (full settling to 100% of setpoint)
# Route: 000000f1--7fed5392b6 (71 segments, Normal transmission mode)
# Plus1 held: 0.208 m/s² (52% of BMW spec - real-world conditions)
# Minus1 held: -0.445 m/s² (74% of BMW spec)
# Minus5 held: -0.784 m/s² (65% of BMW spec)
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

    # Load learned DCC lookup table (data-driven from 795 real braking sequences)
    # Maps: MPC accel → (frequency_hz, num_ticks) for optimal minus1 braking
    # See: ~/driving_data/docs/dcc_mapping/DCC_Learned_Table_Integration.md
    table_path = Path(__file__).parent / "dcc_learned_table.pkl"
    try:
      with open(table_path, 'rb') as f:
        self.dcc_table = pickle.load(f)
      print(f"✅ Loaded DCC learned table: {len(self.dcc_table['accel_grid'])} accel points")
      self.dcc_fallback_mode = False
    except FileNotFoundError:
      print(f"⚠️  DCC learned table not found at {table_path}, using fallback logic")
      self.dcc_table = None
      self.dcc_fallback_mode = True

    # DCC tick-based sequence state tracking
    self.dcc_ticks_remaining = 0  # Number of minus1 ticks left in current sequence
    self.dcc_frequency = 40.0  # Hz - frequency for current sequence
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

    # Acceleration command from planner - used for braking intensity in learned DCC table
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
          # *** BMW DCC Learned Lookup Table Strategy ***
          # Data-driven control based on 795 real minus1 braking sequences
          # Maps: MPC accel → (frequency_hz, num_ticks) that achieved closest actual deceleration
          #
          # v_error: velocity error relative to target (v_target - v_current)
          # accel: MPC acceleration command (m/s²) - used for table lookup
          #
          # BRAKING: Tick-limited sequences prevent setpoint runaway (max 8 km/h change)
          # ACCELERATION: Single plus1 commands accumulate to target speed
          #
          # LEARNED PERFORMANCE (from real data):
          # - 100Hz × 8 ticks: -0.736 desired → -1.099 actual m/s² (49 samples)
          # - 40Hz × 2 ticks:  -0.665 desired → -0.671 actual m/s² (3 samples)
          # - 20Hz × 1 tick:   -0.408 desired → -0.455 actual m/s² (184 samples)
          # - 10Hz × 1 tick:   -0.406 desired → -0.433 actual m/s² (365 samples)

          current_time = now_nanos / 1e9

          # ACCELERATION: v_error > 1.0 km/h and MPC requests acceleration
          # Strategy: Send plus1 commands at 5Hz with speed-dependent setpoint buffer
          # Phase 3: Speed-dependent buffer scaling to prevent high-speed overshoot
          #   - 70 km/h: 2.5 km/h buffer (proven sweet spot from Phase 2)
          #   - 140 km/h: 0.0 km/h buffer (no overshoot at very high speeds)
          #   - Linear interpolation between these points
          #   - CRITICAL: Smaller buffer at high speeds prevents overshoot
          v_error_setpoint = v_target - CS.out.cruiseState.speed  # Setpoint vs target error

          # Calculate speed-dependent buffer (in km/h)
          v_ego_kph = CS.out.vEgo * 3.6
          if v_ego_kph <= 70.0:
            buffer_kph = 2.5
          else:
            # Linear interpolation: 2.5 km/h @ 70 km/h → 0.0 km/h @ 140 km/h
            buffer_kph = 2.5 - ((v_ego_kph - 70.0) / 70.0) * 2.5

          if v_error > 1.0/3.6 and accel > 0 and v_error_setpoint > -buffer_kph/3.6:
            # Allow setpoint to exceed v_target by speed-dependent buffer (2.5→0.0 km/h)
            # Zero buffer at 140+ km/h prevents any overshoot at very high speeds
            # Calculate how many km/h to increase setpoint (rounded)
            v_error_kmh = v_error * 3.6
            setpoint_increase_needed = int(round(v_error_kmh))

            # Send plus1 commands at 5Hz (CRUISE_STALK_PLUS1_SINGLE_TICK = 0.2s)
            # Each plus1 command increases setpoint by 1 km/h
            time_since_last_accel = current_time - self.last_accel_time

            if time_since_last_accel >= CRUISE_STALK_PLUS1_SINGLE_TICK and setpoint_increase_needed > 0:
              cruise_cmd(CruiseStalk.plus1, CRUISE_STALK_PLUS1_SINGLE_TICK)
              self.last_accel_time = current_time
              self.dcc_ticks_remaining = 0  # Cancel any pending braking

          # BRAKING: v_error < -1.0 km/h (tightened from 1.5 for faster response)
          elif v_error < -1.0/3.6 and accel < 0 and CS.out.cruiseState.speed > self.min_cruise_setpoint:

            if self.dcc_table is not None:
              # Use learned lookup table
              # Start new braking sequence if previous one completed
              if self.dcc_ticks_remaining == 0:
                # Query learned table for optimal (frequency, ticks)
                idx = np.argmin(np.abs(self.dcc_table['accel_grid'] - accel))
                self.dcc_frequency = self.dcc_table['frequency_array'][idx]
                self.dcc_ticks_remaining = int(self.dcc_table['num_ticks_array'][idx])
                self.dcc_last_tick_time = current_time

              # Execute tick-based sequence
              if self.dcc_ticks_remaining > 0:
                tick_interval = 1.0 / self.dcc_frequency
                if current_time - self.dcc_last_tick_time >= tick_interval:
                  cruise_cmd(CruiseStalk.minus1, tick_interval)
                  self.dcc_ticks_remaining -= 1
                  self.dcc_last_tick_time = current_time
            else:
              # Fallback: simple threshold-based control if table not loaded
              if accel < -0.8:
                cruise_cmd(CruiseStalk.minus1, CRUISE_STALK_MINUS5_HOLD_TICK)
              elif accel < -0.3:
                cruise_cmd(CruiseStalk.minus1, CRUISE_STALK_MINUS1_HOLD_TICK)
              else:
                cruise_cmd(CruiseStalk.minus1, CRUISE_STALK_MINUS1_SINGLE_TICK)

          # DEADBAND: |v_error| <= 1.0 km/h - coast, no commands
          else:
            self.dcc_ticks_remaining = 0  # Reset braking sequence state

    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      steer_error = not CC.latActive and CC.enabled
      if not steer_error: # don't send steer CAN tx if steering is unavailable
        # *** apply steering torque ***
        if CC.enabled:
          new_steer = actuators.torque * CarControllerParams.STEER_MAX
          # explicitly clip torque before sending on CAN:
          # - don't use apply_meas_steer_torque_limits() due to integer rounding
          apply_torque = apply_dist_to_meas_limits(new_steer, self.apply_torque_last, CS.out.steeringTorqueEps,
                                             CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                             CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_torque))
        elif not CS.cruise_stalk_cancel and not CS.out.brakePressed and not CS.out.gasPressed and self.apply_torque_last != 0:
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.SoftOff, self.apply_torque_last))
          apply_torque = CS.out.steeringTorqueEps
        else:
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
    new_actuators.dccFallbackMode = self.dcc_fallback_mode

    self.frame += 1
    return new_actuators, can_sends
