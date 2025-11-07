from opendbc.car import Bus, DT_CTRL, apply_dist_to_meas_limits
from opendbc.car.bmw import bmwcan
from opendbc.car.bmw.bmwcan import SteeringModes, CruiseStalk
from opendbc.car.bmw.values import CarControllerParams, CanBus, BmwFlags
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
CRUISE_STALK_PLUS1_SINGLE_TICK = 0.05   # 20Hz - gentle acceleration (single presses)
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

    # Cruise setpoint error: direct difference for runaway protection
    # Positive = v_ego > setpoint (going too fast), Negative = v_ego < setpoint (going too slow)
    # Use raw DCC setpoint from CAN (CruiseControlSetpointSpeed), not cluster display value
    v_error_setpoint = v_current - CS.out.cruiseState.speed

    # Acceleration command from planner - used for intent confirmation in cruise control
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
          cruise_cmd(CruiseStalk.plus1, CRUISE_STALK_PLUS1_SINGLE_TICK)  # Support driver acceleration
        else:
          # *** BMW DCC 5-Mode Velocity Control Strategy ***
          # Frequency-optimized braking modes with MPC accel-based thresholds
          #
          # v_error: velocity error relative to target (v_target - v_current)
          # v_error_setpoint: velocity error relative to DCC setpoint (v_setpoint - v_current)
          # accel: MPC acceleration command (m/s²) - used for mode selection
          #
          # DUAL ERROR TRACKING:
          # - v_error: Used for MODE SELECTION (which command to send)
          # - v_error_setpoint: Used for EXIT CONDITIONS (when to stop sending)
          #
          # FREQUENCY-BASED BRAKING:
          # - Emergency: 100Hz (minus5 held) - maximum responsiveness
          # - Moderate: 40Hz (minus1 held) - balanced comfort/response
          # - Cruise: 20Hz (minus1 single) - gentle adjustments
          #
          # MEASURED REAL-WORLD PERFORMANCE (route 000000f1--7fed5392b6):
          # - Plus1 single (20Hz): Gentle acceleration (multiple presses accumulate)
          # - Minus1 held (40Hz): -0.445 m/s² (moderate deceleration)
          # - Minus5 held (100Hz): -0.784 m/s² (emergency braking)

          # MODE 1: Acceleration (Plus1 single @ 20Hz)
          # Entry: v_error > 1.5 km/h AND MPC wants acceleration (accel > 0)
          # Exit: v_error_setpoint > -5 km/h (setpoint within 5 km/h of vEgo - prevent overshoot)
          # Multiple single presses accumulate to reach target speed (comfortable, not aggressive)
          if v_error > 1.5/3.6 and v_error_setpoint > -5/3.6 and accel > 0:
            cruise_cmd(CruiseStalk.plus1, CRUISE_STALK_PLUS1_SINGLE_TICK)

          # MODE 2: Emergency Deceleration (Minus5 held @ 100Hz) ⚠️
          # Entry: v_error < -2 km/h AND strong MPC deceleration (accel < -0.8 m/s²)
          # Maximum frequency for fastest response in emergency situations
          elif v_error < -2/3.6 and accel < -0.8:
            cruise_cmd(CruiseStalk.minus5, CRUISE_STALK_MINUS5_HOLD_TICK)

          # MODE 3: Moderate Deceleration (Minus1 held @ 40Hz)
          # Entry: v_error < -2 km/h AND moderate MPC deceleration (accel < -0.3 m/s²)
          # Exit: v_error_setpoint < 15 km/h (prevent excessive setpoint drop)
          # Balanced frequency for comfortable yet responsive braking
          elif v_error < -2/3.6 and v_error_setpoint < 15.0/3.6 and accel < -0.3:
            cruise_cmd(CruiseStalk.minus1, CRUISE_STALK_MINUS1_HOLD_TICK)

          # MODE 4: Cruise Adjustment (Minus1 single @ 20Hz)
          # Entry: v_error < -1.5 km/h AND light MPC deceleration (accel < -0)
          # Exit: v_error_setpoint < 5 km/h (prevent excessive setpoint drop)
          # Gentle speed adjustments for following and cruise control
          elif v_error < -1.5/3.6 and v_error_setpoint < 5/3.6 and accel < 0:
            cruise_cmd(CruiseStalk.minus1, CRUISE_STALK_MINUS1_SINGLE_TICK)

          # MODE 5: Deadband (Coast)
          # ±1.5 km/h tolerance - no commands sent
          # Prevents oscillation, allows natural speed variations
          # else: pass

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

    self.frame += 1
    return new_actuators, can_sends
