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
# Stock cruise stalk CAN frequency when stalk is not pressed is 5Hz
CRUISE_STALK_IDLE_TICK_STOCK = 0.2
# Stock cruise stalk CAN frequency when stalk is pressed is 20Hz
CRUISE_STALK_HOLD_TICK_STOCK = 0.05

# We will send also at 5Hz in between stock messages to emulate single presses
CRUISE_STALK_SINGLE_TICK = CRUISE_STALK_IDLE_TICK_STOCK
# Emulate held stalk, 100Hz makes stock messages be ignored
CRUISE_STALK_HOLD_TICK = 0.01

# Reference value
#ACCEL_HOLD_MEDIUM = 0.4
#DECEL_HOLD_MEDIUM = -0.6
#ACCEL_HOLD_STRONG = 1.2
#DECEL_HOLD_STRONG = -1.2


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

    # ModelV2 velocity error breakpoints for DCC command mapping (m/s)
    self.v_error_bp = [4.5, 3.0, 1.5, 0.5, -0.5, -1.5, -3.0, -4.5]

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
    # - For BMW: action_t = 0.15s + 0.1s = 0.25s (accounts for cruise command processing delay)
    # - MPC refines ModelV2 with physics/comfort/safety constraints (A_CHANGE_COST=200, J_EGO_COST=5)
    # This is feedforward control - commands what's needed when actuator actually responds!
    v_target = actuators.speed if actuators.speed > 0 else CS.out.vEgoCluster  # Delay-compensated target

    # CRITICAL: Use vision speed for current velocity
    # Three-speed-sources architecture for BMW:
    # 1. GPS: Accurate but intermittent (buildings/tunnels)
    # 2. CAN (vEgo): Reliable but conservative, affected by tire pressure/slip
    # 3. Vision: Accurate and condition-independent (ModelV2 vision-estimated)
    v_current = actuators.visionSpeed  # ModelV2 vision-estimated current velocity (best for control)

    # Safety validation: Vision speed must agree with CAN speed within ±5 km/h (±1.39 m/s)
    # Note: CS.out.vEgo is Kalman-filtered CAN speed (not vision), CS.out.vEgoCluster is display speed
    vision_can_diff = abs(v_current - CS.out.vEgo)  # Compare vision vs Kalman-filtered CAN
    if vision_can_diff > 1.39:  # Safety threshold exceeded
        # Fallback to Kalman-filtered CAN speed if vision-CAN disagreement too large
        v_current = CS.out.vEgo

    v_error = v_target - v_current  # Velocity error using delay-compensated target

    # detect incoming CruiseControlStalk message by observing counter change (message arrives at only 5Hz when nothing pressed)
    if CS.cruise_stalk_counter != self.rx_cruise_stalk_counter_last:
      self.tx_cruise_stalk_counter_last = CS.cruise_stalk_counter
      # stock message was sent some time in between control samples:
      self.last_cruise_rx_timestamp = now_nanos
    self.rx_cruise_stalk_counter_last = CS.cruise_stalk_counter

    # *** send cruise control stalk message at different rates and manage counters ***
    def cruise_cmd(cmd, hold=False):
      time_since_cruise_sent = (now_nanos - self.last_cruise_tx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      time_since_cruise_received = (now_nanos - self.last_cruise_rx_timestamp) / 1e9 + DT_CTRL / 10 # add half task sample time to account for latency
      # send single cmd with an effective rate slower than held stalk rate
      if not hold:
        send = time_since_cruise_sent > CRUISE_STALK_SINGLE_TICK \
          and time_since_cruise_received > CRUISE_STALK_HOLD_TICK_STOCK/2 - DT_CTRL \
          and time_since_cruise_received < CRUISE_STALK_IDLE_TICK_STOCK/2 + DT_CTRL
      else:
        # use faster rate to emulate held stalk. Time first message such that subsequent one will nullify stock message:
        send = hold and time_since_cruise_sent > CRUISE_STALK_HOLD_TICK
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

    if not cruise_stalk_human_pressing and CS.out.cruiseState.enabled:
      if self.cruise_cancel:
        cruise_cmd(CruiseStalk.cancel)
        print("cancel")
      elif CC.enabled:
        # Handle driver gas override first
        if CS.out.gasPressed:
          cruise_cmd(CruiseStalk.plus1)                                   # Support driver acceleration
        else:
          if v_error > self.v_error_bp[0]:                                # > 4.5 m/s (16.2 km/h)
            cruise_cmd(CruiseStalk.plus5, hold=True)                      # Strong acceleration hold
          elif v_error > self.v_error_bp[1]:                             # > 3.0 m/s (10.8 km/h)
            cruise_cmd(CruiseStalk.plus1, hold=True)                      # Medium acceleration hold
          elif v_error > self.v_error_bp[2]:                             # > 1.5 m/s (5.4 km/h)
            cruise_cmd(CruiseStalk.plus5)                                 # Light acceleration burst
          elif v_error > self.v_error_bp[3]:                             # > 0.5 m/s (1.8 km/h)
            cruise_cmd(CruiseStalk.plus1)                                 # Fine speed adjustment
          elif v_error < self.v_error_bp[7]:                             # < -4.5 m/s (-16.2 km/h)
            cruise_cmd(CruiseStalk.minus5, hold=True)                     # Strong deceleration hold
          elif v_error < self.v_error_bp[6]:                             # < -3.0 m/s (-10.8 km/h)
            cruise_cmd(CruiseStalk.minus1, hold=True)                     # Medium deceleration hold
          elif v_error < self.v_error_bp[5]:                             # < -1.5 m/s (-5.4 km/h)
            cruise_cmd(CruiseStalk.minus5)                                # Light deceleration burst
          elif v_error < self.v_error_bp[4]:                             # < -0.5 m/s (-1.8 km/h)
            cruise_cmd(CruiseStalk.minus1)                                # Fine speed reduction
          # else: velocity error within deadband [-0.5, 0.5] m/s - no command needed

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
    new_actuators.accel = v_error  # Velocity error for logging/debugging

    self.frame += 1
    return new_actuators, can_sends
