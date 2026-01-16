import numpy as np
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.common.filter_simple import FirstOrderFilter
from opendbc.car.interfaces import CarStateBase
from opendbc.car.bmw.values import DBC, CanBus, BmwFlags, CruiseSettings
import cereal.messaging as messaging

ButtonType = structs.CarState.ButtonEvent.Type

# Resume button hold duration threshold (in frames at 100Hz = 10ms per frame)
# 0.5 seconds = 50 frames, but counter increments 49 times (reset to 0 on first press, then 49 increments)
# So threshold is 49 to detect 50 frames (0.5 seconds) of button press
# This makes personality cycling more practical (easier to hold for 0.5s than 1s)
RESUME_LONG_PRESS_FRAMES = 49


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint]['pt'])
    self.shifter_values = can_define.dv["TransmissionDataDisplay"]['ShiftLeverPosition']
    self.gas_kickdown = False

    self.cluster_min_speed = CruiseSettings.CLUSTER_OFFSET

    self.is_metric = None
    self.cruise_stalk_speed = 0
    self.cruise_stalk_resume = False
    self.cruise_stalk_cancel = False
    self.cruise_stalk_cancel_up = False
    self.cruise_stalk_cancel_dn = False
    self.cruise_stalk_counter = 0
    self.prev_cruise_stalk_speed = 0
    self.prev_cruise_stalk_resume = self.cruise_stalk_resume
    self.prev_cruise_stalk_cancel = self.cruise_stalk_cancel
    self.prev_cruise_enabled = False  # Track previous openpilot cruise state for resume button logic
    self.resume_button_hold_frames = 0  # Track how many frames resume button has been held (v4 duration-based logic)

    self.right_blinker_pressed = False
    self.left_blinker_pressed = False
    self.other_buttons = False
    self.prev_other_buttons = False
    self.prev_gas_pressed = False
    self.dtc_mode = False

    # Subscribe to radarState and liveDelay for velocity-difference-based T_FOLLOW scaling
    self.sm = messaging.SubMaster(['radarState', 'liveDelay'])

    # FirstOrderFilter for lateral acceleration (fc=0.2Hz, matches 50Hz CAN rate)
    # 0.2Hz cutoff provides good smoothing with minimal phase lag at driving frequencies
    self.lateral_accel_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * 0.2), 0.02)

  def update(self, can_parsers) -> structs.CarState:
    cp_PT = can_parsers[Bus.pt]
    cp_F = can_parsers[Bus.body]
    cp_aux = can_parsers[Bus.alt]

    ret = structs.CarState()

    # set these prev states at the beginning because they are used outside the update()
    self.prev_cruise_stalk_speed = self.cruise_stalk_speed
    self.prev_cruise_stalk_resume = self.cruise_stalk_resume
    self.prev_cruise_stalk_cancel = self.cruise_stalk_cancel

    ret.doorOpen = False # not any([cp.vl["SEATS_DOORS"]['DOOR_OPEN_FL'], cp.vl["SEATS_DOORS"]['DOOR_OPEN_FR']
    ret.seatbeltUnlatched = False # not cp.vl["SEATS_DOORS"]['SEATBELT_DRIVER_UNLATCHED']

    ret.brakePressed = cp_PT.vl["EngineAndBrake"]['BrakePressed'] != 0
    ret.parkingBrake = cp_PT.vl["Status_contact_handbrake"]["Handbrake_pulled_up"] != 0
    # on some cars, when cruise is engaged, half pressed pedal becomes "KickDownPressed", even without pressing kickdown end stop
    ret.gasPressed = cp_PT.vl['AccPedal']["AcceleratorPedalPressed"] != 0 or cp_PT.vl['AccPedal']["KickDownPressed"] != 0
    self.gas_kickdown = cp_PT.vl['AccPedal']["KickDownPressed"] != 0 #BMW has kickdown button at the bottom of the pedal

    # BMW uses centralized VehicleSpeed from Speed message (more reliable than individual wheel speeds)
    ret.vEgoRaw = cp_PT.vl['Speed']["VehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgo + CruiseSettings.CLUSTER_OFFSET * CV.KPH_TO_MS
    ret.standstill = not cp_PT.vl['Speed']["MovingForward"] and not cp_PT.vl['Speed']["MovingReverse"]
    ret.yawRate = cp_PT.vl['Speed']["YawRate"] * CV.DEG_TO_RAD
    ret.lateralAccel = self.lateral_accel_filter.update(cp_PT.vl["Speed"]['LatlAcc']) # BMW uses same convention as openpilot, left positive, right negative
    ret.steeringRateDeg = cp_PT.vl["SteeringWheelAngle"]['SteeringSpeed']
    can_gear = int(cp_PT.vl["TransmissionDataDisplay"]['ShiftLeverPosition'])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    # TurnSignals is pre-subscribed with nan frequency, so missing messages won't break can_valid
    # If missing, signals will be 0/default values, which is correct behavior
    blinker_on = cp_PT.vl["TurnSignals"]['TurnSignalActive'] != 0 and cp_PT.vl["TurnSignals"]['TurnSignalIdle'] == 0
    ret.leftBlinker = blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] != 0   # blinking
    ret.rightBlinker = blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] != 0   # blinking
    self.right_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['RightTurn'] != 0
    self.left_blinker_pressed = not blinker_on and cp_PT.vl["TurnSignals"]['LeftTurn'] != 0

    self.dtc_mode = cp_PT.vl['StatusDSC_KCAN']['DTC_on'] != 0 # drifty traction control ;)

    # other buttons help determine driver is paying attention in case the face is not visible
    self.other_buttons = \
      cp_PT.vl["SteeringButtons"]['Volume_DOWN'] != 0 or cp_PT.vl["SteeringButtons"]['Volume_UP'] != 0 or \
      cp_PT.vl["SteeringButtons"]['Previous_down'] != 0 or cp_PT.vl["SteeringButtons"]['Next_up'] != 0 or \
      cp_PT.vl["SteeringButtons"]['VoiceControl'] != 0 or \
      self.prev_gas_pressed and not ret.gasPressed # treat gas pedal tap as a button - button events indicate driver engagement - useful if face not visible

    # E-series doesn't have torque sensor
    # use Voice button or gas pedal to fake steeringPressed to confirm a lane change
    ret.steeringPressed = cp_PT.vl["SteeringButtons"]['VoiceControl'] != 0 or ret.gasPressed
    if ret.steeringPressed and ret.leftBlinker:
      ret.steeringTorque = 1
    elif ret.steeringPressed and ret.rightBlinker:
      ret.steeringTorque = -1
    else:
      ret.steeringTorque = 0

    ret.espDisabled = cp_PT.vl['StatusDSC_KCAN']['DSC_full_off'] != 0
    ret.cruiseState.available = not ret.espDisabled  #cruise not available when DSC fully off
    ret.cruiseState.nonAdaptive = False # bmw doesn't have a switch

    cruise_control_stal_msg = cp_PT.vl["CruiseControlStalk"]
    if self.CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      ret.steeringAngleDeg = cp_F.vl['SteeringWheelAngle_DSC']['SteeringPosition']  # slightly quicker on F-CAN TODO find the factor and put in DBC
      ret.cruiseState.speed = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["DynamicCruiseControlStatus"]['CruiseActive'] != 0
      # DCC implies that cruise control is done on F-CAN
      # If we are sending on F-can, we also need to read on F-can to differentiate our messages from car messages
      cruise_control_stal_msg = cp_F.vl["CruiseControlStalk"]
    elif self.CP.flags & BmwFlags.NORMAL_CRUISE_CONTROL:
      ret.steeringAngleDeg = cp_PT.vl['SteeringWheelAngle']['SteeringPosition']
      ret.cruiseState.speed = cp_PT.vl["CruiseControlStatus"]['CruiseControlSetpointSpeed'] * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS)
      ret.cruiseState.enabled = cp_PT.vl["CruiseControlStatus"]['CruiseControlActiveFlag'] != 0
    ret.cruiseState.speedCluster = ret.cruiseState.speed + CruiseSettings.CLUSTER_OFFSET * CV.KPH_TO_MS #For logging. Doesn't do anything with pcmCruise = False
    if cruise_control_stal_msg['plus1'] != 0:
      self.cruise_stalk_speed = 1
    elif cruise_control_stal_msg['minus1'] != 0:
      self.cruise_stalk_speed = -1
    elif cruise_control_stal_msg['plus5'] != 0:
      self.cruise_stalk_speed = 5
    elif cruise_control_stal_msg['minus5'] != 0:
      self.cruise_stalk_speed = -5
    else:
      self.cruise_stalk_speed = 0
    self.cruise_stalk_resume = cruise_control_stal_msg['resume'] != 0
    self.cruise_stalk_cancel = cruise_control_stal_msg['cancel'] != 0
    self.cruise_stalk_cancel_up = cruise_control_stal_msg['cancel_lever_up'] != 0
    self.cruise_stalk_counter = cruise_control_stal_msg['Counter_0x194']
    self.cruise_stalk_cancel_dn = self.cruise_stalk_cancel and not self.cruise_stalk_cancel_up

    # *** cruise control units one-time detection ***
    # when cruise is enabled the car sets cruiseState.speed = vEgo, so we can detect the ratio
    # with resume this wouldn't work, but op will not engage on first resume anyway
    if self.is_metric is None and ret.cruiseState.enabled and ret.vEgo > 5:
      # note, when is_metric is None, cruiseState.speed is already scaled by CV.MPH_TO_MS by default
      speed_ratio = ret.cruiseState.speed / ret.vEgo  # 1 if imperial, 1.6 if metric
      if 0.8 < speed_ratio < 1.2:
        self.is_metric = False
      elif 0.8 * CV.MPH_TO_KPH < speed_ratio < 1.2 * CV.MPH_TO_KPH:
        self.is_metric = True
        # Correct the speed to proper KPH scaling
        cruise_msg = "DynamicCruiseControlStatus" if (self.CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL) else "CruiseControlStatus"
        ret.cruiseState.speed = cp_PT.vl[cruise_msg]['CruiseControlSetpointSpeed'] * CV.KPH_TO_MS
      else:
        ret.accFaulted = True

    ret.genericToggle = self.dtc_mode

    # BMW vitals (temperatures only - battery voltage comes from peripheralState hardware sensor)
    # EngineData (0x1D0): TEMP_ENG (coolant) and TEMP_EOI (oil)
    ret.coolantTemp = cp_PT.vl["EngineData"]["TEMP_ENG"]
    ret.oilTemp = cp_PT.vl["EngineData"]["TEMP_EOI"]
    # Note: batteryVoltage is NOT set here - UI reads from peripheralState.voltage instead

    # BMW DCC velocity-difference-based T_FOLLOW scaling
    # BMW has no radar hardware - radarState comes from vision model (ModelV2)
    # Learned T_FOLLOW scales are published via liveDelay message and consumed by planner
    # No need to pass through carState - planner reads liveDelay.personalizedScales directly

    if self.CP.flags & BmwFlags.STEPPER_SERVO_CAN:
      ret.steeringTorqueEps = cp_aux.vl['STEERING_STATUS']['STEERING_TORQUE']
      ret.steeringAngleOffsetDeg = ret.steeringAngleDeg - cp_aux.vl['STEERING_STATUS']['STEERING_ANGLE']
      ret.steerFaultTemporary = (int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x20) != 0 # Comm error
      ret.steerFaultTemporary |= (int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x40) != 0 # motion task overrun
      ret.steerFaultTemporary |= (int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x80) != 0 # service task overrun
      ret.steerFaultTemporary |= (int(cp_aux.vl['STEERING_STATUS']['CONTROL_STATUS']) & 0x4) != 0 # SOFT_OFF lockout

    self.prev_gas_pressed = ret.gasPressed

    # Resume button duration-based logic (v6):
    # - Short press (<0.5s) when cruise NOT engaged → resumeCruise (engage openpilot)
    # - Long press (≥0.5s) when cruise ALREADY engaged → gapAdjustCruise (cycle personality)
    # This eliminates timing race conditions from edge detection

    resume_button_events = []

    # Track button hold duration
    if self.cruise_stalk_resume:
      # Button is pressed - increment hold counter
      if not self.prev_cruise_stalk_resume:
        # Just pressed - reset counter
        self.resume_button_hold_frames = 0
      else:
        # Still holding - increment counter
        self.resume_button_hold_frames += 1
    else:
      # Button not pressed - reset counter
      self.resume_button_hold_frames = 0

    # Generate button events based on press/release edges and hold duration
    if self.cruise_stalk_resume and not self.prev_cruise_stalk_resume:
      # Button just pressed - always generate resumeCruise event
      # BMW DCC stock cruise engages instantly, so we can't rely on prev_cruise_enabled
      resume_button_events.append(structs.CarState.ButtonEvent(
        pressed=True,
        type=ButtonType.resumeCruise
      ))

    elif not self.cruise_stalk_resume and self.prev_cruise_stalk_resume:
      # Button just released
      if self.resume_button_hold_frames >= RESUME_LONG_PRESS_FRAMES:
        # Held for ≥0.5 seconds → send gapAdjustCruise press+release (personality cycle)
        resume_button_events.append(structs.CarState.ButtonEvent(
          pressed=True,
          type=ButtonType.gapAdjustCruise
        ))
        resume_button_events.append(structs.CarState.ButtonEvent(
          pressed=False,
          type=ButtonType.gapAdjustCruise
        ))
      else:
        # Short press → send resumeCruise release event
        resume_button_events.append(structs.CarState.ButtonEvent(
          pressed=False,
          type=ButtonType.resumeCruise
        ))

    ret.buttonEvents = [
      *create_button_events(self.cruise_stalk_speed > 0, self.prev_cruise_stalk_speed > 0, {1: ButtonType.accelCruise}),
      *create_button_events(self.cruise_stalk_speed < 0, self.prev_cruise_stalk_speed < 0, {1: ButtonType.decelCruise}),
      *create_button_events(self.cruise_stalk_cancel, self.prev_cruise_stalk_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.other_buttons, self.prev_other_buttons, {1: ButtonType.altButton2}),
      *resume_button_events  # Use duration-based button events list
      ]

    self.cruise_state_enabled = ret.cruiseState.enabled
    self.prev_cruise_enabled = ret.cruiseState.enabled  # Save for next frame's resume button logic
    self.prev_other_buttons = self.other_buttons  # Save for next frame's altButton2 detection
    return ret

  # this is only to satisfy non pcmCruise test in test_panda_safety_carstate that requires button_enable
  #
  def update_button_enable(self, buttonEvents: list[structs.CarState.ButtonEvent]):
    if self.cruise_state_enabled and not self.out.cruiseState.enabled:
      return True
    return False

  @staticmethod
  def get_can_parsers(CP):
    # Only pre-subscribe problematic messages that are often completely missing
    # All other messages auto-subscribe dynamically when CarState.update() accesses them

    # Use float('nan') for ignore_alive=True on missing/sparse messages
    pt_messages = [
      ("TurnSignals", float('nan')),             # MISSING entirely - ignore liveness
      ("Status_contact_handbrake", float('nan')), # Very sparse (24 msgs) - ignore liveness
      ("EngineData", float('nan')),              # BMW vitals: coolant & oil temps - may be sparse
    ]

    fcan_messages = []
    # DCC mode reads CruiseControlStalk from F-CAN (carstate.py:117)
    # Must pre-subscribe to ensure message history available for resume button detection
    # Variable frequency: 5Hz idle, 20Hz during button presses - use float('nan') for immediate processing
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      fcan_messages.append(("CruiseControlStalk", float('nan')))  # Variable freq: process immediately

    servo_can_messages = []

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.PT_CAN),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.body], fcan_messages, CanBus.F_CAN),
      Bus.alt: CANParser('ocelot_controls', servo_can_messages, CanBus.SERVO_CAN),
    }
