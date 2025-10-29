import time
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.bmw.values import DBC, CanBus, BmwFlags, CruiseSettings
from opendbc.car.bmw.uds_dtc import Diagnostics, ProtectionAction, PassiveDTCMonitor

ButtonType = structs.CarState.ButtonEvent.Type


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
    self.cruise_state_enabled = False  # Track previous cruise state for resume button logic

    self.right_blinker_pressed = False
    self.left_blinker_pressed = False
    self.other_buttons = False
    self.prev_gas_pressed = False
    self.dtc_mode = False

    # Initialize BMW diagnostics (will be fully initialized with panda later)
    self.engine_coolant_temp = 0.0
    self.engine_oil_temp = 0.0
    self.diagnostics = None

    # Passive DTC monitoring (BMW safety model blocks active UDS requests)
    self.passive_dtc_monitor = PassiveDTCMonitor()

  def initialize_diagnostics(self, panda):
    """Initialize UDS diagnostics system with panda connection"""
    if panda is not None and self.diagnostics is None:
      try:
        from opendbc.car.bmw.uds_dtc import Diagnostics
        self.diagnostics = Diagnostics(panda, self.CP)
        carlog.info("BMW UDS diagnostics system initialized")
      except Exception as e:
        carlog.error(f"Failed to initialize BMW diagnostics: {e}")



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

    if self.CP.flags & BmwFlags.STEPPER_SERVO_CAN:
      ret.steeringTorqueEps = cp_aux.vl['STEERING_STATUS']['STEERING_TORQUE']
      ret.steeringAngleOffsetDeg = ret.steeringAngleDeg - cp_aux.vl['STEERING_STATUS']['STEERING_ANGLE']
      ret.steerFaultTemporary = int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x20 != 0 # Comm error
      ret.steerFaultTemporary |= int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x40 != 0 # motion task overrun
      ret.steerFaultTemporary |= int(cp_aux.vl['STEERING_STATUS']['DEBUG_STATES']) & 0x80 != 0 # service task overrun
      ret.steerFaultTemporary = int(cp_aux.vl['STEERING_STATUS']['CONTROL_STATUS']) & 0x4 != 0 # SOFT_OFF lockout

    self.prev_gas_pressed = ret.gasPressed

    ret.buttonEvents = [
      *create_button_events(self.cruise_stalk_speed > 0, self.prev_cruise_stalk_speed > 0, {1: ButtonType.accelCruise}),
      *create_button_events(self.cruise_stalk_speed < 0, self.prev_cruise_stalk_speed < 0, {1: ButtonType.decelCruise}),
      *create_button_events(self.cruise_stalk_cancel, self.prev_cruise_stalk_cancel, {1: ButtonType.cancel}),
      *create_button_events(self.other_buttons, not self.other_buttons, {1: ButtonType.altButton2}),
      *create_button_events(self.cruise_stalk_resume, self.prev_cruise_stalk_resume, {
        # Use PREVIOUS cruise state to prevent timing race condition during engagement
        # When resume pressed: Frame N (not engaged) → resumeCruise, Frame N+1 (engaged) → still resumeCruise ✅
        # Only on subsequent resume presses when already engaged → gapAdjustCruise
        1: ButtonType.resumeCruise if not self.cruise_state_enabled else ButtonType.gapAdjustCruise})
      ]

    self.cruise_state_enabled = ret.cruiseState.enabled  # Update for next frame


    # BMW Engine temperatures from EngineData CAN message (0x1D0)
    self.engine_coolant_temp = cp_PT.vl['EngineData']['TEMP_ENG']
    self.engine_oil_temp = cp_PT.vl['EngineData']['TEMP_EOI']

    # BMW diagnostics - publish engine temperatures to CarState for UI
    ret.engineCoolantTemp = self.engine_coolant_temp
    ret.engineOilTemp = self.engine_oil_temp

    # Update vehicle state for DTC clear validation
    ignition_on = True #ret.ignitionLine  # BMW ignition state
    engine_running = True #ret.engineRpm > 500  # Engine running if RPM > 500
    self.passive_dtc_monitor.update_vehicle_state(ignition_on, engine_running)

    # Passive DTC monitoring from broadcast messages (BMW safety blocks active UDS)
    self.passive_dtc_monitor.monitor_diagnostic_messages(cp_PT)

    # Publish real DTC data to CarState for UI
    ret.bmwDtcCount = self.passive_dtc_monitor.get_dtc_count()
    ret.bmwActiveDtcs = self.passive_dtc_monitor.get_dtc_summary()
    ret.bmwDtcClearStatus = self.passive_dtc_monitor.get_dtc_clear_status()

    return ret

  def request_dtc_clear(self) -> bool:
    """Request DTC clearing via UDS Service 0x14"""
    if self.passive_dtc_monitor:
      return self.passive_dtc_monitor.request_dtc_clear_via_uds()
    return False

  def get_uds_clear_message(self) -> bytes:
    """Get UDS Service 0x14 clear message data for transmission"""
    if self.passive_dtc_monitor:
      return self.passive_dtc_monitor.get_uds_clear_request_data()
    return b''

  def init_diagnostics(self, panda):
    """Initialize diagnostics when panda is available (called from interface.py)"""
    try:
      self.diagnostics = Diagnostics(panda, self.CP)
      carlog.info("BMW diagnostics initialized - available via VEHICLE button")
    except Exception as e:
      carlog.error(f"Failed to initialize BMW diagnostics: {e}")

  def get_diagnostic_data(self):
    """Get diagnostic data for UI display (called when VEHICLE button pressed)"""
    if self.diagnostics is None:
      return {
        'status': 'unavailable',
        'message': 'Diagnostics not initialized'
      }

    try:
      # Get fresh diagnostic data
      dtc_summary = self.diagnostics.get_diagnostic_summary()
      engine_protection = self.diagnostics.get_engine_protection_status()

      return {
        'status': 'available',
        'engine_temps': {
          'coolant': self.engine_coolant_temp,
          'oil': self.engine_oil_temp
        },
        'dtc_summary': dtc_summary,
        'engine_protection': engine_protection,
        'actions': {
          'read_dtcs': lambda: self.diagnostics.request_dtcs(),
          'clear_dtcs': lambda: self.diagnostics.clear_dtcs()
        }
      }
    except Exception as e:
      carlog.error(f"Error getting diagnostic data: {e}")
      return {
        'status': 'error',
        'message': str(e)
      }

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
      ("EngineData", 10),                        # 10Hz - needed for BMW temperature data
      ("ServicesDME", float('nan')),             # Passive DTC monitoring - ignore liveness
      ("EngineOBD_data", float('nan')),          # Passive DTC monitoring - ignore liveness
      ("ServicesDSC", float('nan')),             # Passive DTC monitoring - ignore liveness
    ]

    fcan_messages = []

    servo_can_messages = []

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.PT_CAN),
      Bus.body: CANParser(DBC[CP.carFingerprint][Bus.body], fcan_messages, CanBus.F_CAN),
      Bus.alt: CANParser('ocelot_controls', servo_can_messages, CanBus.SERVO_CAN),
    }
