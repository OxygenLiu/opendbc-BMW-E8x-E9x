from dataclasses import dataclass, field
from enum import Enum, IntFlag
from opendbc.car import Bus, Platforms, CarSpecs, PlatformConfig, DbcDict, STD_CARGO_KG
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.fw_query_definitions import LiveFwVersions, OfflineFwVersions, FwQueryConfig

# Steer torque limits


class CarControllerParams: #controls running @ 100hz
  STEER_STEP = 1 # 100Hz
  STEER_MAX = 12  # Nm
  STEER_DELTA_UP = 0.1       # Nm/10ms
  STEER_DELTA_DOWN = 1.0     # Nm/10ms
  STEER_ERROR_MAX = 999     # max delta between torque cmd and torque motor

  # STEER_BACKLASH = 1 #deg
  def __init__(self, CP):
    pass


class BmwFlags(IntFlag):
  # Detected Flags
  STEPPER_SERVO_CAN = 2 ** 0
  NORMAL_CRUISE_CONTROL = 2 ** 1          # CC  $540
  DYNAMIC_CRUISE_CONTROL = 2 ** 2         # DCC $544
  ACTIVE_CRUISE_CONTROL = 2 ** 3          # ACC $541 - LDM and ACC sensor - #! not supported
  ACTIVE_CRUISE_CONTROL_NO_ACC = 2 ** 4   # no ACC module - DSC, DME, KOMBI coded to $541, LDM coded to $544
  ACTIVE_CRUISE_CONTROL_NO_LDM = 2 ** 5   # no LDM/ACC - DSC, DME, KOMBI coded to $541
  SERVOTRONIC = 2 ** 6                    # ServoTonic $216A - TODO: needs firmware query

  # User-Configurable Flags (set via params)
  DCC_CALIBRATION_MODE = 2 ** 7           # Disable OP engagement, log DCC performance for tuning


class CruiseSettings:
  CLUSTER_OFFSET = 2 # kph
  MIN_SPEED_BUFFER = 1.0  # km/h - add to minEnableSpeed to avoid disengagement

class CurveSpeedParams:
  """Predictive curve speed control parameters for BMW"""
  LOOKAHEAD_TIME = 3.0  # seconds - look ahead for upcoming curves
  LAT_ACCEL_LIMIT = 2.5  # m/s² - EU guideline ISO 11270 (changed from 2.0)
  SPEED_MARGIN = 0.85    # 15% safety margin on calculated curve speed
  MIN_CURVATURE_THRESHOLD = 0.003  # Ignore very gentle curves (< 1/333m radius)
  MIN_SPEED_BUFFER = 5.0  # km/h - add to minEnableSpeed to avoid disengagement


class LongitudinalPersonalityParams:
  # BMW DCC velocity-difference-based T_FOLLOW scaling
  # Adjusts following distance based on closing speed to lead vehicle
  # Extended to cover both deceleration (positive vrel) and acceleration (negative vrel) phases

  # Velocity difference breakpoints (v_ego - v_lead) in kph
  # Negative vrel: catching up to faster lead (acceleration phase)
  # Positive vrel: approaching slower lead (deceleration phase)
  VREL_BP_KPH = [-40, -30, -20, -10, 0, 10, 20, 30, 40]  # kph

  # Convert to m/s for runtime interpolation
  VREL_BP = [v / 3.6 for v in VREL_BP_KPH]  # m/s

  # T_FOLLOW scale factors (multipliers applied to base T_FOLLOW)
  # Initial values: 1.0 (no scaling - use default t_follow)
  # Learned values will be updated based on driver behavior in both phases
  T_FOLLOW_SCALE_FACTORS = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


class CanBus:
  PT_CAN = 0
  SERVO_CAN = 1  # required for steering (STEPPER_SERVO can be on this bus)
  F_CAN = 1  # required for DYNAMIC_CRUISE_CONTROL or optional for logging
  AUX_CAN = 2  # alternative bus for STEPPER_SERVO messages (matches BMW_AUX_CAN in bmw.h)
  K_CAN = 2  # not used - only logging


class Footnote(Enum):
  StepperServoCAN = CarFootnote(
    "Requires StepperServoCAN",
    Column.FSR_STEERING)
  DCC = CarFootnote(
    "Minimum speed with CC or DCC is 30 kph",
    Column.FSR_LONGITUDINAL)
  CC = CarFootnote(
    "Normal cruise control should work but was not tested in a while. Code in DCC instead or provide a fix",
    Column.PACKAGE)
  ACC = CarFootnote(
    "ACC is required. Also LDM module to take over when OP is off.",
    Column.AUTO_RESUME)
  DIY = CarFootnote(
    "For CC and DCC only a diy USB-C and a resistor is required or a harness box DIY connector",
    Column.HARDWARE)


@dataclass
class BmwCarDocs(CarDocs):
  package: str = "Cruise Control - VO540, VO544, VO541"
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.StepperServoCAN, Footnote.DCC, Footnote.CC, Footnote.ACC, Footnote.DIY])

  def init_make(self, CP: CarParams):
      self.car_parts = CarParts.common([CarHarness.custom])


@dataclass
class BmwPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'bmw_e9x_e8x',
    Bus.chassis: 'bmw_e9x_e8x',
    Bus.body: 'bmw_e9x_e8x',
    })


class CAR(Platforms):
  BMW_E82 = BmwPlatformConfig(
    [BmwCarDocs("BMW E82 2004-13")],
    CarSpecs(mass=3145. * CV.LB_TO_KG + STD_CARGO_KG, wheelbase=2.66, steerRatio=16.00, tireStiffnessFactor=0.8)
  )
  BMW_E90 = BmwPlatformConfig(
    [BmwCarDocs("BMW E90 2005-11")],
    CarSpecs(mass=3300. * CV.LB_TO_KG + STD_CARGO_KG, wheelbase=2.76, steerRatio=19.0, tireStiffnessFactor=0.8)
  )


DBC = CAR.create_dbc_map()


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> set[str]:
  """BMW VIN-based model detection to distinguish E82 from E90"""
  if not vin or len(vin) != 17:
    return set()

  # BMW VIN structure: positions 4-6 contain model code
  model_code = vin[3:6]

  # BMW model code mapping for E8x/E9x series
  vin_to_model = {
    # E82 1-Series Coupe/Convertible
    'UF1': 'BMW_E82', 'UF2': 'BMW_E82', 'UH1': 'BMW_E82',
    # E90/E91/E92/E93 3-Series (all use E90 fingerprint)
    'PH1': 'BMW_E90', 'PH2': 'BMW_E90', 'PK1': 'BMW_E90',
    'PK2': 'BMW_E90', 'PM1': 'BMW_E90', 'PM2': 'BMW_E90', 'PN1': 'BMW_E90',
  }

  detected_model = vin_to_model.get(model_code)

  # Only return models that exist in offline_fw_versions (if provided)
  if detected_model and offline_fw_versions:
    if detected_model in offline_fw_versions:
      return {detected_model}
    else:
      return set()
  elif detected_model:
    return {detected_model}
  else:
    return set()


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],  # No firmware queries needed for VIN-based detection
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
  extra_ecus=[],
  non_essential_ecus={},
)
