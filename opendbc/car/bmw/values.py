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


class CruiseSettings:
  CLUSTER_OFFSET = 2 # kph


class CurveSpeedParams:
  """Predictive curve speed control parameters for BMW"""
  LOOKAHEAD_TIME = 3.0  # seconds - look ahead for upcoming curves
  LAT_ACCEL_LIMIT = 2.0  # m/s² - from BMW_E90 override.toml MAX_LAT_ACCEL_MEASURED
  SPEED_MARGIN = 0.85    # 15% safety margin on calculated curve speed
  MIN_CURVATURE_THRESHOLD = 0.003  # Ignore very gentle curves (< 1/333m radius)
  MIN_SPEED_BUFFER = 5.0  # km/h - add to minEnableSpeed to avoid disengagement


class LongitudinalPersonalityParams:
  # BMW-specific longitudinal personality scale factors
  # See docs/BMW_Longitudinal_T_FOLLOW_Tuning.md for detailed configuration options

  # Speed breakpoints for T_FOLLOW scale factor (m/s)
  SPEED_SCALE_BP = [8.3, 13.9, 16.7, 19.4, 22.2, 27.8, 38.9]  # 30, 50, 60, 70, 80, 100, 140 kph

  # Dual-mode configuration:
  # - City (30-80 kph): Ultra-aggressive for stationary approach (2.4-2.7s @ 30 kph)
  # - Highway (80-140 kph): Safe following for 70 kph lead (2-4s @ 70 kph)
  SPEED_SCALE_VALUES = [0.42, 0.42, 1.29, 1.92, 1.0, 0.42, 0.42]


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
