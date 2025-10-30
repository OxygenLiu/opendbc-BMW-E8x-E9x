from parameterized import parameterized

from opendbc.car import Bus
from opendbc.car.structs import CarParams
from opendbc.car.bmw.fingerprints import FINGERPRINTS, FW_VERSIONS
from opendbc.car.bmw.values import CAR, DBC
from opendbc.car.interfaces import get_torque_params

Ecu = CarParams.Ecu


class TestBMWInterfaces:
  """BMW-specific interface tests"""

  def test_car_sets(self):
    # Ensure BMW car models are properly defined
    assert len(CAR) > 0
    assert CAR.BMW_E82 in CAR
    assert CAR.BMW_E90 in CAR

  def test_dbc_consistency(self):
    # Verify DBC mappings exist for all BMW models
    for car_model in CAR:
      assert car_model in DBC
      dbc = DBC[car_model]
      # Verify BMW PT-CAN DBC exists
      assert Bus.pt in dbc
      assert dbc[Bus.pt] == "bmw_e9x_e8x"

      # Verify body DBC exists (used for F-CAN)
      if Bus.body in dbc:
        assert dbc[Bus.body] == "bmw_e9x_e8x"

    # Verify ocelot_controls.dbc exists for stepper servo functionality
    # This DBC is used directly in carstate.py line 212 for Bus.alt stepper servo messages
    from opendbc.can import CANParser
    from opendbc.car.bmw.values import CanBus

    try:
      # Test that ocelot_controls.dbc can be loaded with BMW stepper servo message
      test_parser = CANParser('ocelot_controls', [("STEERING_STATUS", 100)], CanBus.SERVO_CAN)
      assert test_parser is not None, "ocelot_controls.dbc should be loadable"

      # Verify the parser has the expected message structure
      assert hasattr(test_parser, 'vl'), "Parser should have vl attribute"
      assert 'STEERING_STATUS' in test_parser.vl, "STEERING_STATUS message should be defined in ocelot_controls.dbc"

      # Test that expected signals exist in STEERING_STATUS message
      expected_signals = ['STEERING_TORQUE', 'STEERING_ANGLE', 'DEBUG_STATES', 'CONTROL_STATUS']
      steering_status = test_parser.vl['STEERING_STATUS']
      for signal in expected_signals:
        assert signal in steering_status, f"Signal {signal} missing from STEERING_STATUS in ocelot_controls.dbc"

    except Exception as e:
      raise AssertionError(f"ocelot_controls.dbc not found or invalid: {e}") from e

  def test_essential_ecus(self, subtests):
    # BMW uses VIN-based detection with dummy FW entries
    for car_model, ecus in FW_VERSIONS.items():
      with subtests.test(car_model=car_model.value):
        present_ecus = {ecu[0] for ecu in ecus}
        # BMW should have at least one dummy ECU for timing fix
        assert len(present_ecus) > 0
        # Verify fwdRadar dummy ECU exists for timing
        assert Ecu.fwdRadar in present_ecus


class TestBMWVINDetection:
  """BMW VIN-based detection and fingerprinting tests"""

  @parameterized.expand(FINGERPRINTS.items())
  def test_empty_can_fingerprints(self, car_model, fingerprints):
    """BMW uses empty CAN fingerprints for VIN-only detection"""
    assert len(fingerprints) > 0

    # For BMW, fingerprints are intentionally empty - VIN detection only
    assert all(isinstance(finger, dict) for finger in fingerprints)

    # BMW fingerprints should be empty dictionaries for VIN-only detection
    for finger in fingerprints:
      assert finger == {}, f"BMW fingerprint should be empty for VIN detection, got: {finger}"

  def test_dummy_fw_versions(self, subtests):
    """BMW uses dummy FW versions to prevent exact matching collision"""
    for car_model, ecus in FW_VERSIONS.items():
      with subtests.test(car_model=car_model.value):
        for _ecu_data, fw_versions in ecus.items():
          for fw in fw_versions:
            # Verify dummy FW versions contain model identifier
            assert b"BMW_" in fw, f"Dummy FW should contain BMW model identifier: {fw}"
            assert b"_DUMMY" in fw, f"Dummy FW should be marked as dummy: {fw}"

  def test_vin_based_fuzzy_match(self):
    """Test BMW VIN-based fuzzy matching works correctly"""
    from opendbc.car.bmw.values import match_fw_to_car_fuzzy

    # Test known E90 VIN
    test_vin_e90 = "LBVPH18059SC20723"  # E90 3-Series
    result = match_fw_to_car_fuzzy({}, test_vin_e90, {})
    assert "BMW_E90" in result, f"VIN should detect BMW_E90: {result}"

    # Test empty/invalid VIN returns empty set
    result = match_fw_to_car_fuzzy({}, "", {})
    assert result == set(), f"Empty VIN should return empty set: {result}"

    result = match_fw_to_car_fuzzy({}, "INVALID", {})
    assert result == set(), f"Invalid VIN should return empty set: {result}"


class TestBMWCarInterface:
  """BMW-specific car interface tests"""

  @parameterized.expand([(name,) for name in CAR])
  def test_car_interface_bmw(self, car_name):
    """Test BMW car interface initialization and basic functionality"""
    from opendbc.car.bmw.interface import CarInterface

    # Test basic car interface initialization
    car_params = CarInterface.get_non_essential_params(car_name)

    # Verify BMW-specific parameters
    assert car_params.brand == "bmw"
    assert car_params.safetyConfigs[0].safetyModel == CarParams.SafetyModel.bmw
    assert car_params.steerControlType == CarParams.SteerControlType.torque


class TestBMWCANParsing:
  """BMW CAN message parsing validation tests"""

  def test_can_parser_configuration(self):
    """Test that BMW CAN parsers are correctly configured for each bus"""
    from opendbc.car.bmw.carstate import CarState
    from opendbc.car.bmw.values import BmwFlags

    # Test all BMW car configurations
    for car_model in CAR:
      # Create CarParams with all BMW feature flags
      CP = CarParams()
      CP.carFingerprint = car_model
      CP.flags = int(BmwFlags.DYNAMIC_CRUISE_CONTROL | BmwFlags.STEPPER_SERVO_CAN)

      # This should not raise any exceptions
      can_parsers = CarState.get_can_parsers(CP)

      # Verify all required parsers exist
      assert Bus.pt in can_parsers, f"Missing PT-CAN parser for {car_model}"
      assert Bus.body in can_parsers, f"Missing F-CAN parser for {car_model}"
      assert Bus.alt in can_parsers, f"Missing Servo-CAN parser for {car_model}"

      # Verify parsers are CANParser objects
      from opendbc.can import CANParser
      assert isinstance(can_parsers[Bus.pt], CANParser)
      assert isinstance(can_parsers[Bus.body], CANParser)
      assert isinstance(can_parsers[Bus.alt], CANParser)

  def test_f_can_parser_uses_correct_dbc(self):
    """Test that F-CAN parser uses Bus.body DBC (critical bug fix)"""
    from opendbc.car.bmw.carstate import CarState
    from opendbc.car.bmw.values import BmwFlags

    for car_model in CAR:
      CP = CarParams()
      CP.carFingerprint = car_model
      CP.flags = int(BmwFlags.DYNAMIC_CRUISE_CONTROL)

      # Get the parsers
      can_parsers = CarState.get_can_parsers(CP)

      # The F-CAN parser (Bus.body) should use the body DBC, not PT DBC
      # This ensures F-CAN messages like CruiseControlStalk can be decoded
      body_parser = can_parsers[Bus.body]

      # Check that the parser was created successfully (would fail if wrong DBC)
      assert body_parser is not None, f"F-CAN parser creation failed for {car_model}"

      # Verify the parser has the expected F-CAN messages in its value list
      # These messages only exist in the correct DBC, so this validates the fix
      if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
        assert hasattr(body_parser, 'vl'), "Parser should have vl attribute"
        # The parser will have these messages if using correct DBC
        # If it was using the wrong DBC (PT instead of body), these would be missing
        expected_fcan_messages = ["CruiseControlStalk", "SteeringWheelAngle_DSC"]
        for msg in expected_fcan_messages:
          assert msg in body_parser.vl, f"F-CAN message {msg} missing - parser using wrong DBC!"

  def test_carstate_instantiation(self):
    """Test that CarState can be instantiated for all BMW models"""
    from opendbc.car.bmw.carstate import CarState
    from opendbc.car.bmw.values import BmwFlags

    for car_model in CAR:
      # Test with different flag combinations
      flag_combinations = [
        0,  # No flags
        int(BmwFlags.NORMAL_CRUISE_CONTROL),
        int(BmwFlags.DYNAMIC_CRUISE_CONTROL),
        int(BmwFlags.STEPPER_SERVO_CAN),
        int(BmwFlags.DYNAMIC_CRUISE_CONTROL | BmwFlags.STEPPER_SERVO_CAN),
      ]

      for flags in flag_combinations:
        CP = CarParams()
        CP.carFingerprint = car_model
        CP.flags = flags

        # Should not raise any exceptions
        cs = CarState(CP)
        assert cs is not None
        assert cs.shifter_values is not None
        assert cs.cluster_min_speed == 2  # CruiseSettings.CLUSTER_OFFSET


class TestBMWPlatformConfig:
  """BMW platform configuration tests"""

  @parameterized.expand([(name,) for name in CAR])
  def test_platform_config_bmw(self, car_name):
    """Test BMW platform configuration integrity"""
    from opendbc.car.values import PLATFORMS

    # BMW-specific validations
    platform = PLATFORMS[str(car_name)]

    # Test platform configuration basics
    assert platform.config._frozen, f"Platform config should be frozen: {car_name}"
    assert len(platform.config.dbc_dict) > 0, f"Platform should have DBC dict: {car_name}"
    assert len(platform.config.platform_str) > 0, f"Platform should have platform_str: {car_name}"
    assert str(car_name) == platform.config.platform_str, f"Platform string mismatch: {car_name}"
    assert platform.config.specs is not None, f"Platform should have specs: {car_name}"

    # BMW-specific DBC validation
    dbc_dict = platform.config.dbc_dict
    assert Bus.pt in dbc_dict, f"BMW should have PT-CAN DBC: {car_name}"
    assert dbc_dict[Bus.pt] == "bmw_e9x_e8x", f"BMW should use bmw_e9x_e8x DBC: {car_name}"


class TestBMWLateralLimits:
  """BMW lateral control tests"""

  @parameterized.expand([(name,) for name in CAR])
  def test_torque_data_present(self, car_name):
    """Ensure BMW has torque data in override.toml"""

    torque_params = get_torque_params()
    platform_str = str(car_name).replace("CAR.", "")

    assert platform_str in torque_params, f"BMW {platform_str} missing from torque_data/override.toml"

    # Validate torque parameter format
    params = torque_params[platform_str]
    assert isinstance(params, dict), f"BMW torque params should be a dict: {params}"

    # Check required fields
    required_fields = ['LAT_ACCEL_FACTOR', 'MAX_LAT_ACCEL_MEASURED', 'FRICTION']
    for field in required_fields:
      assert field in params, f"BMW {platform_str} missing {field} in torque params: {params}"
      assert params[field] > 0, f"BMW {platform_str} {field} should be positive: {params[field]}"

  @parameterized.expand([(name,) for name in CAR])
  def test_lateral_tuning(self, car_name):
    """Test BMW lateral control tuning parameters"""
    from opendbc.car.bmw.interface import CarInterface

    # Get car parameters
    car_params = CarInterface.get_non_essential_params(car_name)

    # Verify torque control type
    assert car_params.steerControlType == CarParams.SteerControlType.torque, \
      f"BMW should use torque control: {car_name}"

    # Verify reasonable tuning parameters
    assert car_params.lateralTuning.which() == 'torque', f"BMW should use torque tuning: {car_name}"

    torque_tune = car_params.lateralTuning.torque
    assert torque_tune.kp > 0, f"BMW kp should be positive: {torque_tune.kp}"
    assert torque_tune.ki >= 0, f"BMW ki should be non-negative: {torque_tune.ki}"
    assert torque_tune.kf > 0, f"BMW kf should be positive: {torque_tune.kf}"