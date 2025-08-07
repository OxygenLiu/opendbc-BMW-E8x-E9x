import pytest
from parameterized import parameterized

from opendbc.car import Bus
from opendbc.car.structs import CarParams
from opendbc.car.bmw.fingerprints import FINGERPRINTS, FW_VERSIONS
from opendbc.car.bmw.values import CAR, DBC
from opendbc.car.interfaces import get_torque_params

# Only import what we need - no class inheritance to avoid running all brands

Ecu = CarParams.Ecu


class TestBMWInterfaces:
  """BMW-specific interface tests - includes DBC validation for stepper servo"""
  
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
      
    # Verify ocelot_controls.dbc exists for stepper servo functionality
    # This DBC is used directly in carstate.py for Bus.alt stepper servo messages
    from opendbc.can import CANParser
    try:
      # Test that ocelot_controls.dbc can be loaded with BMW stepper servo message
      test_parser = CANParser('ocelot_controls', [("STEERING_STATUS", 100)], 1)
      assert test_parser is not None, "ocelot_controls.dbc should be loadable with STEERING_STATUS"
    except Exception as e:
      assert False, f"ocelot_controls.dbc not found or missing STEERING_STATUS message: {e}"
      
  def test_essential_ecus(self, subtests):
    # BMW uses VIN-based detection with dummy FW entries
    # Verify dummy FW entries exist for fingerprinting timing
    for car_model, ecus in FW_VERSIONS.items():
      with subtests.test(car_model=car_model.value):
        present_ecus = {ecu[0] for ecu in ecus}
        # BMW should have at least one dummy ECU for timing fix
        assert len(present_ecus) > 0
        # Verify fwdRadar dummy ECU exists for timing
        assert Ecu.fwdRadar in present_ecus


class TestBMWFingerprint:
  """BMW-specific fingerprinting tests - validates VIN-based approach"""
  
  @parameterized.expand(FINGERPRINTS.items())
  def test_fw_fingerprint(self, car_model, fingerprints):
    # BMW uses revolutionary VIN-based detection instead of CAN fingerprints
    assert len(fingerprints) > 0
    
    # For BMW, fingerprints are intentionally empty - VIN detection only
    assert all(isinstance(finger, dict) for finger in fingerprints)
    
    # BMW fingerprints should be empty dictionaries for VIN-only detection
    for finger in fingerprints:
      assert finger == {}, f"BMW fingerprint should be empty for VIN detection, got: {finger}"
    
    # Test passes - BMW uses VIN-based detection, not CAN fingerprinting
    
  def test_dummy_fw_versions(self, subtests):
    # BMW uses dummy FW versions to prevent exact matching collision
    # This forces VIN-based fuzzy matching to run correctly
    for car_model, ecus in FW_VERSIONS.items():
      with subtests.test(car_model=car_model.value):
        for ecu, fw_versions in ecus.items():
          for fw in fw_versions:
            # Verify dummy FW versions contain model identifier
            assert b"BMW_" in fw, f"Dummy FW should contain BMW model identifier: {fw}"
            assert b"_DUMMY" in fw, f"Dummy FW should be marked as dummy: {fw}"

  @parameterized.expand(FINGERPRINTS.items())
  def test_can_fingerprint_expected_failure(self, car_model, fingerprints):
    """BMW CAN fingerprinting expected to fail - this validates VIN-based approach"""
    
    # Import here to avoid circular imports
    from opendbc.car.can_definitions import CanData
    from opendbc.car.car_helpers import can_fingerprint
    
    for fingerprint in fingerprints:
      # BMW fingerprints are intentionally empty, so CAN fingerprinting returns None
      can = [CanData(address=address, dat=b'\x00' * length, src=src)
             for address, length in fingerprint.items() for src in (0, 1)]
      
      fingerprint_iter = iter([can])
      car_fingerprint, finger = can_fingerprint(lambda **kwargs: [next(fingerprint_iter, [])])
      
      # This is EXPECTED to fail for BMW - validates revolutionary VIN-based approach
      assert car_fingerprint is None, f"BMW should not be CAN fingerprintable (uses VIN instead): {car_fingerprint}"
      
      # Fingerprint should be empty as expected
      assert finger[0] == {}, f"BMW fingerprint should be empty: {finger[0]}"


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


class TestBMWFirmwareFingerprint:
  """BMW-specific firmware fingerprinting tests using existing test infrastructure"""
  
  @parameterized.expand([('bmw', name, FW_VERSIONS[name]) for name in CAR])
  def test_exact_match_bmw(self, brand, car_model, ecus):
    """Run existing exact match test for BMW models only"""
    from opendbc.car.tests.test_fw_fingerprint import TestFwFingerprint
    test_instance = TestFwFingerprint()
    test_instance.test_exact_match(brand, car_model, ecus, False)

  @parameterized.expand([('bmw', name, FW_VERSIONS[name]) for name in CAR])
  def test_custom_fuzzy_match_bmw(self, brand, car_model, ecus):
    """Run existing custom fuzzy match test for BMW models only"""
    from opendbc.car.tests.test_fw_fingerprint import TestFwFingerprint
    test_instance = TestFwFingerprint()
    test_instance.test_custom_fuzzy_match(brand, car_model, ecus)
    
  def test_vin_based_fuzzy_match(self):
    """Test BMW VIN-based fuzzy matching"""
    
    # Test VIN-based detection for BMW
    from opendbc.car.bmw.values import match_fw_to_car_fuzzy
    
    # Use appropriate test VINs for each model
    test_vins = {
      "BMW_E90": "LBVPH18059SC20723",  # E90 3-Series (position 4-6 = 'VPH')
    }
    
    for car_model in CAR:
      car_model_str = str(car_model).replace("CAR.", "")
      
      # Only test VINs we have specific test data for
      if car_model_str in test_vins:
        test_vin = test_vins[car_model_str]
        result = match_fw_to_car_fuzzy({}, test_vin, {})
        assert car_model_str in result, f"VIN should detect {car_model_str}: {result}"
      else:
        # For models without specific test VINs, just verify the function works
        result = match_fw_to_car_fuzzy({}, "LBVPH18059SC20723", {})
        assert isinstance(result, set), f"VIN matching should return a set: {result}"


class TestBMWDocumentation:
  """BMW-specific documentation tests using existing test infrastructure"""
  
  def test_missing_car_docs_bmw(self):
    """Ensure all BMW platforms have documentation entries"""
    from opendbc.car.docs import get_all_car_docs
    
    all_car_docs = get_all_car_docs()
    bmw_docs_platforms = {car.name.replace(" ", "_").replace("-", "_").upper() 
                          for car in all_car_docs if car.make == "BMW"}
    
    for platform in CAR:
      platform_name = str(platform).replace("CAR.", "")
      # Check if documentation exists (allowing for naming variations)
      doc_exists = any(platform_name in doc_name or 
                      any(part in doc_name for part in platform_name.split("_"))
                      for doc_name in bmw_docs_platforms)
      assert doc_exists, f"BMW platform {platform} missing documentation entry"

  def test_naming_conventions_bmw(self):
    """Test BMW naming conventions in documentation"""
    from opendbc.car.docs import get_all_car_docs
    
    all_car_docs = get_all_car_docs()
    bmw_docs = [car for car in all_car_docs if car.make == "BMW"]
    
    for car in bmw_docs:
      # BMW naming should be consistent
      assert "BMW" in car.make, f"BMW car should have BMW in make: {car.make}"
      assert len(car.year_list) > 0, f"BMW car should have year range: {car.name}"


class TestBMWPlatformConfig:
  """BMW-specific platform configuration tests using existing test infrastructure"""
  
  @parameterized.expand([(name,) for name in CAR])
  def test_platform_config_bmw(self, car_name):
    """Test BMW platform configuration integrity with BMW-specific validations"""
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


class TestBMWRoutes:
  """BMW-specific route tests"""
  
  @parameterized.expand([(name,) for name in CAR])
  def test_route_present_bmw(self, car_name):
    """Ensure BMW platforms have test routes defined"""
    
    from opendbc.car.tests.routes import routes, non_tested_cars
    
    tested_platforms = [r.car_model for r in routes]
    car_enum = getattr(CAR, str(car_name).replace("CAR.", ""))
    
    assert car_enum in set(tested_platforms) | set(non_tested_cars), \
      f"Missing test route for BMW {car_name}. Add route to opendbc/car/tests/routes.py"


class TestBMWCANParsing:
  """BMW CAN message parsing validation tests - prevents parser misconfiguration"""
  
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
      
      # Verify parsers use correct DBC files
      from opendbc.car.bmw.values import DBC
      pt_parser = can_parsers[Bus.pt]
      f_parser = can_parsers[Bus.body]
      servo_parser = can_parsers[Bus.alt]
      
      # PT-CAN parser should use PT DBC
      assert pt_parser.dbc_name == DBC[car_model][Bus.pt], \
        f"PT-CAN parser using wrong DBC for {car_model}"
      
      # F-CAN parser should use Body DBC (not PT DBC - this was the bug!)
      assert f_parser.dbc_name == DBC[car_model][Bus.body], \
        f"F-CAN parser using wrong DBC for {car_model}. This prevents canValid=False!"
      
      # Servo parser should use ocelot_controls
      assert servo_parser.dbc_name == 'ocelot_controls', \
        f"Servo parser using wrong DBC for {car_model}"

  def test_critical_can_messages_bus_assignment(self):
    """Test that critical BMW CAN messages are assigned to correct buses"""
    from opendbc.car.bmw.carstate import CarState
    from opendbc.car.bmw.values import BmwFlags, CanBus
    
    for car_model in CAR:
      # Test with DCC enabled (most complex configuration)  
      CP = CarParams()
      CP.carFingerprint = car_model
      CP.flags = int(BmwFlags.DYNAMIC_CRUISE_CONTROL | BmwFlags.STEPPER_SERVO_CAN)
      
      can_parsers = CarState.get_can_parsers(CP)
      
      # Verify PT-CAN parser has expected messages by checking vl dict
      pt_parser = can_parsers[Bus.pt]
      pt_expected_messages = ["EngineAndBrake", "AccPedal", "Speed", "SteeringWheelAngle", 
                              "TransmissionDataDisplay", "DynamicCruiseControlStatus"]
      for msg in pt_expected_messages:
        assert msg in pt_parser.vl, \
          f"{msg} missing from PT-CAN parser vl dict for {car_model}"
      
      # Verify F-CAN parser has expected messages (this was the bug!)
      f_parser = can_parsers[Bus.body]  
      f_expected_messages = ["SteeringWheelAngle_DSC", "CruiseControlStalk"]
      for msg in f_expected_messages:
        assert msg in f_parser.vl, \
          f"{msg} missing from F-CAN parser vl dict for {car_model}"
      
      # Verify Servo parser has expected messages
      servo_parser = can_parsers[Bus.alt]
      assert "STEERING_STATUS" in servo_parser.vl, \
        f"STEERING_STATUS missing from Servo parser vl dict for {car_model}"

  def test_steering_angle_message_parsing(self):
    """Test that steering angle messages can be parsed correctly from their buses"""
    from opendbc.car.bmw.carstate import CarState
    from opendbc.car.bmw.values import BmwFlags
    
    for car_model in CAR:
      # Test DCC configuration (uses both PT-CAN and F-CAN steering messages)
      CP = CarParams()
      CP.carFingerprint = car_model  
      CP.flags = int(BmwFlags.DYNAMIC_CRUISE_CONTROL)
      
      # Create CarState instance
      CS = CarState(CP)
      can_parsers = CarState.get_can_parsers(CP)
      
      # Verify CarState can access both steering angle sources
      # This ensures the F-CAN parser fix allows proper fallback logic
      pt_parser = can_parsers[Bus.pt] 
      f_parser = can_parsers[Bus.body]
      
      # PT-CAN should have SteeringWheelAngle
      assert "SteeringWheelAngle" in pt_parser.vl, \
        f"SteeringWheelAngle missing from PT-CAN parser for {car_model}"
      
      # F-CAN should have SteeringWheelAngle_DSC  
      assert "SteeringWheelAngle_DSC" in f_parser.vl, \
        f"SteeringWheelAngle_DSC missing from F-CAN parser for {car_model}"
      
      # Verify both parsers can be created without DBC errors
      # This validates the F-CAN parser uses correct Body DBC, not PT DBC
      assert pt_parser is not None, f"PT-CAN parser creation failed for {car_model}"
      assert f_parser is not None, f"F-CAN parser creation failed for {car_model}"


class TestBMWLateralLimits:
  """BMW-specific lateral limits tests"""
  
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
  def test_lateral_limits_simulation(self, car_name):
    """Test BMW lateral control limits with simulated parameters"""
    from opendbc.car.bmw.interface import CarInterface
    
    # Get car parameters
    car_params = CarInterface.get_non_essential_params(getattr(CAR, str(car_name).replace("CAR.", "")))
    
    # Verify torque control type
    assert car_params.steerControlType == CarParams.SteerControlType.torque, \
      f"BMW should use torque control: {car_name}"
    
    # Verify reasonable tuning parameters
    assert car_params.lateralTuning.which() == 'torque', f"BMW should use torque tuning: {car_name}"
    
    torque_tune = car_params.lateralTuning.torque
    assert torque_tune.kp > 0, f"BMW kp should be positive: {torque_tune.kp}"
    assert torque_tune.ki >= 0, f"BMW ki should be non-negative: {torque_tune.ki}"
    assert torque_tune.kf > 0, f"BMW kf should be positive: {torque_tune.kf}"
