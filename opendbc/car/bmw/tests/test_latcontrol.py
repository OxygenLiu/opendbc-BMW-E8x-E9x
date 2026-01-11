from parameterized import parameterized

from cereal import car, log
from opendbc.car.car_helpers import interfaces
from opendbc.car.bmw.values import CAR as BMW
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque


class TestBMWLatControl:
  @parameterized.expand([(BMW.BMW_E90, LatControlTorque)])
  def test_saturation(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringPressed = False

    params = log.LiveParametersData.new_message()

    # Saturate for curvature limited and controller limited
    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, True, 0.2)
    assert lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 0, False, 0.2)
    assert not lac_log.saturated

    for _ in range(1000):
      _, _, lac_log = controller.update(True, CS, VM, params, False, 1, False, 0.2)
    assert lac_log.saturated

  @parameterized.expand([(BMW.BMW_E90, LatControlTorque)])
  def test_bmw_version_logging(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringAngleDeg = 0.0

    params = log.LiveParametersData.new_message()
    params.roll = 0.0
    params.angleOffsetDeg = 0.0

    _, _, lac_log = controller.update(True, CS, VM, params, False, 0.0, False, 0.2)

    # BMW should log version 1 (delay independent jerk)
    assert lac_log.version == 1

  @parameterized.expand([(BMW.BMW_E90, LatControlTorque)])
  def test_bmw_delay_independent_jerk(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringAngleDeg = 0.0

    params = log.LiveParametersData.new_message()
    params.roll = 0.0
    params.angleOffsetDeg = 0.0

    # Build up jerk buffer
    for _ in range(100):
      controller.update(True, CS, VM, params, False, 0.0, False, 0.2)

    # Check that desired lateral jerk is logged
    _, _, lac_log = controller.update(True, CS, VM, params, False, 1.0, False, 0.2)
    assert hasattr(lac_log, 'desiredLateralJerk')
    assert lac_log.actualLateralAccel != 0.0
    assert lac_log.desiredLateralAccel != 0.0

  @parameterized.expand([(BMW.BMW_E90, LatControlTorque)])
  def test_bmw_lat_delay_parameter(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    CS = car.CarState.new_message()
    CS.vEgo = 30
    CS.steeringAngleDeg = 0.0

    params = log.LiveParametersData.new_message()
    params.roll = 0.0
    params.angleOffsetDeg = 0.0

    # Test with different lat_delay values (BMW DCC uses this)
    for lat_delay in [0.1, 0.2, 0.3]:
      steer, _, lac_log = controller.update(True, CS, VM, params, False, lat_delay, False, 0.2)
      assert lac_log.active
      assert lac_log.saturated == False

  @parameterized.expand([(BMW.BMW_E90, LatControlTorque)])
  def test_bmw_pid_constants(self, car_name, controller):
    CarInterface = interfaces[car_name]
    CP = CarInterface.get_non_essential_params(car_name)
    CI = CarInterface(CP)
    VM = VehicleModel(CP)

    controller = controller(CP.as_reader(), CI, DT_CTRL)

    # Verify PID constants match updated values
    from openpilot.selfdrive.controls.lib.latcontrol_torque import KP, KI, JERK_LOOKAHEAD_SECONDS, JERK_GAIN

    assert KP == 0.8, f"Expected KP=0.8, got {KP}"
    assert KI == 0.15, f"Expected KI=0.15, got {KI}"
    assert JERK_LOOKAHEAD_SECONDS == 0.19, f"Expected JERK_LOOKAHEAD_SECONDS=0.19, got {JERK_LOOKAHEAD_SECONDS}"
    assert JERK_GAIN == 0.3, f"Expected JERK_GAIN=0.3, got {JERK_GAIN}"
