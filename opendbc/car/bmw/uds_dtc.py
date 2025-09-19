#!/usr/bin/env python3
"""
BMW UDS (Unified Diagnostic Services) DTC Reading/Clearing Implementation
Phase 1: Foundation - Using existing opendbc UDS framework
"""

import time
from typing import Dict, List, Optional, Any
from collections import deque, defaultdict
from enum import IntEnum
from dataclasses import dataclass

from opendbc.car.carlog import carlog
from opendbc.car.uds import UdsClient, SERVICE_TYPE, DTC_REPORT_TYPE, DTC_STATUS_MASK_TYPE, NegativeResponseError
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery

# ECU Addresses (confirmed from route data analysis)
ECU_ADDRESSES = {
    0x12: {
        'name': 'DME (Engine Control)',
        'request_id': 0x7E0,
        'response_id': 0x7E8,
        'bus': 0,  # PT-CAN
        'priority': 'HIGH',
        'status': 'ACTIVE'
    },
    0x18: {
        'name': 'EGS (Transmission Control)', 
        'request_id': 0x7E4,
        'response_id': 0x7EC,
        'bus': 0,  # PT-CAN
        'priority': 'HIGH',
        'status': 'ACTIVE'
    }
}

# Engine Protection Actions
class ProtectionAction(IntEnum):
    NORMAL = 0
    REDUCE_POWER = 1
    LIMP_MODE = 2
    IMMEDIATE_SHUTDOWN = 3

@dataclass
class DTCInfo:
    """Diagnostic Trouble Code information"""
    code: str  # e.g., "P0301"
    description: str
    severity: str  # "INFO", "WARNING", "CRITICAL"
    ecu_id: int
    ecu_name: str
    timestamp: float
    status: str  # "ACTIVE", "PENDING", "HISTORICAL"
    raw_data: bytes
    
@dataclass
class EngineStatus:
    """Engine monitoring status"""
    coolant_temp: float
    oil_temp: float
    ambient_temp: float
    oil_level: float
    engine_running: bool
    protection_mode: ProtectionAction
    service_distance_km: float

class DTCDatabase:
    """BMW-specific DTC descriptions and severity"""
    
    # Common BMW DTCs
    DTC_DESCRIPTIONS = {
        # Engine (P0xxx)
        'P0300': ('Random/Multiple Cylinder Misfire Detected', 'WARNING'),
        'P0301': ('Cylinder 1 Misfire Detected', 'WARNING'),
        'P0302': ('Cylinder 2 Misfire Detected', 'WARNING'),
        'P0303': ('Cylinder 3 Misfire Detected', 'WARNING'),
        'P0304': ('Cylinder 4 Misfire Detected', 'WARNING'),
        'P0305': ('Cylinder 5 Misfire Detected', 'WARNING'),
        'P0306': ('Cylinder 6 Misfire Detected', 'WARNING'),
        'P0171': ('System Too Lean (Bank 1)', 'WARNING'),
        'P0174': ('System Too Lean (Bank 2)', 'WARNING'),
        'P0128': ('Coolant Thermostat Below Regulating Temperature', 'INFO'),
        'P0420': ('Catalyst Efficiency Below Threshold (Bank 1)', 'WARNING'),
        
        # BMW-specific (P1xxx)
        'P1083': ('Fuel Control Mixture Lean Bank 1 Sensor 1', 'WARNING'),
        'P1084': ('Fuel Control Mixture Rich Bank 1 Sensor 1', 'WARNING'),
        'P1085': ('Fuel Control Mixture Lean Bank 2 Sensor 1', 'WARNING'),
        'P1086': ('Fuel Control Mixture Rich Bank 2 Sensor 1', 'WARNING'),
        
        # Transmission (P0xxx)
        'P0700': ('Transmission Control System Malfunction', 'CRITICAL'),
        'P0715': ('Turbine/Input Speed Sensor Circuit', 'WARNING'),
        'P0720': ('Output Speed Sensor Circuit', 'WARNING'),
        'P0730': ('Incorrect Gear Ratio', 'WARNING'),
        
        # Network (U0xxx)
        'U0001': ('High Speed CAN Communication Bus', 'CRITICAL'),
        'U0100': ('Lost Communication with ECM/PCM', 'CRITICAL'),
        'U0121': ('Lost Communication with ABS Module', 'WARNING'),
        'U0140': ('Lost Communication with Body Control Module', 'INFO'),
    }
    
    @classmethod
    def get_description(cls, dtc_code: str) -> str:
        """Get DTC description"""
        if dtc_code in cls.DTC_DESCRIPTIONS:
            return cls.DTC_DESCRIPTIONS[dtc_code][0]
        return f"Unknown DTC: {dtc_code}"
    
    @classmethod
    def get_severity(cls, dtc_code: str) -> str:
        """Get DTC severity level"""
        if dtc_code in cls.DTC_DESCRIPTIONS:
            return cls.DTC_DESCRIPTIONS[dtc_code][1]
        # Default severity based on prefix
        if dtc_code.startswith('P'):
            return 'WARNING'
        elif dtc_code.startswith('U'):
            return 'WARNING'
        elif dtc_code.startswith('C'):
            return 'INFO'
        elif dtc_code.startswith('B'):
            return 'INFO'
        return 'INFO'

class EngineProtectionSystem:
    """Advanced engine protection with predictive analysis"""
    
    def __init__(self):
        self.coolant_temp_history = deque(maxlen=3600)  # 1 hour at 1Hz
        self.oil_temp_history = deque(maxlen=3600)
        self.protection_events = deque(maxlen=100)  # Track protection events
        self.last_protection_action = ProtectionAction.NORMAL
        self.thermal_stress_score = 0.0
        
    def update(self, coolant_temp: float, oil_temp: float, ambient_temp: float) -> ProtectionAction:
        """Enhanced protection with predictive analysis"""
        current_time = time.time()
        
        # Add to history
        temp_data = {
            'timestamp': current_time,
            'coolant': coolant_temp,
            'oil': oil_temp,
            'ambient': ambient_temp
        }
        self.coolant_temp_history.append(temp_data)
        self.oil_temp_history.append(temp_data)
        
        # Calculate thermal stress
        self._calculate_thermal_stress()
        
        # Enhanced protection logic with trend analysis
        action = self._determine_protection_action(coolant_temp, oil_temp)
        
        # Log protection events
        if action != self.last_protection_action:
            self.protection_events.append({
                'timestamp': current_time,
                'old_action': self.last_protection_action,
                'new_action': action,
                'coolant_temp': coolant_temp,
                'oil_temp': oil_temp,
                'thermal_stress': self.thermal_stress_score
            })
            
        self.last_protection_action = action
        return action
    
    def _determine_protection_action(self, coolant_temp: float, oil_temp: float) -> ProtectionAction:
        """Determine protection action with enhanced logic"""
        
        # Get temperature trends
        coolant_trend = self._get_temperature_trend(self.coolant_temp_history, 'coolant')
        oil_trend = self._get_temperature_trend(self.oil_temp_history, 'oil')
        
        # Critical temperature monitoring with trend consideration
        if coolant_temp > 105 or (coolant_temp > 100 and coolant_trend > 2):
            return ProtectionAction.IMMEDIATE_SHUTDOWN
        elif coolant_temp > 95 or (coolant_temp > 90 and coolant_trend > 1):
            return ProtectionAction.REDUCE_POWER
        elif oil_temp > 150 or (oil_temp > 145 and oil_trend > 3):
            return ProtectionAction.IMMEDIATE_SHUTDOWN
        elif oil_temp > 130 or (oil_temp > 125 and oil_trend > 2):
            return ProtectionAction.LIMP_MODE
        elif self.thermal_stress_score > 0.8:  # High cumulative stress
            return ProtectionAction.LIMP_MODE
        else:
            return ProtectionAction.NORMAL
    
    def _get_temperature_trend(self, history: deque, temp_type: str) -> float:
        """Calculate temperature trend (°C/minute)"""
        if len(history) < 60:  # Need at least 1 minute of data
            return 0.0
            
        recent_data = list(history)[-60:]  # Last minute
        if len(recent_data) < 2:
            return 0.0
            
        temps = [data[temp_type] for data in recent_data]
        time_span = recent_data[-1]['timestamp'] - recent_data[0]['timestamp']
        
        if time_span <= 0:
            return 0.0
            
        temp_change = temps[-1] - temps[0]
        return (temp_change / time_span) * 60  # Convert to °C/minute
    
    def _calculate_thermal_stress(self):
        """Calculate cumulative thermal stress score"""
        if len(self.coolant_temp_history) < 10:
            return
            
        recent_data = list(self.coolant_temp_history)[-300:]  # Last 5 minutes
        
        stress_factors = []
        for data in recent_data:
            coolant_stress = max(0, (data['coolant'] - 85) / 20)  # 0-1 scale above 85°C
            oil_stress = max(0, (data['oil'] - 100) / 50)  # 0-1 scale above 100°C
            combined_stress = min(1.0, coolant_stress + oil_stress)
            stress_factors.append(combined_stress)
        
        # Exponential moving average
        if stress_factors:
            current_stress = sum(stress_factors) / len(stress_factors)
            alpha = 0.1
            self.thermal_stress_score = (
                alpha * current_stress + 
                (1 - alpha) * self.thermal_stress_score
            )
    
    def analyze_cooling_efficiency(self, coolant_temp: float, ambient_temp: float, 
                                  vehicle_speed: float) -> Dict[str, Any]:
        """Enhanced cooling system analysis"""
        temp_differential = coolant_temp - ambient_temp
        
        analysis = {
            'status': 'NORMAL',
            'efficiency_score': 1.0,
            'issues': [],
            'recommendations': []
        }
        
        # Speed-dependent analysis
        if vehicle_speed > 80:  # Highway driving
            expected_differential = 15 + (vehicle_speed - 80) * 0.1  # Slightly higher at very high speeds
            if temp_differential > expected_differential + 15:
                analysis['status'] = 'POOR'
                analysis['issues'].append('High temperature at highway speeds')
                analysis['recommendations'].append('Check radiator and cooling fan')
        elif vehicle_speed > 50:  # Normal highway
            if temp_differential > 30:
                analysis['status'] = 'DEGRADED'
                analysis['issues'].append('Elevated cooling differential')
                analysis['recommendations'].append('Inspect cooling system')
        elif vehicle_speed < 20:  # City/idle
            if temp_differential > 50:
                analysis['status'] = 'POOR'
                analysis['issues'].append('Poor cooling at low speeds')
                analysis['recommendations'].append('Check cooling fan operation')
        
        # Thermostat analysis
        if temp_differential < 10 and coolant_temp > 70:
            analysis['status'] = 'THERMOSTAT_ISSUE'
            analysis['issues'].append('Thermostat may be stuck open')
            analysis['recommendations'].append('Check thermostat operation')
        
        # Calculate efficiency score
        if analysis['status'] == 'NORMAL':
            analysis['efficiency_score'] = 1.0
        elif analysis['status'] == 'DEGRADED':
            analysis['efficiency_score'] = 0.7
        else:
            analysis['efficiency_score'] = 0.4
            
        return analysis
    
    def get_protection_history(self) -> List[Dict]:
        """Get recent protection events"""
        return list(self.protection_events)
    
    def get_thermal_analysis(self) -> Dict[str, Any]:
        """Get comprehensive thermal analysis"""
        if not self.coolant_temp_history:
            return {'status': 'insufficient_data'}
            
        recent_data = list(self.coolant_temp_history)[-300:]  # Last 5 minutes
        
        coolant_temps = [d['coolant'] for d in recent_data]
        oil_temps = [d['oil'] for d in recent_data]
        
        return {
            'thermal_stress_score': self.thermal_stress_score,
            'coolant_stats': {
                'current': coolant_temps[-1] if coolant_temps else 0,
                'avg_5min': sum(coolant_temps) / len(coolant_temps) if coolant_temps else 0,
                'max_5min': max(coolant_temps) if coolant_temps else 0,
                'trend': self._get_temperature_trend(self.coolant_temp_history, 'coolant')
            },
            'oil_stats': {
                'current': oil_temps[-1] if oil_temps else 0,
                'avg_5min': sum(oil_temps) / len(oil_temps) if oil_temps else 0,
                'max_5min': max(oil_temps) if oil_temps else 0,
                'trend': self._get_temperature_trend(self.oil_temp_history, 'oil')
            },
            'protection_events_count': len(self.protection_events)
        }

class Diagnostics:
    """Main diagnostic interface using opendbc UDS framework"""
    
    def __init__(self, panda, CP):
        self.panda = panda
        self.CP = CP
        self.dtc_database = DTCDatabase()
        self.engine_protection = EngineProtectionSystem()
        
        # Create UDS clients for known ECUs
        self.uds_clients = {}
        for ecu_id, ecu_info in ECU_ADDRESSES.items():
            if ecu_info['status'] == 'ACTIVE':
                try:
                    client = UdsClient(
                        panda=panda,
                        tx_addr=ecu_info['request_id'],
                        rx_addr=ecu_info['response_id'],
                        bus=ecu_info['bus'],
                        timeout=1.0
                    )
                    self.uds_clients[ecu_id] = client
                    carlog.info(f"UDS client created for {ecu_info['name']}")
                except Exception as e:
                    carlog.error(f"Failed to create UDS client for {ecu_info['name']}: {e}")
        
        # Enhanced DTC storage with change detection
        self.active_dtcs: Dict[int, List[DTCInfo]] = defaultdict(list)
        self.pending_dtcs: Dict[int, List[DTCInfo]] = defaultdict(list)
        self.historical_dtcs: Dict[int, List[DTCInfo]] = defaultdict(list)
        self.dtc_snapshots: Dict[int, Dict[str, DTCInfo]] = defaultdict(dict)  # For change detection
        
        # DTC change tracking
        self.new_dtcs_since_last_check: List[DTCInfo] = []
        self.cleared_dtcs_since_last_check: List[DTCInfo] = []
        self.dtc_change_callbacks: List[callable] = []
        
        # Enhanced engine monitoring
        self.engine_status = EngineStatus(
            coolant_temp=0.0,
            oil_temp=0.0,
            ambient_temp=0.0,
            oil_level=1.0,
            engine_running=False,
            protection_mode=ProtectionAction.NORMAL,
            service_distance_km=15000.0
        )
        
        # Advanced monitoring data
        self.temperature_history = deque(maxlen=3600)  # 1 hour at 1Hz
        self.oil_quality_data = {
            'degradation_score': 0.0,
            'service_interval_km': 15000.0,
            'high_temp_exposure_time': 0.0,
            'last_service_km': 0.0
        }
        
        # Timing control
        self.last_dtc_check = 0.0
        self.dtc_check_interval = 30.0  # Check DTCs every 30 seconds
        self.last_temperature_log = 0.0
        self.temperature_log_interval = 1.0  # Log temperatures every second
        
        carlog.info("Diagnostics initialized with Phase 2 enhancements")
        
    def update(self, cp_pt, cp_body, current_time: float, vehicle_speed: float = 0.0) -> EngineStatus:
        """Enhanced update with comprehensive monitoring"""
        
        # Process engine temperature data from PT-CAN (0x1D0)
        if cp_pt and cp_pt.vl:
            if "EngineData" in cp_pt.vl:
                engine_data = cp_pt.vl["EngineData"]
                
                # Extract temperatures with BMW-specific scaling
                if 'TEMP_ENG' in engine_data:
                    self.engine_status.coolant_temp = engine_data['TEMP_ENG'] - 48  # (1,-48) scaling
                if 'TEMP_EOI' in engine_data:
                    self.engine_status.oil_temp = engine_data['TEMP_EOI'] - 48  # (1,-48) scaling
                if 'ST_ENG_RUN' in engine_data:
                    self.engine_status.engine_running = (engine_data['ST_ENG_RUN'] == 2)  # 2 = Running
                    
                # Extract additional engine parameters
                if 'AIP_ENG' in engine_data:
                    intake_pressure = engine_data['AIP_ENG'] * 2 + 598  # BMW scaling (2,598)
                if 'IJV_FU' in engine_data:
                    fuel_temp = engine_data['IJV_FU'] - 48  # BMW scaling (1,-48)
                    
        # Process ambient temperature from K-CAN (0x2CA)
        if cp_body and cp_body.vl:
            if "Outside_temperature" in cp_body.vl:
                ambient_data = cp_body.vl["Outside_temperature"]
                if 'ST_TEMP_OUT' in ambient_data:
                    # BMW ambient temp scaling
                    self.engine_status.ambient_temp = ambient_data['ST_TEMP_OUT'] * 0.5 - 50
            
            # Enhanced oil system monitoring (0x381, 0x382)
            if "EngineOilLevel" in cp_body.vl:
                oil_data = cp_body.vl["EngineOilLevel"]
                if 'OIL_LEVEL_1' in oil_data:
                    self.engine_status.oil_level = oil_data['OIL_LEVEL_1'] / 100.0  # Convert to 0-1 range
                
                # Process oil quality indicators
                if 'OIL_DEGRADATION' in oil_data:
                    self.oil_quality_data['degradation_score'] = oil_data['OIL_DEGRADATION'] / 100.0
                    
            # Electronic oil dipstick data (0x382)
            if "Electronic_engine_oil_dipstick_M" in cp_body.vl:
                dipstick_data = cp_body.vl["Electronic_engine_oil_dipstick_M"]
                if 'PRECISE_OIL_LEVEL' in dipstick_data:
                    precise_level = dipstick_data['PRECISE_OIL_LEVEL']
                    # Temperature compensated oil level
                    compensated_level = self._compensate_oil_level(precise_level, self.engine_status.oil_temp)
                    self.engine_status.oil_level = max(self.engine_status.oil_level, compensated_level)
        
        # Enhanced temperature monitoring with history
        if current_time - self.last_temperature_log > self.temperature_log_interval:
            self._log_temperature_data(current_time, vehicle_speed)
            self.last_temperature_log = current_time
        
        # Update engine protection with enhanced analysis
        self.engine_status.protection_mode = self.engine_protection.update(
            self.engine_status.coolant_temp,
            self.engine_status.oil_temp,
            self.engine_status.ambient_temp
        )
        
        # Update oil quality analysis
        self._update_oil_quality_analysis(current_time)
        
        # Periodic DTC check
        if current_time - self.last_dtc_check > self.dtc_check_interval:
            self.request_dtcs()
            self.last_dtc_check = current_time
            
        return self.engine_status
    
    def _compensate_oil_level(self, raw_level: float, oil_temp: float) -> float:
        """Compensate oil level reading for temperature"""
        # Oil expands ~0.07% per degree C above 20C
        temp_correction = 1.0 + (oil_temp - 20) * 0.0007
        return raw_level / temp_correction
    
    def _log_temperature_data(self, current_time: float, vehicle_speed: float):
        """Comprehensive temperature data logging with predictive analysis"""
        temp_data = {
            'timestamp': current_time,
            'coolant_temp': self.engine_status.coolant_temp,
            'oil_temp': self.engine_status.oil_temp,
            'ambient_temp': self.engine_status.ambient_temp,
            'vehicle_speed': vehicle_speed,
            'engine_running': self.engine_status.engine_running,
            'protection_mode': self.engine_status.protection_mode,
            'thermal_stress': self._calculate_thermal_stress(),
            'cooling_efficiency': self._get_current_cooling_efficiency(vehicle_speed)
        }
        self.temperature_history.append(temp_data)
        
        # Track high temperature exposure with granular levels
        if self.engine_status.oil_temp > 120:
            self.oil_quality_data['high_temp_exposure_time'] += self.temperature_log_interval
        if self.engine_status.coolant_temp > 95:
            # Track coolant overheat exposure
            if 'coolant_overheat_time' not in self.oil_quality_data:
                self.oil_quality_data['coolant_overheat_time'] = 0.0
            self.oil_quality_data['coolant_overheat_time'] += self.temperature_log_interval
            
        # Perform predictive analysis every 60 seconds
        if len(self.temperature_history) % 60 == 0:
            self._perform_temperature_trend_analysis()
    
    def _calculate_thermal_stress(self) -> float:
        """Calculate comprehensive thermal stress score (0-100)"""
        coolant_stress = max(0, (self.engine_status.coolant_temp - 85) / 20.0) * 40  # 40% weight
        oil_stress = max(0, (self.engine_status.oil_temp - 100) / 40.0) * 35      # 35% weight
        
        # Temperature differential stress (rapid changes)
        diff_stress = 0
        if len(self.temperature_history) >= 2:
            prev_data = self.temperature_history[-2]
            coolant_delta = abs(self.engine_status.coolant_temp - prev_data['coolant_temp'])
            oil_delta = abs(self.engine_status.oil_temp - prev_data['oil_temp'])
            diff_stress = min((coolant_delta + oil_delta) / 10.0, 1.0) * 15  # 15% weight
        
        # Ambient compensation factor
        ambient_factor = max(0, (self.engine_status.ambient_temp - 25) / 15.0) * 10  # 10% weight
        
        return min(100, coolant_stress + oil_stress + diff_stress + ambient_factor)
    
    def _get_current_cooling_efficiency(self, vehicle_speed: float) -> str:
        """Get real-time cooling efficiency assessment"""
        return self.engine_protection.analyze_cooling_efficiency(
            self.engine_status.coolant_temp,
            self.engine_status.ambient_temp,
            vehicle_speed
        )
    
    def _perform_temperature_trend_analysis(self):
        """Perform predictive temperature trend analysis"""
        if len(self.temperature_history) < 300:  # Need 5 minutes of data
            return
            
        recent_data = list(self.temperature_history)[-300:]  # Last 5 minutes
        
        # Calculate temperature trends
        coolant_temps = [d['coolant_temp'] for d in recent_data]
        oil_temps = [d['oil_temp'] for d in recent_data]
        
        # Simple linear trend calculation
        coolant_trend = (coolant_temps[-1] - coolant_temps[0]) / len(coolant_temps)
        oil_trend = (oil_temps[-1] - oil_temps[0]) / len(oil_temps)
        
        # Detect concerning trends
        if coolant_trend > 0.1:  # Rising > 0.1°C per minute
            carlog.warning(f"Rising coolant temperature trend: +{coolant_trend:.2f}°C/min")
        if oil_trend > 0.15:  # Rising > 0.15°C per minute  
            carlog.warning(f"Rising oil temperature trend: +{oil_trend:.2f}°C/min")
        
        # Update trend data for UI
        if not hasattr(self, 'temperature_trends'):
            self.temperature_trends = {}
        
        self.temperature_trends.update({
            'coolant_trend_c_per_min': coolant_trend,
            'oil_trend_c_per_min': oil_trend,
            'avg_thermal_stress': sum(d['thermal_stress'] for d in recent_data) / len(recent_data),
            'peak_thermal_stress': max(d['thermal_stress'] for d in recent_data),
            'cooling_efficiency_distribution': self._analyze_cooling_distribution(recent_data)
        })
    
    def _analyze_cooling_distribution(self, data_points: List[Dict]) -> Dict[str, float]:
        """Analyze distribution of cooling efficiency states"""
        efficiency_counts = {}
        for point in data_points:
            efficiency = point.get('cooling_efficiency', 'UNKNOWN')
            efficiency_counts[efficiency] = efficiency_counts.get(efficiency, 0) + 1
        
        total = len(data_points)
        return {state: count/total for state, count in efficiency_counts.items()}
        
    def get_comprehensive_temperature_report(self) -> Dict[str, Any]:
        """Generate comprehensive temperature monitoring report"""
        if not self.temperature_history:
            return {'status': 'no_data', 'message': 'No temperature data available'}
        
        recent_data = list(self.temperature_history)[-300:]  # Last 5 minutes
        all_data = list(self.temperature_history)  # All recorded data
        
        # Current status
        current_status = {
            'coolant_temp': self.engine_status.coolant_temp,
            'oil_temp': self.engine_status.oil_temp,
            'ambient_temp': self.engine_status.ambient_temp,
            'thermal_stress_score': self._calculate_thermal_stress(),
            'protection_mode': self.engine_status.protection_mode.name
        }
        
        # Historical analysis
        if recent_data:
            coolant_temps = [d['coolant_temp'] for d in recent_data]
            oil_temps = [d['oil_temp'] for d in recent_data]
            
            historical_analysis = {
                'recent_5min': {
                    'coolant_avg': sum(coolant_temps) / len(coolant_temps),
                    'coolant_max': max(coolant_temps),
                    'coolant_min': min(coolant_temps),
                    'oil_avg': sum(oil_temps) / len(oil_temps),
                    'oil_max': max(oil_temps),
                    'oil_min': min(oil_temps),
                    'overheat_events': sum(1 for t in coolant_temps if t > 105),
                    'high_oil_temp_events': sum(1 for t in oil_temps if t > 130)
                }
            }
        else:
            historical_analysis = {'recent_5min': 'insufficient_data'}
        
        # Trend analysis
        trends = getattr(self, 'temperature_trends', {})
        
        # Recommendations
        recommendations = self._generate_temperature_recommendations(current_status, historical_analysis, trends)
        
        return {
            'status': 'comprehensive',
            'current_status': current_status,
            'historical_analysis': historical_analysis,
            'trends': trends,
            'recommendations': recommendations,
            'data_points': len(all_data),
            'monitoring_duration_hours': (len(all_data) * self.temperature_log_interval) / 3600
        }
    
    def _generate_temperature_recommendations(self, current: Dict, historical: Dict, trends: Dict) -> List[str]:
        """Generate intelligent temperature management recommendations"""
        recommendations = []
        
        # Current temperature checks
        if current['coolant_temp'] > 100:
            recommendations.append("URGENT: Coolant temperature critical - reduce engine load immediately")
        elif current['coolant_temp'] > 95:
            recommendations.append("WARNING: Coolant temperature elevated - monitor closely")
            
        if current['oil_temp'] > 140:
            recommendations.append("URGENT: Oil temperature critical - engine protection active")
        elif current['oil_temp'] > 125:
            recommendations.append("CAUTION: Oil temperature high - consider reducing sustained load")
        
        # Trend-based recommendations  
        if trends.get('coolant_trend_c_per_min', 0) > 0.15:
            recommendations.append("Coolant temperature rising rapidly - check cooling system")
        if trends.get('oil_trend_c_per_min', 0) > 0.2:
            recommendations.append("Oil temperature climbing - verify oil level and quality")
            
        # Thermal stress analysis
        thermal_stress = current.get('thermal_stress_score', 0)
        if thermal_stress > 80:
            recommendations.append("High thermal stress detected - consider extended cool-down period")
        elif thermal_stress > 60:
            recommendations.append("Moderate thermal stress - allow gradual cool-down")
        
        # Cooling efficiency analysis
        cooling_dist = trends.get('cooling_efficiency_distribution', {})
        if cooling_dist.get('INSUFFICIENT', 0) > 0.3:  # >30% insufficient cooling
            recommendations.append("Frequent cooling insufficiency - check radiator and fans")
        
        # Oil quality recommendations
        high_temp_exposure = self.oil_quality_data.get('high_temp_exposure_time', 0)
        if high_temp_exposure > 1800:  # 30 minutes
            recommendations.append("Extended high oil temperatures - consider early oil change")
            
        if not recommendations:
            recommendations.append("Temperature monitoring normal - continue regular operation")
            
        return recommendations
    
    def _update_oil_quality_analysis(self, current_time: float):
        """Update oil quality and service interval analysis"""
        if not self.temperature_history:
            return
            
        # Calculate degradation factors
        recent_data = list(self.temperature_history)[-300:]  # Last 5 minutes
        
        high_temp_count = sum(1 for data in recent_data if data['oil_temp'] > 120)
        extreme_temp_count = sum(1 for data in recent_data if data['oil_temp'] > 140)
        
        # Update degradation score
        high_temp_factor = min(high_temp_count / 300.0, 1.0)  # 0-1 based on % time over 120C
        extreme_temp_factor = min(extreme_temp_count / 300.0, 1.0) * 2  # Double weight for extreme temps
        low_oil_factor = 2.0 if self.engine_status.oil_level < 0.3 else 0.0
        
        current_degradation = high_temp_factor + extreme_temp_factor + low_oil_factor
        
        # Exponential moving average for degradation score
        alpha = 0.01  # Slow adaptation
        self.oil_quality_data['degradation_score'] = (
            alpha * current_degradation + 
            (1 - alpha) * self.oil_quality_data['degradation_score']
        )
        
        # Calculate dynamic service interval
        base_interval = 15000  # km
        degradation_multiplier = max(0.5, 1.0 - self.oil_quality_data['degradation_score'])
        self.oil_quality_data['service_interval_km'] = base_interval * degradation_multiplier
        
        # Update engine status service distance
        self.engine_status.service_distance_km = self.oil_quality_data['service_interval_km']
    
    def _decode_dtc(self, dtc_bytes: bytes) -> Optional[str]:
        """Decode DTC bytes to standard format (P/C/B/U codes)"""
        if len(dtc_bytes) < 3:
            return None
            
        # Standard OBD-II DTC format
        dtc_num = (dtc_bytes[0] << 8) | dtc_bytes[1]
        
        # First two bits determine the prefix
        prefix_map = {
            0: 'P',  # Powertrain
            1: 'C',  # Chassis
            2: 'B',  # Body
            3: 'U',  # Network
        }
        
        prefix = prefix_map.get((dtc_num >> 14) & 0x03, 'P')
        
        # Format: Prefix + 4 hex digits
        code_num = dtc_num & 0x3FFF
        return f"{prefix}{code_num:04X}"
    
    def _update_dtcs(self, ecu_id: int, dtc_codes: List[str], raw_data: bytes = b''):
        """Enhanced DTC update with change detection"""
        current_time = time.time()
        ecu_info = ECU_ADDRESSES.get(ecu_id, {})
        ecu_name = ecu_info.get('name', f'ECU_{ecu_id:02X}')
        
        # Get previous DTCs for change detection
        previous_dtcs = set(dtc.code for dtc in self.active_dtcs[ecu_id])
        current_dtcs = set(dtc_codes)
        
        # Detect changes
        new_dtcs = current_dtcs - previous_dtcs
        cleared_dtcs = previous_dtcs - current_dtcs
        
        # Clear old DTCs for this ECU
        self.active_dtcs[ecu_id].clear()
        self.new_dtcs_since_last_check.clear()
        self.cleared_dtcs_since_last_check.clear()
        
        # Add new DTCs
        for dtc_code in dtc_codes:
            dtc_info = DTCInfo(
                code=dtc_code,
                description=self.dtc_database.get_description(dtc_code),
                severity=self.dtc_database.get_severity(dtc_code),
                ecu_id=ecu_id,
                ecu_name=ecu_name,
                timestamp=current_time,
                status='ACTIVE',
                raw_data=raw_data
            )
            self.active_dtcs[ecu_id].append(dtc_info)
            
            # Track new DTCs
            if dtc_code in new_dtcs:
                self.new_dtcs_since_last_check.append(dtc_info)
                carlog.warning(f"NEW DTC detected: {dtc_code} - {dtc_info.description}")
                
            # Log critical DTCs
            if dtc_info.severity == 'CRITICAL':
                carlog.critical(f"Critical DTC: {dtc_code} - {dtc_info.description}")
        
        # Track cleared DTCs
        for cleared_code in cleared_dtcs:
            if cleared_code in self.dtc_snapshots[ecu_id]:
                cleared_dtc = self.dtc_snapshots[ecu_id][cleared_code]
                cleared_dtc.status = 'CLEARED'
                cleared_dtc.timestamp = current_time
                self.cleared_dtcs_since_last_check.append(cleared_dtc)
                self.historical_dtcs[ecu_id].append(cleared_dtc)
                carlog.info(f"DTC cleared: {cleared_code}")
        
        # Update snapshots for next comparison
        self.dtc_snapshots[ecu_id] = {dtc.code: dtc for dtc in self.active_dtcs[ecu_id]}
        
        # Call change detection callbacks
        if new_dtcs or cleared_dtcs:
            self._notify_dtc_changes(ecu_id, new_dtcs, cleared_dtcs)
    
    def _notify_dtc_changes(self, ecu_id: int, new_dtcs: set, cleared_dtcs: set):
        """Notify registered callbacks of DTC changes"""
        for callback in self.dtc_change_callbacks:
            try:
                callback(ecu_id, new_dtcs, cleared_dtcs)
            except Exception as e:
                carlog.error(f"Error in DTC change callback: {e}")
    
    def register_dtc_change_callback(self, callback: callable):
        """Register callback for DTC changes"""
        self.dtc_change_callbacks.append(callback)
    
    def get_dtc_changes_since_last_check(self) -> Dict[str, List[DTCInfo]]:
        """Get DTCs that have changed since last check"""
        return {
            'new': self.new_dtcs_since_last_check.copy(),
            'cleared': self.cleared_dtcs_since_last_check.copy()
        }
    
    def request_dtcs(self) -> Dict[int, List[str]]:
        """Request DTCs from all known ECUs using UDS framework"""
        all_dtcs = {}
        
        for ecu_id, client in self.uds_clients.items():
            ecu_info = ECU_ADDRESSES[ecu_id]
            try:
                # Use standard UDS service 0x19 subfn 0x02 to read DTCs
                # Report type: 0x02 = reportDTCByStatusMask
                # Status mask: 0xFF = all DTCs
                response = client.read_dtc_information(
                    report_type=DTC_REPORT_TYPE.DTC_BY_STATUS_MASK,
                    status_mask=DTC_STATUS_MASK_TYPE.ALL
                )
                
                if response:
                    dtc_list = []
                    # Parse DTC data (3 bytes per DTC: 2 for code, 1 for status)
                    for i in range(0, len(response), 3):
                        if i + 2 < len(response):
                            dtc_code = self._decode_dtc(response[i:i+3])
                            if dtc_code:
                                dtc_list.append(dtc_code)
                    
                    all_dtcs[ecu_id] = dtc_list
                    self._update_dtcs(ecu_id, dtc_list)
                    
                    if dtc_list:
                        carlog.info(f"Found {len(dtc_list)} DTCs in {ecu_info['name']}")
                        
            except NegativeResponseError as e:
                carlog.warning(f"Negative response from {ecu_info['name']}: {e}")
            except Exception as e:
                carlog.error(f"Failed to read DTCs from {ecu_info['name']}: {e}")
                
        return all_dtcs
    
    def clear_dtcs(self, ecu_id: Optional[int] = None) -> bool:
        """Clear DTCs for specific ECU or all ECUs using UDS framework"""
        success = True
        
        ecus_to_clear = [ecu_id] if ecu_id else list(self.uds_clients.keys())
        
        for ecu_id in ecus_to_clear:
            if ecu_id in self.uds_clients:
                ecu_info = ECU_ADDRESSES[ecu_id]
                try:
                    # Use standard UDS service 0x14 to clear DTCs
                    # Group of DTC: 0xFFFFFF = all groups
                    self.uds_clients[ecu_id].clear_diagnostic_information(0xFFFFFF)
                    
                    # Clear local storage
                    self.active_dtcs[ecu_id].clear()
                    self.pending_dtcs[ecu_id].clear()
                    
                    carlog.info(f"DTCs cleared for {ecu_info['name']}")
                    
                except NegativeResponseError as e:
                    carlog.warning(f"Failed to clear DTCs for {ecu_info['name']}: {e}")
                    success = False
                except Exception as e:
                    carlog.error(f"Error clearing DTCs for {ecu_info['name']}: {e}")
                    success = False
            
        return success
    
    def get_active_dtcs(self) -> List[DTCInfo]:
        """Get all active DTCs from all ECUs"""
        all_dtcs = []
        for ecu_dtcs in self.active_dtcs.values():
            all_dtcs.extend(ecu_dtcs)
        return all_dtcs
    
    def get_engine_protection_status(self) -> Dict[str, Any]:
        """Get current engine protection status"""
        return {
            'protection_mode': self.engine_status.protection_mode,
            'coolant_temp': self.engine_status.coolant_temp,
            'oil_temp': self.engine_status.oil_temp,
            'cooling_efficiency': self.engine_protection.analyze_cooling_efficiency(
                self.engine_status.coolant_temp,
                self.engine_status.ambient_temp,
                0  # TODO: Get actual vehicle speed
            ),
            'action_required': self._get_protection_action_text(self.engine_status.protection_mode)
        }
    
    def _get_protection_action_text(self, action: ProtectionAction) -> str:
        """Get human-readable protection action"""
        action_texts = {
            ProtectionAction.NORMAL: "Normal operation",
            ProtectionAction.REDUCE_POWER: "Reduce power - engine overheating",
            ProtectionAction.LIMP_MODE: "Drive conservatively - oil temperature high",
            ProtectionAction.IMMEDIATE_SHUTDOWN: "PULL OVER IMMEDIATELY - Critical temperature"
        }
        return action_texts.get(action, "Unknown")
    
    def get_diagnostic_summary(self) -> Dict[str, Any]:
        """Get complete diagnostic summary for UI display with comprehensive monitoring"""
        active_dtcs = self.get_active_dtcs()
        
        # Get comprehensive temperature report
        temperature_report = self.get_comprehensive_temperature_report()
        
        return {
            'dtc_count': len(active_dtcs),
            'critical_dtcs': [dtc for dtc in active_dtcs if dtc.severity == 'CRITICAL'],
            'warning_dtcs': [dtc for dtc in active_dtcs if dtc.severity == 'WARNING'],
            'info_dtcs': [dtc for dtc in active_dtcs if dtc.severity == 'INFO'],
            'engine_status': {
                'coolant_temp': self.engine_status.coolant_temp,
                'oil_temp': self.engine_status.oil_temp,
                'ambient_temp': self.engine_status.ambient_temp,
                'oil_level': self.engine_status.oil_level,
                'running': self.engine_status.engine_running,
                'protection': self.engine_status.protection_mode.name,
                'service_km': self.engine_status.service_distance_km,
                'thermal_stress_score': self._calculate_thermal_stress()
            },
            'cooling_status': self.engine_protection.analyze_cooling_efficiency(
                self.engine_status.coolant_temp,
                self.engine_status.ambient_temp,
                0  # TODO: Get actual vehicle speed
            ),
            'comprehensive_temperature_monitoring': temperature_report,
            'oil_quality_analysis': {
                'degradation_score': self.oil_quality_data['degradation_score'],
                'service_interval_km': self.oil_quality_data['service_interval_km'],
                'high_temp_exposure_hours': self.oil_quality_data.get('high_temp_exposure_time', 0) / 3600,
                'coolant_overheat_hours': self.oil_quality_data.get('coolant_overheat_time', 0) / 3600
            }
        }

# Export main class
__all__ = ['Diagnostics', 'EngineStatus', 'DTCInfo', 'ProtectionAction']