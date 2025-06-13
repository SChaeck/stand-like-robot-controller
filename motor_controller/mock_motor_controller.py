"""
시뮬레이션용 모의 모터 컨트롤러

핵심 기능:
- 실제 모터 동작 시뮬레이션
- 위치 설정 및 읽기
- 즉시 응답 (실시간 시뮬레이션)
- RealMotorController와 동일한 라디안 기반 API 제공
"""

import numpy as np

class MockMotorController:
    """시뮬레이션용 모의 모터 컨트롤러"""
    
    def __init__(self, motor_id, initial_position, initial_software_rad=0.0):
        """
        모의 모터 초기화
        Args:
            motor_id: 모터 ID
            initial_position: 초기 위치
            initial_software_rad: 초기 소프트웨어 라디안 값
        """
        self.motor_id = motor_id
        self.current_position = initial_position
        self.target_position = initial_position
        
        # 라디안 기반 상태 (YAML에서 정의된 초기값 사용)
        self.current_software_rad = initial_software_rad
        
    ######## Legacy Position-Based API ########
    
    def get_current_position(self):
        """현재 위치 반환"""
        return self.current_position
    
    def read_motor_position(self):
        """모터 위치 읽기 (get_current_position 별칭)"""
        return self.current_position
    
    def move_to_position(self, position):
        """모터 이동 시뮬레이션"""
        self.target_position = position
        self.current_position = position  # 시뮬레이션에서는 즉시 이동
        
    def move_motor_to_position(self, position, time_ms):
        """시간 지정 모터 이동 (move_to_position 별칭)"""
        self.move_to_position(position)
    
    ######## Radian-Based API (RealMotorController 호환성) ########
    
    def move_to_software_radian(self, software_rad):
        """소프트웨어 라디안 각도로 모터 이동 - RealMotorController 호환"""
        self.current_software_rad = software_rad
        # print(f"🎮 MOCK Motor {self.motor_id}: SW={np.rad2deg(software_rad):.1f}° (시뮬레이션)")
    
    def read_current_software_radian(self):
        """현재 소프트웨어 라디안 각도 읽기 - RealMotorController 호환"""
        return self.current_software_rad
    
    ######## Motor Control Functions ########

    def torque_enable(self):
        """Mock torque enable"""
        print(f"🎮 MOCK: {self.motor_id} torque enabled")
        
    def set_moving_speed(self, speed_value):
        """Mock set moving speed"""
        print(f"🎮 MOCK: {self.motor_id} speed set to {speed_value}")
        
    def set_position_p_gain(self, speed_value):
        """Mock set position P gain"""
        print(f"🎮 MOCK: {self.motor_id} P gain set to {speed_value}")
    
    ######## Information and Debugging ########
    
    def get_calibration_info(self):
        """캘리브레이션 정보 반환 - RealMotorController 호환"""
        return {
            'motor_id': self.motor_id,
            'motor_type': 'MOCK',
            'calibration_type': '시뮬레이션 모드',
            'calibration_hw_rad': 0.0,
            'calibration_degrees': 0.0,
            'current_software_rad': self.current_software_rad,
            'angle_offset_rad': 0.0,
            # 레거시 호환성
            'software_zero_hw_position': self.current_position,
            'current_software_position': self.current_position
        }
    
    def get_status_info(self):
        """현재 상태 정보 반환 - RealMotorController 호환"""
        return {
            'motor_id': self.motor_id,
            'motor_type': 'MOCK',
            'current_software_rad': self.current_software_rad,
            'current_software_deg': np.rad2deg(self.current_software_rad),
            'current_hardware_rad': self.current_software_rad,  # 시뮬레이션에서는 동일
            'current_hardware_deg': np.rad2deg(self.current_software_rad),
            'angle_offset_rad': 0.0,
            'angle_offset_deg': 0.0
        } 