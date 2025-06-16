"""
실제 모터 컨트롤러 - 단일 라디안 기반 시스템

핵심 기능:
1. MockMotorController와 동일한 API 제공 (인터페이스 통일)
2. 단일 라디안 기반 시스템으로 일관성 보장
3. 소프트웨어 라디안 ↔ 하드웨어 라디안 자동 변환
4. 깔끔한 라디안 전용 API
"""

import numpy as np
import time
from dynamixel_sdk.controllerAX import ControllerAX
from dynamixel_sdk.controllerXC import ControllerXC

class RealMotorController:
    """실제 모터 하드웨어 추상화 컨트롤러 - 단일 라디안 기반"""
    
    def __init__(self, port_handler, baudrate, motor_info, motor_type='AX', safe_init=True, hw_initial_rad=0.0, sw_initial_rad=0.0):
        """
        실제 모터 초기화 및 캘리브레이션
        Args:
            port_handler: DynamixelSDK 포트 핸들러
            baudrate: 통신 속도
            motor_info: 모터 설정 정보 (id, speed 등)
            motor_type: 'AX' 또는 'XC'
            safe_init: True면 현재 위치 기준 캘리브레이션, False면 150도로 이동
            hw_initial_rad: 하드웨어 초기 각도 (라디안)
            sw_initial_rad: 소프트웨어 초기 각도 (라디안)
        """
        self.motor_id = motor_info['id']
        self.motor_type = motor_type
        self.port_handler = port_handler
        self.baudrate = baudrate
        self.safe_init = safe_init
        self.sw_initial_rad = sw_initial_rad  # sw_initial_rad 저장
        
        # 소프트웨어-하드웨어 각도 오프셋 계산
        self.angle_offset_rad = hw_initial_rad - sw_initial_rad
        
        # print(f"🔧 Motor {self.motor_id} 오프셋: HW={np.rad2deg(hw_initial_rad):.1f}° - SW={np.rad2deg(sw_initial_rad):.1f}° = {np.rad2deg(self.angle_offset_rad):.1f}°")
        
        # 실제 하드웨어 컨트롤러 생성
        if motor_type == 'AX':
            self.hw_controller = ControllerAX(
                port_handler, 
                baudrate,
                motor_info['id']
            )
        else:  # XC
            self.hw_controller = ControllerXC(
                port_handler, 
                baudrate,
                motor_info['id']
            )
        
        # 모터 활성화
        self.hw_controller.torque_enable()
        time.sleep(0.1)  # 안정화 대기
        
        # 초기 캘리브레이션 수행
        if safe_init:
            self._perform_safe_calibration()
        else:
            self._perform_initial_calibration()
        
        # print(f"✅ Real Motor Controller initialized: ID={self.motor_id}, Type={motor_type}")
    
    ######## Core Radian-Based System ########
    
    def _software_rad_to_hardware_rad(self, software_rad):
        """소프트웨어 라디안을 하드웨어 라디안으로 변환"""
        return software_rad + self.angle_offset_rad
    
    def _hardware_rad_to_software_rad(self, hardware_rad):
        """하드웨어 라디안을 소프트웨어 라디안으로 변환"""
        return hardware_rad - self.angle_offset_rad
    
    def _rad_to_position(self, radians):
        """라디안을 모터 위치값으로 변환"""
        degrees = np.rad2deg(radians)
        if self.motor_type == 'AX':
            return int(degrees * 1023.0 / 300.0)
        else:  # XC
            return int(degrees * 4095.0 / 360.0)
    
    def _position_to_rad(self, position):
        """모터 위치값을 라디안으로 변환"""
        if self.motor_type == 'AX':
            degrees = position * 300.0 / 1023.0
        else:  # XC
            degrees = position * 360.0 / 4095.0
        return np.deg2rad(degrees)
    
    ######## Primary API - Radian Based ########
    
    def move_to_software_radian(self, software_rad):
        """소프트웨어 라디안 각도로 모터 이동 - 주요 API"""
        # 1. 소프트웨어 라디안 → 하드웨어 라디안
        hardware_rad = self._software_rad_to_hardware_rad(software_rad)
        
        # 2. 하드웨어 라디안 → 모터 위치값
        hardware_position = self._rad_to_position(hardware_rad)
        
        try:
            # 3. 실제 하드웨어로 명령 전송
            self.hw_controller.move(hardware_position)
            
            # 4. 현재 소프트웨어 위치 업데이트
            self.current_software_rad = software_rad
            
            # print(f"🎯 Motor {self.motor_id}: SW={np.rad2deg(software_rad):.1f}° → HW={np.rad2deg(hardware_rad):.1f}° → Pos={hardware_position}")
            
            # 명령 완료 대기
            time.sleep(0.01)
            
        except Exception as e:
            print(f"❌ Motor {self.motor_id} 이동 오류: {e}")
    
    def read_current_hardware_radian(self):
        """현재 하드웨어 라디안 각도 직접 읽기 - 오프셋 적용 없음"""
        try:
            # 1. 하드웨어 위치 직접 읽기
            hardware_position = self.hw_controller.read_current_position()
            
            # 2. 모터 위치값 → 하드웨어 라디안 (오프셋 적용 안함)
            hardware_rad = self._position_to_rad(hardware_position)
            
            return hardware_rad
            
        except Exception as e:
            print(f"❌ Motor {self.motor_id} 하드웨어 위치 읽기 오류: {e}")
            return 0.0

    def read_current_software_radian(self):
        """현재 소프트웨어 라디안 각도 읽기 - 주요 API"""
        try:
            # 1. 하드웨어 라디안 직접 읽기
            hardware_rad = self.read_current_hardware_radian()
            
            # 2. 하드웨어 라디안 → 소프트웨어 라디안
            software_rad = self._hardware_rad_to_software_rad(hardware_rad)
            
            # 3. 현재 소프트웨어 위치 업데이트
            self.current_software_rad = software_rad
            
            return software_rad
            
        except Exception as e:
            print(f"❌ Motor {self.motor_id} 위치 읽기 오류: {e}")
            return getattr(self, 'current_software_rad', 0.0)
    
    ######## Calibration System ########
    
    def _perform_safe_calibration(self):
        """안전한 캘리브레이션: 현재 위치를 기준점으로 설정 (이동 없음)"""
        print(f"🔧 Motor {self.motor_id} 안전 캘리브레이션 시작...")
        
        # 1. 현재 실제 하드웨어 위치 읽기
        current_hw_position = self.hw_controller.read_current_position()
        current_hw_rad = self._position_to_rad(current_hw_position)
        current_hw_degrees = np.rad2deg(current_hw_rad)
        
        print(f"   현재 하드웨어 위치: {current_hw_position} (≈ {current_hw_degrees:.1f}도, {current_hw_rad:.3f}rad)")
        
        # 2. 현재 위치를 소프트웨어 0 라디안 기준점으로 설정
        self.current_software_rad = 0.0  # 소프트웨어상 0 라디안 위치
        self.calibration_hw_rad = current_hw_rad  # 캘리브레이션 기준 하드웨어 각도
        
        print(f"   ✅ 안전 캘리브레이션 완료: HW={current_hw_rad:.3f}rad = SW=0.0rad")
        print(f"   ⚠️ 모터가 현재 위치에서 캘리브레이션됨 (이동 없음)")
    
    def _perform_initial_calibration(self):
        """기존 캘리브레이션: 150도 위치로 이동 (안전하지 않음)"""
        print(f"🔧 Motor {self.motor_id} 캘리브레이션 시작...")
        
        # 1. 현재 실제 하드웨어 위치 읽기
        current_hw_position = self.hw_controller.read_current_position()
        current_hw_rad = self._position_to_rad(current_hw_position)
        current_hw_degrees = np.rad2deg(current_hw_rad)
        
        print(f"   현재 하드웨어 위치: {current_hw_position} (≈ {current_hw_degrees:.1f}도)")
        
        # 2. 150도 위치로 천천히 이동
        target_degrees = 150.0
        target_rad = np.deg2rad(target_degrees)
        target_position = self._rad_to_position(target_rad)
        
        print(f"   150도로 캘리브레이션 중... (목표: {target_rad:.3f}rad, 위치: {target_position})")
        
        # 천천히 이동하기 위해 단계별 이동
        steps = 10
        step_size = (target_position - current_hw_position) // steps
        
        for i in range(steps):
            intermediate_pos = current_hw_position + step_size * (i + 1)
            self.hw_controller.move(intermediate_pos)
            time.sleep(0.3)  # 각 단계마다 0.3초 대기
        
        # 최종 목표 위치로 이동
        self.hw_controller.move(target_position)
        time.sleep(1.0)  # 이동 완료 대기
        
        # 3. 이동 후 실제 위치 확인
        final_hw_position = self.hw_controller.read_current_position()
        final_hw_rad = self._position_to_rad(final_hw_position)
        final_hw_degrees = np.rad2deg(final_hw_rad)
        
        print(f"   캘리브레이션 완료: {final_hw_position} (≈ {final_hw_degrees:.1f}도, {final_hw_rad:.3f}rad)")
        
        # 4. 이 위치를 소프트웨어 0 라디안 기준점으로 설정
        self.current_software_rad = 0.0
        self.calibration_hw_rad = final_hw_rad
        
        print(f"   소프트웨어 기준점 설정: HW={final_hw_rad:.3f}rad = SW=0.0rad")
    
    ######## Motor Control Functions ########
    
    def torque_enable(self):
        """토크 활성화"""
        self.hw_controller.torque_enable()
        print(f"🔧 Motor {self.motor_id} torque enabled")
    
    def torque_disable(self):
        """토크 비활성화"""
        if hasattr(self.hw_controller, 'torque_disable'):
            self.hw_controller.torque_disable()
        print(f"🔧 Motor {self.motor_id} torque disabled")
    
    def set_moving_speed(self, speed_value):
        """이동 속도 설정"""
        if hasattr(self.hw_controller, 'set_moving_speed'):
            self.hw_controller.set_moving_speed(speed_value)
        print(f"⚡ Motor {self.motor_id} speed set to {speed_value}")
    
    def set_position_p_gain(self, gain_value):
        """위치 P 게인 설정"""
        if hasattr(self.hw_controller, 'set_position_p_gain'):
            self.hw_controller.set_position_p_gain(gain_value)
        print(f"🎛️ Motor {self.motor_id} P gain set to {gain_value}")
    
    ######## Information and Debugging ########
    
    def get_calibration_info(self):
        """캘리브레이션 정보 반환"""
        calibration_type = "안전 캘리브레이션" if self.safe_init else "150도 캘리브레이션"
        
        # 현재 상태 정보
        current_hw_rad = getattr(self, 'calibration_hw_rad', 0.0)
        current_sw_rad = getattr(self, 'current_software_rad', 0.0)
        
        return {
            'motor_id': self.motor_id,
            'motor_type': self.motor_type,
            'calibration_type': calibration_type,
            'calibration_hw_rad': current_hw_rad,
            'calibration_degrees': np.rad2deg(current_hw_rad),
            'current_software_rad': current_sw_rad,
            'angle_offset_rad': self.angle_offset_rad
        }
    
    def get_status_info(self):
        """현재 상태 정보 반환 (디버깅용)"""
        current_sw_rad = self.read_current_software_radian()
        current_hw_rad = self._software_rad_to_hardware_rad(current_sw_rad)
        
        return {
            'motor_id': self.motor_id,
            'motor_type': self.motor_type,
            'current_software_rad': current_sw_rad,
            'current_software_deg': np.rad2deg(current_sw_rad),
            'current_hardware_rad': current_hw_rad,
            'current_hardware_deg': np.rad2deg(current_hw_rad),
            'angle_offset_rad': self.angle_offset_rad,
            'angle_offset_deg': np.rad2deg(self.angle_offset_rad)
        } 