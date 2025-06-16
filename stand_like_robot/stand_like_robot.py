"""
4 DoF 로봇 제어 클래스

핵심 기능:
- 모터 초기화 및 제어
- 순운동학/역운동학 계산
- 부드러운 궤적 계획 및 실행

좌표계:
- 직교좌표: x, y, z (cm 단위)
- 관절좌표: joint 1-4 (라디안 단위)
"""
# joint_angles:  [ 0.          1.04719755 -1.04719755  1.57079633]
# joint_angles:  [ 0.31099828  1.1752937  -1.1752937   1.57079633]
# joint_angles:  [ 0.31099828  1.1752937  -1.1752937   1.57079633]

import os
import yaml
import time
import numpy as np
import sys

# Add parent directory to path for imports
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from dynamixel_sdk.controllerXC import *
from dynamixel_sdk.controllerAX import *
from stand_like_robot.kinematic_solver import *
from motor_controller.mock_motor_controller import MockMotorController
from motor_controller.real_motor_controller import RealMotorController
from stand_like_robot.trajectory_planner import TrajectoryPlanner

class StandLikeRobot:
    ######## Initialization ########
    def __init__(
        self, 
        metadata_path=None,
        simulation_mode=False,
        port_address='/dev/cu.usbserial-FT8ISMU2',
        dual_mode=False  # 새로운 옵션: 시뮬레이션 + 실제 로봇 동시 사용
    ):
        """로봇 초기화"""
        self.simulation_mode = simulation_mode
        self.dual_mode = dual_mode  # 시뮬레이션과 실제 로봇 동시 사용 여부
        self.port_handler = PortHandler(port_address) if (not simulation_mode or dual_mode) else None
        
        # 기본 메타데이터 경로 설정
        if metadata_path is None:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            metadata_path = os.path.join(current_dir, 'config', 'stand_like_robot.yaml')
        
        if dual_mode:
            print("🔄 DUAL MODE: 시뮬레이션 + 실제 로봇 동시 구동")
        elif simulation_mode:
            print("🎮 SIMULATION MODE: 시뮬레이션만")
        else:
            print("🤖 HARDWARE MODE: 실제 로봇만")
        
        # 메타데이터 로드
        try:
            with open(metadata_path, 'r') as file:
                self.metadata = yaml.safe_load(file)
            arm_motor_count = len(self.metadata['arm_motors'])
            gripper_motor_count = len(self.metadata['gripper_motors'])
            print(f"메타데이터 로드 완료: 팔 모터 {arm_motor_count}개, 그리퍼 모터 {gripper_motor_count}개")
        except Exception as e:
            raise ValueError(f"YAML 파일 오류: {e}")
        
        # 팔 모터 초기값 및 제한값 추출
        self.hw_init_joint_radians = [] # 이 데이터는 real_motor_controller에서 사용
        self.sw_init_joint_radians = [] # 이 데이터는 시뮬레이션 모드와 내부 계산에서 사용
        self.joint_limits_rad = [] # 이 데이터는 시뮬레이션 모드와 내부 계산에서 사용
        
        for i, motor in enumerate(self.metadata['arm_motors']):
            motor_name = list(motor.keys())[0] # id
            motor_config = motor[motor_name]
            
            # hw_initial_degree를 라디안으로 변환
            hw_initial_rad = np.deg2rad(motor_config['hw_initial_degree'])
            self.hw_init_joint_radians.append(hw_initial_rad)
            self.sw_init_joint_radians.append(motor_config['sw_initial_rad'])
            
            limit_min = motor_config.get('sw_limit_rad_min', -np.pi)
            limit_max = motor_config.get('sw_limit_rad_max', np.pi)
            self.joint_limits_rad.append((limit_min, limit_max))
        
        # 그리퍼 모터 초기값
        self.hw_gripper_init_radians = []
        self.sw_gripper_init_radians = []
        self.gripper_limits_rad = []
        
        for motor in self.metadata['gripper_motors']:
            motor_name = list(motor.keys())[0]
            motor_config = motor[motor_name]
            
            # hw_initial_degree를 라디안으로 변환
            hw_initial_rad = np.deg2rad(motor_config['hw_initial_degree'])
            self.hw_gripper_init_radians.append(hw_initial_rad)
            self.sw_gripper_init_radians.append(motor_config['sw_initial_rad'])
            
            limit_min = motor_config.get('sw_limit_rad_min', -np.pi)
            limit_max = motor_config.get('sw_limit_rad_max', np.pi)
            self.gripper_limits_rad.append((limit_min, limit_max))
        
        # 운동학 해결기 초기화
        self.kinematic_solver = KinematicSolver()
        
        # 궤적 계획기 초기화
        self.trajectory_planner = TrajectoryPlanner(time_step=0.02)
        
        # YAML에서 최대 속도 설정 읽기
        ax_max_velocity = self.metadata['general_settings']['motor_specs']['AX']['max_velocity']
        xc_max_velocity = self.metadata['general_settings']['motor_specs']['XC']['max_velocity']
        
        # 관절별 최대 각속도 설정 (YAML 기반, rad/s 단위로 변환)
        # AX 모터의 max_velocity를 rad/s로 변환: RPM * 2π / 60
        ax_max_rad_per_sec = ax_max_velocity * 2 * np.pi / 60
        xc_max_rad_per_sec = xc_max_velocity * 2 * np.pi / 60
        
        self.max_joint_velocities = np.array([
            ax_max_rad_per_sec,  # Joint 0 (베이스): AX 모터
            ax_max_rad_per_sec,  # Joint 1 (어깨): AX 모터  
            ax_max_rad_per_sec,  # Joint 2 (팔꿈치): AX 모터
            ax_max_rad_per_sec   # Joint 3 (손목): AX 모터
        ])
        
        print(f"✅ 궤적 계획기 초기화 완료")
        print(f"   AX 모터 최대속도: {ax_max_velocity} RPM = {ax_max_rad_per_sec:.2f} rad/s")
        print(f"   XC 모터 최대속도: {xc_max_velocity} RPM = {xc_max_rad_per_sec:.2f} rad/s")
        print(f"   관절별 최대 각속도: {self.max_joint_velocities} rad/s")
        
        # 하드웨어/시뮬레이션 모드 초기화
        if dual_mode:
            self._init_dual_mode()  # 시뮬레이션 + 실제 로봇 동시
        elif simulation_mode:
            self._init_simulation_mode()
        else:
            self._init_hardware_mode()
        
        # 초기 위치로 이동
        self.move_to_initial_position()
        print("✅ 로봇 초기화 완료")

    def _init_simulation_mode(self):
        """시뮬레이션 모드 초기화"""
        if not hasattr(self, 'dual_mode') or not self.dual_mode:
            print("🤖 시뮬레이션 모드 초기화...")
        
        self.arm_controllers, self.gripper_controllers = self._create_mock_controllers(self.metadata)
        
        if not hasattr(self, 'dual_mode') or not self.dual_mode:
            print(f"시뮬레이션 초기화 완료: 팔 모터 {len(self.arm_controllers)}개, 그리퍼 모터 {len(self.gripper_controllers)}개")

    def _init_hardware_mode(self):
        """하드웨어 모드 초기화"""
        if not hasattr(self, 'dual_mode') or not self.dual_mode:
            print("🔌 하드웨어 모드 초기화...")
        
        self._init_port()
        
        self.arm_controllers = self._create_real_controllers(
            self.metadata['arm_motors'], 'AX', 
            self.hw_init_joint_radians, self.sw_init_joint_radians
        )
        self.gripper_controllers = self._create_real_controllers(
            self.metadata['gripper_motors'], 'XC',
            self.hw_gripper_init_radians, self.sw_gripper_init_radians
        )
        
        if not hasattr(self, 'dual_mode') or not self.dual_mode:
            print(f"하드웨어 초기화 완료: 팔 모터 {len(self.arm_controllers)}개, 그리퍼 모터 {len(self.gripper_controllers)}개")
            self._print_calibration_info(self.arm_controllers, self.gripper_controllers)

    def _init_dual_mode(self):
        """시뮬레이션과 실제 로봇 동시 모드 초기화"""
        print("🔄 DUAL MODE 초기화...")
        
        # 1단계: 시뮬레이션 모드 초기화 및 백업
        print("   📱 시뮬레이션 컨트롤러 초기화...")
        self._init_simulation_mode()
        
        # Mock 컨트롤러들을 미리 백업 (덮어쓰기 방지)
        mock_arm_controllers = self.arm_controllers.copy()
        mock_gripper_controllers = self.gripper_controllers.copy()
        print(f"   ✅ 시뮬레이션: 팔 모터 {len(mock_arm_controllers)}개, 그리퍼 모터 {len(mock_gripper_controllers)}개")
        
        # 2단계: 실제 하드웨어 초기화 시도
        print("   🔧 실제 하드웨어 컨트롤러 초기화...")
        try:
            # 하드웨어 초기화 (self.arm_controllers, self.gripper_controllers 덮어쓰기됨)
            self._init_hardware_mode()
            
            # 하드웨어 컨트롤러들을 별도 변수로 저장
            self.real_arm_controllers = self.arm_controllers
            self.real_gripper_controllers = self.gripper_controllers
            
            # 시뮬레이션 컨트롤러들을 원래 변수로 복원
            self.arm_controllers = mock_arm_controllers
            self.gripper_controllers = mock_gripper_controllers
            
            print(f"   ✅ 실제 하드웨어: 팔 모터 {len(self.real_arm_controllers)}개, 그리퍼 모터 {len(self.real_gripper_controllers)}개")
            
            self.hardware_available = True
            print("   🎉 하드웨어 초기화 성공!")
            
        except Exception as e:
            print(f"   ⚠️ 실제 하드웨어 초기화 실패: {e}")
            print(f"   📱 시뮬레이션 모드로만 동작합니다")
            
            # 하드웨어 실패 시 시뮬레이션 컨트롤러들 복원
            self.arm_controllers = mock_arm_controllers
            self.gripper_controllers = mock_gripper_controllers
            self.real_arm_controllers = {}
            self.real_gripper_controllers = {}
            self.hardware_available = False
        
        print("🔄 DUAL MODE 초기화 완료!")
    
    def _create_mock_controllers(self, motor_configs):
        """Mock 모터 컨트롤러들 생성"""
        arm_controllers = {}
        gripper_controllers = {}
        
        # 팔 모터 생성
        for i, motor in enumerate(motor_configs['arm_motors']):
            motor_name = list(motor.keys())[0]
            motor_info = motor[motor_name]
            arm_controllers[motor_name] = MockMotorController(
                motor_id=motor_info['id'],
                initial_position=motor_info['hw_initial_position'],
                initial_software_rad=self.sw_init_joint_radians[i]
            )
        
        # 그리퍼 모터 생성
        for i, motor in enumerate(motor_configs['gripper_motors']):
            motor_name = list(motor.keys())[0]
            motor_info = motor[motor_name]
            gripper_controllers[motor_name] = MockMotorController(
                motor_id=motor_info['id'],
                initial_position=motor_info['hw_initial_position'],
                initial_software_rad=self.sw_gripper_init_radians[i]
            )
        
        return arm_controllers, gripper_controllers
    
    def _create_real_controllers(self, motor_configs, motor_type, hw_initial_rads, sw_initial_rads):
        """Real 모터 컨트롤러들 생성"""
        controllers = {}
        for i, motor in enumerate(motor_configs):
            motor_name = list(motor.keys())[0]
            motor_info = motor[motor_name]
            
            controller = RealMotorController(
                self.port_handler, 
                1000000,  # baudrate 추가
                motor_info, 
                motor_type=motor_type,
                safe_init=True,
                hw_initial_rad=hw_initial_rads[i],
                sw_initial_rad=sw_initial_rads[i]
            )
            controllers[motor_name] = controller
        return controllers
    
    def _init_port(self):
        """포트 초기화"""
        if not self.port_handler.openPort():
            raise Exception("포트 열기 실패")
        if not self.port_handler.setBaudRate(115200):
            raise Exception("보드레이트 설정 실패")
    
    def _print_calibration_info(self, arm_controllers, gripper_controllers, prefix=""):
        """캘리브레이션 정보 출력"""
        print(f"\n📊 {prefix}모터 캘리브레이션 정보:")
        for motor_name, controller in arm_controllers.items():
            info = controller.get_calibration_info()
            print(f"   {motor_name}: {info['calibration_type']}, HW기준점={info['software_zero_hw_position']}, 각도={info['calibration_degrees']:.1f}도")
        
        for motor_name, controller in gripper_controllers.items():
            info = controller.get_calibration_info()
            print(f"   {motor_name}: {info['calibration_type']}, HW기준점={info['software_zero_hw_position']}, 각도={info['calibration_degrees']:.1f}도")

    ######## Arm Movement ########
    # 1-1. Inverse Kinematicss
    def move_to_cartesian_position(self, target_position, time_to_go=2.0, respect_hw_limits=True):
        """직교좌표 위치로 이동 (팔만)"""
        try:
            # KinematicSolver의 역운동학 사용
            solution_radians = self.kinematic_solver.inverse_kinematics(target_position)
            
            if solution_radians is not None:
                # 관절 제한 확인 (옵션)
                if respect_hw_limits:
                    violations = self.kinematic_solver.check_joint_limits(solution_radians, self.joint_limits_rad)
                    if violations:
                        print(f"⚠️ 관절 제한 위반: {violations}")
                        print("💡 제한 무시하고 실행하려면 respect_hw_limits=False 사용")
                        return
                
                # 관절 각도로 이동
                self.move_arm_to_joint_radians(solution_radians, time_to_go)
                print("✅ 직교좌표 이동 완료")
            else:
                print("❌ 역운동학 해가 없습니다 (도달 불가능한 위치)")
            
        except Exception as e:
            print(f"❌ 직교좌표 이동 오류: {e}")
    
    # 1-2. Initial Position
    def move_to_initial_position(self):
        """초기 위치로 이동 (팔만) - 천천히 이동"""
        print("🏠 초기 위치로 이동 중...")
        
        # 안전한 속도로 초기 위치로 이동 (3초에 걸쳐)
        safe_time = 1.5
        
        degrees = [np.rad2deg(rad) for rad in self.sw_init_joint_radians]
        print(f"   목표 초기 위치: {degrees} 도")
        print(f"   {safe_time}초에 걸쳐 안전하게 이동합니다...")
        
        # 초기 위치로 이동 실행
        self.move_arm_to_joint_radians(self.sw_init_joint_radians, time_to_go=safe_time)
        
        print(f"✅ 초기 위치 설정 완료: {degrees} 도")

    # 2. Joint Movement
    def move_arm_to_joint_radians(self, target_joint_radians, time_to_go=2.0):
        """
        팔 관절을 부드러운 궤적으로 이동 (소프트웨어 라디안 입력)
        
        Args:
            target_joint_radians: 목표 관절 각도 (라디안)
            time_to_go: 이동 시간 (초)
        """        
        # 현재 관절 각도 읽기
        current_joint_radians = self.get_current_arm_joint_radians()
        
        print(f"🛤️ 부드러운 팔 이동:")
        print(f"   시작: {np.rad2deg(current_joint_radians)} 도")
        print(f"   목표: {np.rad2deg(target_joint_radians)} 도")
        print(f"   시간: {time_to_go:.1f}초")
        
        # 관절 제한 확인
        violations = self.kinematic_solver.check_joint_limits(target_joint_radians, self.joint_limits_rad)
        if violations:
            print(f"⚠️ 목표 위치 관절 제한 위반: {violations}")
        
        # 궤적 계획 수행
        try:
            trajectory_result = self.trajectory_planner.plan(
                start_joint_radians=current_joint_radians,
                end_joint_radians=target_joint_radians,
                max_velocities=self.max_joint_velocities,
                time_to_go=time_to_go
            )
            
            # 궤적 실행
            self._execute_trajectory(trajectory_result)
            
        except Exception as e:
            print(f"❌ 궤적 계획 오류: {e}")
    
    def _execute_trajectory(self, trajectory_result):
        """계획된 궤적을 실행 - 순차 실행"""
        joint_trajectories = trajectory_result['radians']
        time_points = trajectory_result['time_points']
        
        print(f"🎬 궤적 실행: {len(time_points)}개 포인트, {time_points[-1]:.2f}초")
        
        start_time = time.time()
        
        # 안전한 실행을 위해 포인트 간격 조절
        min_interval = self.trajectory_planner.time_step  # 0.02 s
        
        
        for i, joint_angles in enumerate(joint_trajectories):
            # 실제 시간과 동기화
            target_time = time_points[i]
            current_time = time.time() - start_time
            
            # 최소 간격 보장
            if i > 0:
                elapsed_since_last = current_time - time_points[i-1]
                # print('elapsed_since_last', elapsed_since_last)
                if elapsed_since_last < min_interval:
                    time.sleep(min_interval - elapsed_since_last)
            
            if current_time < target_time:
                # print('target_time - current_time', target_time - current_time)
                time.sleep(target_time - current_time)
            
            # 각 모터로 명령 전송 (순차 동기화)
            self._control_motors(joint_angles)
            
            # 주기적 상태 출력 (20포인트마다)
            if (i + 1) % 20 == 0:
                progress = (i + 1) / len(joint_trajectories) * 100
                print(f"  📊 진행률: {progress:.1f}% ({i+1}/{len(joint_trajectories)})")
        
        print(f"✅ 궤적 실행 완료")
    
    def _control_motors(self, joint_radians):
        """통합된 모터 제어 메서드 - 비동기 실행으로 모든 모터 동시 제어"""
        # print(f"🎯 모터 제어: {len(joint_radians)}개 모터 동시 실행 (소프트웨어 라디안)")
        
        # 안정화 딜레이
        stabilization_delay = 0.0
        
        # 모든 모터에 동시에 명령 전송
        motor_commands = []
        for i, (motor_name, controller) in enumerate(self.arm_controllers.items()):
            if i >= len(joint_radians):
                break
                
            software_rad = joint_radians[i]
            # print(f"🎯 Motor {i} ({motor_name}): SW={np.rad2deg(software_rad):.1f}°")
            
            # 명령 준비 (실행은 나중에)
            motor_commands.append((motor_name, controller, software_rad))
        
        # 모든 모터에 동시 명령 전송
        # print(f"⚡ {len(motor_commands)}개 모터 동시 실행 시작...")
        
        for motor_name, controller, software_rad in motor_commands:
            try:
                self._send_single_motor_command(motor_name, controller, software_rad)
            except Exception as e:
                print(f"❌ {motor_name} 오류: {e}")
        
        # 모든 명령 전송 후 안정화 대기
        time.sleep(stabilization_delay)
        # print(f"✅ 🎯 모터 제어 완료 (동시 실행)")
    
    def _send_single_motor_command(self, motor_name, controller, software_rad):
        """단일 모터에 명령 전송 - 완전한 라디안 기반"""
        if self.simulation_mode and not self.dual_mode:
            # 시뮬레이션만 - 완전한 라디안 기반
            controller.move_to_software_radian(software_rad)
            
        elif self.dual_mode:
            # DUAL MODE: Mock + Real (모두 라디안 기반)
            controller.move_to_software_radian(software_rad)
            
            if self.hardware_available and motor_name in self.real_arm_controllers:
                real_controller = self.real_arm_controllers[motor_name]
                try:
                    real_controller.move_to_software_radian(software_rad)
                    # print(f"🔄 DUAL: {motor_name} → Mock+Real 완료")
                except Exception as e:
                    print(f"⚠️ {motor_name} 실제 모터 오류: {e}")
                    
        else:
            # 실제 모터만 - 라디안 기반
            controller.move_to_software_radian(software_rad)

    ######## Arm Position Reading ########

    def get_current_arm_joint_degrees(self):
        """현재 팔 관절 각도 (도 단위)"""
        radians = self.get_current_arm_joint_radians()
        return [np.rad2deg(rad) for rad in radians]

    def get_current_arm_joint_radians(self):
        """현재 팔 관절 각도 (라디안 단위) - 통합된 읽기"""
        radians = []
        
        for motor_name, controller in self.arm_controllers.items():
            radian = self._read_single_arm_position(motor_name, controller)
            radians.append(radian)
        
        return radians
    
    def _read_single_arm_position(self, motor_name, controller):
        """단일 팔 모터 위치 읽기 - 하드웨어 직접 읽기 방식"""
        try:
            # 하드웨어 포지션을 직접 읽어서 오프셋 적용
            # 이렇게 하면 초기화 시 순환 참조 문제 방지
            if hasattr(controller, 'read_current_hardware_radian'):
                # 실제 모터: 하드웨어 라디안 직접 읽기
                hardware_rad = controller.read_current_hardware_radian()
                software_rad = controller._hardware_rad_to_software_rad(hardware_rad)
                return software_rad
            else:
                # 모의 모터: 기존 방식 사용
                return controller.read_current_software_radian()
        except Exception as e:
            print(f"⚠️ {motor_name} 위치 읽기 오류: {e}")
            return 0.0

    def get_current_cartesian_position(self):
        """현재 직교좌표 위치 (팔만)"""
        arm_radians = self.get_current_arm_joint_radians()
        return self.kinematic_solver.forward_kinematics(arm_radians)

    ######## End Effector Control ########
    def set_end_effector_orientation(self, z_rotation_deg, time_to_go=1.0):
        """엔드이펙터의 Z축 회전 각도 설정 (도 단위)"""
        print(f"🔄 엔드이펙터 Z축 회전: {z_rotation_deg}도")
        
        # 현재 팔 관절 각도 가져오기 (0~2번 관절)
        current_arm_joints = self.get_current_arm_joint_radians()[:3]
        
        # 4번째 관절(AX3)의 목표 각도 설정
        z_rotation_rad = np.deg2rad(z_rotation_deg)
        
        # 전체 관절 각도 조합 (0~3번 관절)
        full_joint_angles = list(current_arm_joints) + [z_rotation_rad]
        
        # 4개 관절 모두 이동
        self.move_arm_to_joint_radians(full_joint_angles, time_to_go)

    def rotate_end_effector(self, delta_rotation_deg, time_to_go=1.0):
        """엔드이펙터를 현재 위치에서 상대적으로 회전"""
        current_joints = self.get_current_arm_joint_radians()
        
        if len(current_joints) >= 4:
            current_z_rotation_deg = np.rad2deg(current_joints[3])
            new_z_rotation_deg = current_z_rotation_deg + delta_rotation_deg
            print(f"🔄 상대 회전: {current_z_rotation_deg}° → {new_z_rotation_deg}°")
            self.set_end_effector_orientation(new_z_rotation_deg, time_to_go)
        else:
            print("⚠️ 4번째 관절(orientation) 정보 없음")

    def get_current_end_effector_orientation(self):
        """현재 엔드이펙터의 Z축 회전 각도 반환 (도 단위)"""
        current_joints = self.get_current_arm_joint_radians()
        if len(current_joints) >= 4:
            return np.rad2deg(current_joints[3])
        else:
            return 0.0

    def move_to_position_with_orientation(self, target_position, z_rotation_deg=0, time_to_go=2.0):
        """위치와 orientation을 함께 제어하는 통합 함수"""
        print(f"🎯 목표: 위치{target_position}, 회전{z_rotation_deg}°")
        
        # 역운동학으로 0~2번 관절 계산
        try:
            # 위치만으로 역운동학 계산 (0~2번 관절)
            solution_joints_3dof = self.kinematic_solver.inverse_kinematics(target_position)
            
            if solution_joints_3dof is not None:
                # 4번째 관절(orientation) 추가
                z_rotation_rad = np.deg2rad(z_rotation_deg)
                full_solution = list(solution_joints_3dof[:3]) + [z_rotation_rad]
                
                # 4개 관절 모두 이동
                self.move_arm_to_joint_radians(full_solution, time_to_go)
                print(f"✅ 위치+회전 이동 완료")
            else:
                print("❌ 해당 위치에 대한 역운동학 해가 없습니다")
                
        except Exception as e:
            print(f"❌ 위치+회전 이동 오류: {e}")


    ######## Gripper Control ########
    def open_gripper(self, time_to_go=1.0):
        """그리퍼 열기"""
        print("🖐️ 그리퍼 열기...")
        open_radians = [np.radians(100), np.radians(140)]
        self.move_gripper_to_radians(open_radians, time_to_go)

    def close_gripper(self, time_to_go=1.0):
        """그리퍼 닫기"""
        print("✊ 그리퍼 닫기...")
        close_radians = [np.radians(125), np.radians(115)]
        self.move_gripper_to_radians(close_radians, time_to_go)

    def move_gripper_to_radians(self, gripper_radians, time_to_go=1.0):
        """그리퍼를 특정 각도로 이동 - 완전한 라디안 기반"""
        if len(gripper_radians) != len(self.gripper_controllers):
            raise ValueError(f"그리퍼 관절 수 불일치: {len(gripper_radians)} != {len(self.gripper_controllers)}")
        
        print(f"🤏 그리퍼 제어: {len(gripper_radians)}개 모터 (완전한 라디안 기반)")
        
        for i, (motor_name, controller) in enumerate(self.gripper_controllers.items()):
            gripper_rad = gripper_radians[i]
            self._control_single_gripper(motor_name, controller, gripper_rad, time_to_go)
        
        # 최종 대기
        if not self.simulation_mode or (self.dual_mode and self.hardware_available):
            time.sleep(time_to_go)
    
    def _control_single_gripper(self, motor_name, controller, gripper_rad, time_to_go):
        """단일 그리퍼 모터 제어 - 완전한 라디안 기반"""
        try:
            if self.simulation_mode and not self.dual_mode:
                # 시뮬레이션만 - 완전한 라디안 기반
                controller.move_to_software_radian(gripper_rad)
                
            elif self.dual_mode:
                # DUAL MODE: Mock + Real (모두 라디안 기반)
                controller.move_to_software_radian(gripper_rad)
                
                if self.hardware_available and motor_name in self.real_gripper_controllers:
                    real_controller = self.real_gripper_controllers[motor_name]
                    try:
                        real_controller.move_to_software_radian(gripper_rad)
                        print(f"🔄 그리퍼: {motor_name} → Mock+Real 완료 ({np.rad2deg(gripper_rad):.1f}°)")
                    except Exception as e:
                        print(f"⚠️ {motor_name} 실제 그리퍼 제어 오류: {e}")
            else:
                # 실제 모터만 - 완전한 라디안 기반
                controller.move_to_software_radian(gripper_rad)
                
        except Exception as e:
            print(f"❌ {motor_name} 그리퍼 제어 오류: {e}")

    def get_current_gripper_joint_radians(self):
        """현재 그리퍼 관절 각도 (라디안 단위) - 완전한 라디안 기반"""
        radians = []
        
        for motor_name, controller in self.gripper_controllers.items():
            radian = self._read_single_gripper_position(motor_name, controller)
            radians.append(radian)
        
        return radians
    
    def _read_single_gripper_position(self, motor_name, controller):
        """단일 그리퍼 위치 읽기 - 완전한 라디안 기반"""
        try:
            return controller.read_current_software_radian()
        except Exception as e:
            print(f"⚠️ {motor_name} 그리퍼 위치 읽기 오류: {e}")
            return 0.0

    def get_gripper_status(self):
        """그리퍼 상태 정보 반환"""
        current_gripper_radians = self.get_current_gripper_joint_radians()
        
        if len(current_gripper_radians) >= 2:
            # 두 핑거의 평균 각도 계산
            avg_angle_deg = np.rad2deg(np.mean(current_gripper_radians[:2]))
            
            # 120도(닫힘) ~ 180도(열림) 범위를 0~100%로 변환
            closed_angle = 120
            open_angle = 180
            
            opening_pct = (avg_angle_deg - closed_angle) / (open_angle - closed_angle) * 100
            opening_pct = max(0, min(100, opening_pct))  # 0~100% 범위 제한
        else:
            opening_pct = 0.0
        
        if opening_pct > 80:
            status = "열림"
        elif opening_pct < 20:
            status = "닫힘"
        else:
            status = "부분열림"
        
        return {
            'opening_percentage': opening_pct,
            'status': status,
            'angles_deg': np.rad2deg(current_gripper_radians).tolist()
        }

if __name__ == "__main__":
    # 간단한 테스트
    robot = StandLikeRobot(simulation_mode=True)
    print("✅ 로봇 클래스 테스트 완료")
    print("🚀 시뮬레이션을 시작하려면 'python simple_robot_demo.py'를 실행하세요")    