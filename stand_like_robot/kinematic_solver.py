"""
간단한 기하학적 운동학 해결기

4-DOF 로봇팔을 위한 직관적인 기하학 계산:
- Joint 0: 베이스 회전 (Z축 기준)
- Joint 1: 어깨 관절 (수직 운동)
- Joint 2: 팔꿈치 관절 (굽힘)
- Joint 3: 손목 회전

소프트웨어 좌표계 기준:
- Joint 0 = 0: 팔이 X축 방향으로 뻗음
- Joint 1 = 0: 팔이 Z축 방향으로 올곧게 서있음
- Joint 2 = 0: 팔이 -Z축 방향으로 올곧게 내려옴
"""
import numpy as np
from scipy.optimize import fsolve

class KinematicSolver:
    def __init__(self, config=None):
        """
        간단한 기하학적 운동학 해결기
        
        로봇 구조 (cm 단위):
        - 베이스 높이: 17 cm
        - 상완 길이: 10 cm  
        - 상완-하완 링크 길이: 7cm
        - 하완 길이: 10 cm
        - 연필 길이: 17 cm
        """
        # 로봇 링크 길이 (cm)
        self.L1 = 17.0      # 베이스 높이
        self.L2 = 10.0        # 상완 길이
        self.L3 = 7.0          # 상완-하완 링크 길이
        self.L4 = 10.0        # 하완 길이 
        self.L5 = 17.0        # 연필 길이
        
        # 호환성을 위한 속성
        self.num_motor_variables = 4
        self.n_joints = 4
        
        print(f"✅ 기하학적 운동학 해결기 초기화:")
        print(f"   베이스: {self.L1}cm, 상완: {self.L2}cm")
        print(f"   하완: {self.L4}cm, 손목: {self.L5}cm")

    def forward_kinematics(self, joint_angles):
        """
        순운동학: 관절 각도 → 엔드이펙터 위치
        
        Args:
            joint_angles: [theta0, theta1, theta2, theta3] (라디안)
        
        Returns:
            position: [x, y, z] (cm)
            orientation: [roll, pitch, yaw] (라디안)
        """
        theta_0, theta_1, theta_2, _ = joint_angles

        # 수평 거리(r) 계산
        # 팔 링크(L2, L4)와 오프셋(L3)을 고려
        r = self.L2 * np.cos(theta_1) + self.L4 * np.cos(theta_2) + self.L3
        
        # Z 좌표 계산
        # 베이스 높이(L1), 팔 링크(L2, L4) 고려
        z = self.L1 + self.L2 * np.sin(theta_1) + self.L4 * np.sin(theta_2)
        
        # 베이스 회전 적용하여 X, Y 좌표 계산
        x = r * np.cos(theta_0)
        y = r * np.sin(theta_0)
        
        position = np.array([x, y, z])
        
        # 간단한 자세 (팔의 끝 각도와 베이스 회전)
        orientation = np.array([0, theta_1 + theta_2, theta_0])
        
        return position, orientation
    
    def inverse_kinematics(self, target_position, initial_guess_joints=None, **kwargs):
        """
        역운동학: 목표 위치 → 관절 각도
        
        올바른 물리적 구조 기반 해법:
        1. (x, y) → (r, theta_0) 베이스 회전 계산
        2. 실제 구조: 15cm(theta_1) + 10.7cm(수평) + 15cm(theta_2)
        3. r = 15*cos(theta_1) + 10.7 + 15*cos(theta_2)
        4. r = 30*cos(theta_alpha)*cos(theta_beta) + 10.7
        5. 반복적 탐색으로 theta_alpha, theta_beta 계산
        
        Args:
            target_position: [x, y, z] (미터 또는 cm, 자동 감지)
            
        Returns:
            joint_angles: [theta0, theta1, theta2, theta3] (라디안)
        """
        try:
            x, y, z = target_position[:3]
            
            length_n = (np.sqrt(x**2 + y**2) - self.L3)
            length_z = z - self.L1
            
            def equations(vars):
                theta1, theta2 = vars
                eq1 = (np.sin(theta1) + np.sin(theta2))*self.L2 - length_n
                eq2 = (np.cos(theta1) - np.cos(theta2))*self.L2 - length_z
                return [eq1, eq2]
            
            # 초기 추정값
            if initial_guess_joints is not None and len(initial_guess_joints) >= 3:
                # 이전 해에서 fsolve를 위한 초기 추정값 변환
                th_1_guess, th_2_guess = initial_guess_joints[1], initial_guess_joints[2]
                theta1_sol_guess = np.pi / 2.0 - th_1_guess
                theta2_sol_guess = th_2_guess + np.pi / 2.0
                initial_guess = [theta1_sol_guess, theta2_sol_guess]
            else:
                initial_guess = [0, 0]
            
            # 수치 해 찾기
            solution = fsolve(equations, initial_guess)
            theta1_sol, theta2_sol = solution
            
            th_z = np.arctan2(y,x)
            th_0 = self.normalize_angle(th_z)
            th_1 = self.normalize_angle(np.pi/2 - theta1_sol)
            th_2 = self.normalize_angle(-(np.pi/2) + theta2_sol)
            
            th_3 = self.normalize_angle(np.pi/2)
            
            # 뒤집어진 solve 결과 변환 + 앞발이 많이 올라가는 동작 방지
            if (th_1 < 0 and th_2 > 0) or (th_1 < th_2):
                temp = th_1
                th_1 = th_2
                th_2 = temp
            
            # 1번째 관절 뒤로 넘어가려는거 제한
            if th_1 > 1.7:
                th_1 = 1.7
           
            joint_angles = np.array([th_0, th_1, th_2, th_3])
            
            print("joint_angles: ", joint_angles)
            
            # 검증: Forward kinematics로 확인
            verify_pos, _ = self.forward_kinematics(joint_angles)
            error = np.linalg.norm(np.array([x, y, z]) - verify_pos)
            
            if error > 1.0:  # 1cm 오차 허용
                print(f"⚠️ IK 검증 오차: {error:.3f}cm")
                print(f"   목표: [{x:.1f}, {y:.1f}, {z:.1f}] cm")
                print(f"   실제: [{verify_pos[0]:.1f}, {verify_pos[1]:.1f}, {verify_pos[2]:.1f}] cm")
                print(f"   관절: [{np.rad2deg(th_0):.1f}, {np.rad2deg(th_1):.1f}, {np.rad2deg(th_2):.1f}, {np.rad2deg(th_3):.1f}] 도")
            
            return joint_angles
            
        except Exception as e:
            print(f"❌ IK 계산 오류: {e}")
            return None

    def get_workspace_bounds(self):
        """작업 공간 경계 반환 (cm)"""
        max_reach = self.L2 + self.L4 + self.ee_length
        min_reach = max(0, self.L2 - (self.L4 + self.ee_length))
        
        return {
            'max_reach': max_reach,
            'min_reach': min_reach,
            'max_height': self.L1 + max_reach,
            'min_height': self.L1 - max_reach
        }

    def check_joint_limits(self, joint_angles, joint_limits_rad=None):
        """관절 제한 확인"""
        if joint_limits_rad is None:
            # 기본 제한값 (라디안) - YAML 값이 없을 때만 사용
            joint_limits_rad = [
                (-np.pi, np.pi),        # Joint 0: 베이스 회전
                (-np.pi/2, np.pi/2),    # Joint 1: 어깨
                (-np.pi/2, 5*np.pi/6),  # Joint 2: 팔꿈치 (-90° ~ 150°)
                (-np.pi, np.pi)         # Joint 3: 손목 회전
            ]
        
        violations = []
        
        for i, (angle_rad, (min_rad, max_rad)) in enumerate(zip(joint_angles, joint_limits_rad)):
            angle_deg = np.rad2deg(angle_rad)
            min_deg = np.rad2deg(min_rad)
            max_deg = np.rad2deg(max_rad)
            
            if angle_rad < min_rad or angle_rad > max_rad:
                violations.append(f"Joint {i}: {angle_deg:.1f}° (범위: {min_deg:.0f}°~{max_deg:.0f}°)")
        
        return violations

    def joints_to_geometric(self, theta1, theta2):
        """물리적 관절 각도(theta1, theta2)를 기하학적 각도(θ_alpha, θ_beta)로 변환합니다.
        Args:
            theta1 (float): Joint 1 각도 (라디안)
            theta2 (float): Joint 2 각도 (라디안)
        Returns:
            tuple: (theta_alpha, theta_beta) in radians
        """
        theta_alpha = (theta1 + theta2) / 2.0
        theta_beta = (theta1 - theta2) / 2.0
        return theta_alpha, theta_beta

    def geometric_to_joints(self, theta_alpha, theta_beta):
        """기하학적 각도(θ_alpha, θ_beta)를 물리적 관절 각도(theta1, theta2)로 변환합니다.
        Args:
            theta_alpha (float): θ_alpha in radians
            theta_beta (float): θ_beta in radians
        Returns:
            tuple: (theta1, theta2) in radians corresponding to Joint1, Joint2
        """
        theta1 = theta_alpha + theta_beta
        theta2 = theta_alpha - theta_beta
        return theta1, theta2

    def normalize_angle(self, angle):
        """각도를 -π ~ π 범위로 정규화 (래핑)"""
        # 모듈로 연산을 사용한 효율적인 방법
        return ((angle + np.pi) % (2 * np.pi)) - np.pi


def test_kinematics():
    """간단한 테스트"""
    solver = KinematicSolver()
    
    print("\n🧪 순운동학 테스트:")
    test_angles = [0, 0, np.pi/6, 0]  # 30도 팔꿈치
    pos, ori = solver.forward_kinematics(test_angles)
    print(f"   관절: {np.rad2deg(test_angles)} 도")
    print(f"   위치: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}] cm")
    
    print("\n🎯 역운동학 테스트:")
    target = [25, 10, 20]  # cm
    angles = solver.inverse_kinematics(target)
    if angles is not None:
        print(f"   목표: {target} cm")
        print(f"   해: {np.rad2deg(angles)} 도")
    
    print(f"\n📏 작업공간: {solver.get_workspace_bounds()}")

if __name__ == "__main__":
    test_kinematics()
