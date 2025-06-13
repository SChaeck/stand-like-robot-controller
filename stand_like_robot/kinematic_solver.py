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
        self.base_height = 17.0      # 베이스 높이
        self.upper_arm = 10.0        # 상완 길이
        self.lower_arm = 10.0        # 하완 길이 
        self.arm_link = 7.0          # 상완-하완 링크 길이
        self.ee_length = 17.0     # 연필 길이
        # 연필 = 17cm
        
        # 호환성을 위한 속성
        self.num_motor_variables = 4
        self.n_joints = 4
        
        print(f"✅ 기하학적 운동학 해결기 초기화:")
        print(f"   베이스: {self.base_height}cm, 상완: {self.upper_arm}cm")
        print(f"   하완: {self.lower_arm}cm, 손목: {self.ee_length}cm")

    def forward_kinematics(self, joint_angles):
        """
        순운동학: 관절 각도 → 엔드이펙터 위치
        
        올바른 기하학적 해법:
        - Joint 0: 베이스 회전 (theta_0)
        - Joint 1: theta_alpha + theta_beta  
        - Joint 2: theta_alpha - theta_beta
        - 수평거리: 30*cos(theta_alpha)*cos(theta_beta) + 10.7 (cm)
        - Z좌표: 17 + 30*sin(theta_alpha)*cos(theta_beta) (cm)
        
        Args:
            joint_angles: [theta0, theta1, theta2, theta3] (라디안)
        
        Returns:
            position: [x, y, z] (cm)
            orientation: [roll, pitch, yaw] (라디안)
        """
        theta_0 = joint_angles[0]  # 베이스 회전
        theta_1 = joint_angles[1]  # theta_alpha + theta_beta
        theta_2 = joint_angles[2]  # theta_alpha - theta_beta
        theta_3 = joint_angles[3]  # 손목 회전
        
        # 올바른 공식에 따른 계산
        theta_alpha = (theta_1 + theta_2) / 2
        theta_beta = (theta_1 - theta_2) / 2
        
        # 수평 거리 계산: 30*cos(theta_alpha)*cos(theta_beta) + 10.7
        r = 30.0 * np.cos(theta_alpha) * np.cos(theta_beta) + 10.7
        
        # Z 좌표 계산: 17 + 30*sin(theta_alpha)*cos(theta_beta)
        z = 17.0 + 30.0 * np.sin(theta_alpha) * np.cos(theta_beta)
        
        # 베이스 회전 적용
        x = r * np.cos(theta_0)
        y = r * np.sin(theta_0)
        
        position = np.array([x, y, z])
        
        # 간단한 자세 (팔 방향)
        orientation = np.array([0, theta_1, theta_0])
        
        return position, orientation

    def inverse_kinematics(self, target_position, **kwargs):
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
            
            # 단위 자동 감지 (미터면 cm로 변환)
            if abs(x) < 5 and abs(y) < 5 and abs(z) < 5:  # 미터로 추정
                x, y, z = x * 100, y * 100, z * 100
            
            # 1단계: 베이스 회전 계산
            theta_0 = np.arctan2(y, x)
            r_target = np.sqrt(x**2 + y**2)  # 목표점까지의 수평 거리
            
            # 2단계: 실제 로봇 구조 기반 해석적 IK
            # 목표: r = 30*cos(theta_alpha)*cos(theta_beta) + 10.7
            #       z = 17 + 30*sin(theta_alpha)*cos(theta_beta)
            
            # 특별한 경우: z = 17 (베이스 높이와 같음)
            if abs(z - 17.0) < 0.1:  # z ≈ 17cm
                # z = 17이면 30*sin(theta_alpha)*cos(theta_beta) = 0이어야 함
                # sin(theta_alpha) = 0 또는 cos(theta_beta) = 0
                # 일반적으로 theta_alpha = 0 (팔이 수평)
                best_theta_alpha = 0.0
                
                # r = 30*cos(0)*cos(theta_beta) + 10.7 = 30*cos(theta_beta) + 10.7
                cos_beta = (r_target - 10.7) / 30.0
                if abs(cos_beta) <= 1.0:
                    best_theta_beta = np.arccos(abs(cos_beta))
                    best_error = 0.0
                    print(f"✅ z=17 특별한 경우: theta_alpha=0°, theta_beta={np.rad2deg(best_theta_beta):.1f}°")
                else:
                    print(f"⚠️ z=17이지만 도달 불가능한 거리: r={r_target:.1f}cm (최대: {40.7}cm)")
                    best_theta_alpha = 0.0
                    best_theta_beta = 0.0
                    best_error = float('inf')
            else:
                # 일반적인 경우: 반복적 탐색 (정밀도 개선)
                best_error = float('inf')
                best_theta_alpha = 0
                best_theta_beta = 0
                
                # 1단계: 거친 탐색 (1도 간격)
                for beta_deg in range(0, 91, 1):  # 0~90도, 1도 간격
                    theta_beta = np.deg2rad(beta_deg)
                    
                    # alpha 범위: [-beta, 90-beta]
                    alpha_min = max(-beta_deg, -90)  # 최소 -90도로 제한
                    alpha_max = min(90 - beta_deg, 90)  # 최대 90도로 제한
                    
                    for alpha_deg in range(alpha_min, alpha_max + 1, 1):  # 1도 간격
                        theta_alpha = np.deg2rad(alpha_deg)
                        
                        # Forward kinematics 계산
                        r_calc = 30.0 * np.cos(theta_alpha) * np.cos(theta_beta) + 10.7
                        z_calc = 17.0 + 30.0 * np.sin(theta_alpha) * np.cos(theta_beta)
                        
                        # 오차 계산
                        error_r = abs(r_calc - r_target)
                        error_z = abs(z_calc - z)
                        total_error = error_r + error_z
                        
                        if total_error < best_error:
                            best_error = total_error
                            best_theta_alpha = theta_alpha
                            best_theta_beta = theta_beta
                
                # 2단계: 세밀한 탐색 (0.1도 간격으로 미세 조정)
                if best_error > 0.01:  # 1cm 이상 오차가 있으면 미세 조정
                    best_alpha_deg = np.rad2deg(best_theta_alpha)
                    best_beta_deg = np.rad2deg(best_theta_beta)
                    
                    # 최적해 주변 ±2도 범위에서 0.1도 간격 탐색
                    fine_alpha_min = max(best_alpha_deg - 2, -90)
                    fine_alpha_max = min(best_alpha_deg + 2, 90)
                    fine_beta_min = max(best_beta_deg - 2, 0)
                    fine_beta_max = min(best_beta_deg + 2, 90)
                    
                    alpha_range = np.arange(fine_alpha_min, fine_alpha_max + 0.1, 0.1)
                    beta_range = np.arange(fine_beta_min, fine_beta_max + 0.1, 0.1)
                    
                    for beta_deg in beta_range:
                        theta_beta = np.deg2rad(beta_deg)
                        
                        for alpha_deg in alpha_range:
                            # alpha 범위 재확인
                            if alpha_deg < -beta_deg or alpha_deg > 90 - beta_deg:
                                continue
                                
                            theta_alpha = np.deg2rad(alpha_deg)
                            
                            # Forward kinematics 계산
                            r_calc = 30.0 * np.cos(theta_alpha) * np.cos(theta_beta) + 10.7
                            z_calc = 17.0 + 30.0 * np.sin(theta_alpha) * np.cos(theta_beta)
                            
                            # 오차 계산
                            error_r = abs(r_calc - r_target)
                            error_z = abs(z_calc - z)
                            total_error = error_r + error_z
                            
                            if total_error < best_error:
                                best_error = total_error
                                best_theta_alpha = theta_alpha
                                best_theta_beta = theta_beta
            
            # 3단계: theta_alpha, theta_beta를 theta_1, theta_2로 변환
            theta_1 = best_theta_alpha + best_theta_beta
            theta_2 = best_theta_alpha - best_theta_beta
            theta_3 = 0.0  # Joint 3 (손목)
            
            joint_angles = np.array([theta_0, theta_1, theta_2, theta_3])
            
            # 검증: Forward kinematics로 확인
            verify_pos, _ = self.forward_kinematics(joint_angles)
            error = np.linalg.norm(np.array([x, y, z]) - verify_pos)
            
            if error > 1.0:  # 1cm 오차 허용
                print(f"⚠️ IK 검증 오차: {error:.3f}cm")
                print(f"   목표: [{x:.1f}, {y:.1f}, {z:.1f}] cm")
                print(f"   실제: [{verify_pos[0]:.1f}, {verify_pos[1]:.1f}, {verify_pos[2]:.1f}] cm")
                print(f"   관절: [{np.rad2deg(theta_0):.1f}, {np.rad2deg(theta_1):.1f}, {np.rad2deg(theta_2):.1f}, {np.rad2deg(theta_3):.1f}] 도")
                print(f"   theta_alpha: {np.rad2deg(best_theta_alpha):.1f}°, theta_beta: {np.rad2deg(best_theta_beta):.1f}°")
            
            return joint_angles
            
        except Exception as e:
            print(f"❌ IK 계산 오류: {e}")
            return None

    def get_workspace_bounds(self):
        """작업 공간 경계 반환 (cm)"""
        max_reach = self.upper_arm + self.lower_arm + self.ee_length
        min_reach = max(0, self.upper_arm - (self.lower_arm + self.ee_length))
        
        return {
            'max_reach': max_reach,
            'min_reach': min_reach,
            'max_height': self.base_height + max_reach,
            'min_height': self.base_height - max_reach
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
