"""
로봇 시뮬레이터 클래스

시뮬레이션 전담 기능:
- 3D 시각화 로직
- 모터 위치 ↔ 라디안 변환
- 시뮬레이션 상태 관리
- GUI 데모들이 공통으로 사용할 수 있는 인터페이스
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotSimulator:
    def __init__(self, robot_instance):
        """
        로봇 시뮬레이터 초기화
        
        Args:
            robot_instance: StandLikeRobot 인스턴스
        """
        self.robot = robot_instance
        self.kinematic_solver = robot_instance.kinematic_solver
        
        # 시뮬레이션 상태
        self.current_joints = robot_instance.sw_init_joint_radians.copy()
        self.target_position = None
        self.circle_points = []
        self.trajectory = []
        
        print("🎮 RobotSimulator 초기화 완료")
    
    ######## Simulation State Management ########
    def update_joint_angles(self, joint_radians):
        """시뮬레이션 관절 각도 업데이트"""
        self.current_joints = joint_radians.copy()
    
    def get_current_joint_angles(self):
        """현재 시뮬레이션 관절 각도 반환"""
        return self.current_joints.copy()
    
    def get_current_joint_degrees(self):
        """현재 시뮬레이션 관절 각도 (도 단위)"""
        return [np.rad2deg(rad) for rad in self.current_joints]
    
    def get_end_effector_position(self):
        """현재 엔드이펙터 위치 계산"""
        return self.kinematic_solver.forward_kinematics(self.current_joints)
    
    ######## 3D Visualization ########
    def visualize_robot(self, ax, show_circle_points=None, current_point_idx=None, target_pos=None):
        """로봇 3D 시각화 (interactive_robot_demo 스타일)"""
        ax.clear()
        
        # 현재 관절 각도
        theta_0 = self.current_joints[0]
        theta_1 = self.current_joints[1]
        theta_2 = self.current_joints[2]
        
        # === 실제 로봇 구조 시각화 ===
        points = []
        
        # 0번: 베이스 (0, 0, 0)
        points.append(np.array([0, 0, 0]))
        
        # 1번: Joint0 & Joint1 위치 (z=17)
        points.append(np.array([0, 0, 17]))
        
        # 2번: Joint1에서 theta_1 각도로 15cm 링크 끝점
        link1_x = 15.0 * np.cos(theta_1)
        link1_z = 17.0 + 15.0 * np.sin(theta_1)
        points.append(np.array([link1_x, 0, link1_z]))
        
        # 3번: Point 2에서 Z축에 수평으로 10.7cm 연장
        joint2_x = link1_x + 10.7
        joint2_z = link1_z
        points.append(np.array([joint2_x, 0, joint2_z]))
        
        # 4번: Joint2에서 theta_2 각도로 15cm 링크
        end_x_physical = joint2_x + 15.0 * np.cos(theta_2)
        end_z_physical = joint2_z + 15.0 * np.sin(theta_2)
        points.append(np.array([end_x_physical, 0, end_z_physical]))
        
        # theta_0 회전 적용
        final_points = []
        for i, point in enumerate(points):
            if i <= 1:  # 베이스와 Joint0/1은 Z축 상에 고정
                final_points.append(point)
            else:
                rotated = np.array([
                    point[0] * np.cos(theta_0) - point[1] * np.sin(theta_0),
                    point[0] * np.sin(theta_0) + point[1] * np.cos(theta_0),
                    point[2]
                ])
                final_points.append(rotated)
        
        # 링크 연결선 그리기
        connections = [(0,1), (1,2), (2,3), (3,4)]
        for start_idx, end_idx in connections:
            start_point = final_points[start_idx]
            end_point = final_points[end_idx]
            ax.plot([start_point[0], end_point[0]],
                   [start_point[1], end_point[1]],
                   [start_point[2], end_point[2]], 'b-', linewidth=4)
        
        # 포인트 표시
        colors = ['black', 'red', 'green', 'blue', 'purple']
        names = ['Base', 'Joint0&1', '15cm끝', 'Joint2', 'End']
        
        for i, (point, color, name) in enumerate(zip(final_points, colors, names)):
            ax.scatter(*point, color=color, s=100, label=f'{name}')
        
        # 원의 포인트들 표시 (생성된 경우)
        if show_circle_points is not None:
            points_array = np.array(show_circle_points)
            ax.plot(points_array[:, 0], points_array[:, 1], points_array[:, 2], 
                   'r--', alpha=0.7, linewidth=2, label='목표 원')
            ax.scatter(points_array[:, 0], points_array[:, 1], points_array[:, 2], 
                      c='orange', s=30, alpha=0.7)
        
        # 현재 그리고 있는 포인트 강조 (애니메이션 중)
        if current_point_idx is not None and show_circle_points is not None:
            if current_point_idx < len(show_circle_points):
                current_target = show_circle_points[current_point_idx]
                ax.scatter(*current_target, color='red', s=200, marker='*', 
                          label='현재 목표', edgecolor='black', linewidth=2)
        
        # 목표 위치 표시 (IK 계산용)
        if target_pos is not None:
            ax.scatter(*target_pos, color='gold', s=200, marker='*', 
                      label='Target', edgecolor='black', linewidth=2)
        
        # 축 설정
        ax.set_xlim([-50, 50])
        ax.set_ylim([-50, 50])
        ax.set_zlim([0, 50])
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_zlabel('Z (cm)')
        ax.set_title('Robot Simulation')
        
        # 범례
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        return final_points
    
    def visualize_robot_with_gripper(self, ax, left_finger_angle=150, right_finger_angle=150, 
                                   show_orientation=True, target_pos=None):
        """그리퍼와 orientation을 포함한 로봇 시각화 (interactive_robot_demo 스타일)"""
        # 기본 로봇 구조 시각화
        final_points = self.visualize_robot(ax, target_pos=target_pos)
        
        if len(self.current_joints) >= 4 and show_orientation:
            # 4번째 관절 (Orientation) 시각화
            end_effector_pos = final_points[4]
            theta_0 = self.current_joints[0]
            theta_3 = self.current_joints[3]  # 4번째 관절 (Z축 회전)
            
            # 엔드이펙터에서 orientation 방향 표시 (5cm 길이 화살표)
            arrow_length = 5.0
            
            # 로컬 X축 방향 (theta_3 회전 적용)
            local_x_dir = np.array([
                arrow_length * np.cos(theta_3),
                arrow_length * np.sin(theta_3),
                0
            ])
            
            # 베이스 회전(theta_0) 적용
            rotated_x_dir = np.array([
                local_x_dir[0] * np.cos(theta_0) - local_x_dir[1] * np.sin(theta_0),
                local_x_dir[0] * np.sin(theta_0) + local_x_dir[1] * np.cos(theta_0),
                local_x_dir[2]
            ])
            
            orientation_end = end_effector_pos + rotated_x_dir
            
            # Orientation arrow drawing (red)
            ax.plot([end_effector_pos[0], orientation_end[0]],
                   [end_effector_pos[1], orientation_end[1]],
                   [end_effector_pos[2], orientation_end[2]], 'r-', linewidth=4, label='Orientation')
            
            # === 그리퍼 시각화 ===
            finger_length = 3.0
            
            # 왼쪽 핑거
            left_finger_angle_rad = np.deg2rad(left_finger_angle) + theta_3
            left_finger_dir = np.array([
                finger_length * np.cos(left_finger_angle_rad),
                finger_length * np.sin(left_finger_angle_rad),
                0
            ])
            rotated_left_finger = np.array([
                left_finger_dir[0] * np.cos(theta_0) - left_finger_dir[1] * np.sin(theta_0),
                left_finger_dir[0] * np.sin(theta_0) + left_finger_dir[1] * np.cos(theta_0),
                left_finger_dir[2]
            ])
            left_finger_end = end_effector_pos + rotated_left_finger
            
            # 오른쪽 핑거
            right_finger_angle_rad = np.deg2rad(right_finger_angle) + theta_3 + np.pi
            right_finger_dir = np.array([
                finger_length * np.cos(right_finger_angle_rad),
                finger_length * np.sin(right_finger_angle_rad),
                0
            ])
            rotated_right_finger = np.array([
                right_finger_dir[0] * np.cos(theta_0) - right_finger_dir[1] * np.sin(theta_0),
                right_finger_dir[0] * np.sin(theta_0) + right_finger_dir[1] * np.cos(theta_0),
                right_finger_dir[2]
            ])
            right_finger_end = end_effector_pos + rotated_right_finger
            
            # Gripper fingers drawing (green)
            ax.plot([end_effector_pos[0], left_finger_end[0]],
                   [end_effector_pos[1], left_finger_end[1]],
                   [end_effector_pos[2], left_finger_end[2]], 'g-', linewidth=3, label='Left Finger')
            
            ax.plot([end_effector_pos[0], right_finger_end[0]],
                   [end_effector_pos[1], right_finger_end[1]],
                   [end_effector_pos[2], right_finger_end[2]], 'g--', linewidth=3, label='Right Finger')
        
        return final_points
    
    ######## Information Display ########
    def get_status_data(self, target_pos=None):
        """현재 상태 정보를 dict 형태로 반환한다.

        Keys:
            joint_rad: list[float] – 관절 각도(라디안)
            joint_deg: list[float] – 관절 각도(도)
            end_effector_cm: list[float] – [x,y,z] cm 위치
            target_error: float|None – 타깃과의 오차(cm, None 이면 미계산)
        """
        joint_deg = self.get_current_joint_degrees()
        end_pos, _ = self.get_end_effector_position()

        status = {
            "joint_rad": self.current_joints.copy(),
            "joint_deg": joint_deg,
            "end_effector_cm": end_pos.tolist() if hasattr(end_pos, 'tolist') else list(end_pos),
            "target_error": None,
        }

        if target_pos is not None:
            status["target_error"] = float(np.linalg.norm(np.array(end_pos) - np.array(target_pos)))

        return status

    def get_gripper_state(self, left_angle=150, right_angle=150):
        """그리퍼 정보를 dict 형태로 반환한다."""
        avg_angle = (left_angle + right_angle) / 2
        opening_percentage = (avg_angle - 120) / (180 - 120) * 100
        opening_percentage = max(0, min(100, opening_percentage))

        if opening_percentage > 80:
            status_str = "Open"
        elif opening_percentage < 20:
            status_str = "Closed"
        else:
            status_str = "Partially Open"

        return {
            "opening_percentage": opening_percentage,
            "left_angle": left_angle,
            "right_angle": right_angle,
            "status": status_str,
        }

    def get_status_info(self, target_pos=None):
        """현재 상태 정보 반환"""
        data = self.get_status_data(target_pos)

        info = "=== Current Status ===\n"
        jd = data["joint_deg"]
        info += f"Joint Angles: [{jd[0]:.1f}, {jd[1]:.1f}, {jd[2]:.1f}"
        if len(jd) >= 4:
            info += f", {jd[3]:.1f}"
        info += "]°\n"

        x, y, z = data["end_effector_cm"]
        info += f"End Effector: [{x:.1f}, {y:.1f}, {z:.1f}]cm\n"

        if data["target_error"] is not None:
            info += f"Target Error: {data['target_error']:.2f}cm\n"

        return info
    
    def get_gripper_info(self, left_angle=150, right_angle=150):
        """그리퍼 상태 정보 반환"""
        data = self.get_gripper_state(left_angle, right_angle)

        info = "=== Gripper Information ===\n"
        info += f"Opening Percentage: {data['opening_percentage']:.0f}%\n"
        info += f"Left Finger: {data['left_angle']:.1f}°\n"
        info += f"Right Finger: {data['right_angle']:.1f}°\n"
        info += f"Status: {data['status']}"

        return info

    ######## High-level Action API ########
    def move_end_effector(self, target_pos_cm, z_rotation_deg=0, time_to_go=2.0, sync_state=True):
        """엔드이펙터를 (x,y,z)로 이동시키고 내부 시뮬레이터 상태를 동기화한다.

        Args:
            target_pos_cm (list|tuple): [x, y, z] in **cm**.
            z_rotation_deg (float): 엔드이펙터 Z-축 회전 각도(도). 기본 0.
            time_to_go (float): 이동에 걸리는 시간(초).
            sync_state (bool): True면 이동 후 StandLikeRobot 의 관절값을 읽어 current_joints 에 반영.

        Returns:
            list[float]: 최종 관절 라디안 값(4개) 또는 None (실패 시)
        """
        try:
            # StandLikeRobot 내부에서 IK → Trajectory → Motor 제어 수행
            self.robot.move_to_position_with_orientation(
                target_position=target_pos_cm,
                z_rotation_deg=z_rotation_deg,
                time_to_go=time_to_go
            )

            # 이동 완료 후 현재 관절 각도 읽기
            joint_radians = self.robot.get_current_arm_joint_radians()

            if sync_state and joint_radians:
                self.update_joint_angles(joint_radians)

            return joint_radians

        except Exception as e:
            print(f"❌ move_end_effector 실패: {e}")
            return None

    def move_to_home_position(self, sync_state=True):
        """로봇을 YAML에서 정의된 초기 위치로 이동시킨다.
        
        Args:
            sync_state (bool): True면 이동 후 시뮬레이터 상태를 동기화.
            
        Returns:
            list[float]: 최종 관절 라디안 값(4개) 또는 None (실패 시)
        """
        try:
            # StandLikeRobot의 초기 위치 이동 함수 호출
            self.robot.move_to_initial_position()
            
            # 이동 완료 후 현재 관절 각도 읽기
            joint_radians = self.robot.get_current_arm_joint_radians()
            
            if sync_state and joint_radians:
                self.update_joint_angles(joint_radians)
            
            return joint_radians
            
        except Exception as e:
            print(f"❌ move_to_home_position 실패: {e}")
            return None

    def open_gripper(self, time_to_go: float = 1.0, sync_state: bool = True):
        """StandLikeRobot의 open_gripper 를 래핑한다."""
        try:
            self.robot.open_gripper(time_to_go=time_to_go)
            if sync_state:
                # gripper does not affect arm joints, but future extensions may add finger joints
                pass
            return True
        except Exception as e:
            print(f"❌ open_gripper 실패: {e}")
            return False

    def close_gripper(self, time_to_go: float = 1.0, sync_state: bool = True):
        """StandLikeRobot의 close_gripper 를 래핑한다."""
        try:
            self.robot.close_gripper(time_to_go=time_to_go)
            if sync_state:
                pass
            return True
        except Exception as e:
            print(f"❌ close_gripper 실패: {e}")
            return False 