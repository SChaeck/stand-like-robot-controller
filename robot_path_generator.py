import numpy as np
import json
import argparse

def generate_robot_path(
    l1, l2, theta1, theta2, 
    object1_offset=0, object2_offset=0, 
    reach_offset=0, 
    output_file="robot_path.json", time_to_go=1.5):
    """
    로봇 경로를 생성하여 JSON 파일로 저장하는 함수
    
    Args:
        l1 (float): 첫 번째 거리 파라미터
        l2 (float): 두 번째 거리 파라미터  
        theta1 (float): 첫 번째 각도 파라미터 (라디안)
        theta2 (float): 두 번째 각도 파라미터 (라디안)
        output_file (str): 출력 JSON 파일명
    """
    
    path = []
    
    l1 = l1 + reach_offset
    l2 = l2 + reach_offset
    
    # 몸통 돌려서 서랍 열기
    path.extend([
        {
            "id": 1.1,
            "type": "gripper",
            "open": False
        },
        {
            "id": 1.2,
            "type": "move",
            "pos": [8.74 * np.cos(theta2), 8.74 * np.sin(theta2), 17],
            "rot": 0,
            "time": time_to_go,
        },
        {
            "id": 2,
            "type": "move",
            "pos": [(l2-2) * np.cos(theta2), (l2-2) * np.sin(theta2), 25+object2_offset],
            "rot": 0,
            "time": time_to_go
        },
        {
            "id": 4,
            "type": "move", 
            "pos": [(l2-2) * np.cos(theta2), (l2-2) * np.sin(theta2), 17+object2_offset],
            "rot": 0,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 5.1,
            "type": "move",
            "pos": [(l2-4) * np.cos(theta2), (l2-4) * np.sin(theta2), 17+object2_offset], # [temp] 8 + `1`(충분히 열기 위한 추가 힘)
            "rot": 0,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 5.2,
            "type": "move",
            "pos": [(l2-6) * np.cos(theta2), (l2-6) * np.sin(theta2), 17+object2_offset], # [temp] 8 + `1`(충분히 열기 위한 추가 힘)
            "rot": 0,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 5.3,
            "type": "move",
            "pos": [(l2-(8+1)) * np.cos(theta2), (l2-(8+1)) * np.sin(theta2), 17.5+object2_offset], # [temp] 8 + `1`(충분히 열기 위한 추가 힘)
            "rot": 0,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 6,
            "type": "move",
            "pos": [(l2-(8+1)) * np.cos(theta2), (l2-(8+1)) * np.sin(theta2), 22+object2_offset], # [temp] 8 + `1`(충분히 열기 위한 추가 힘)
            "rot": 0,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 7,
            "type": "move",
            "pos": [l2 * np.cos(theta2), l2 * np.sin(theta2), 27+object2_offset],
            "rot": 0,
            "time": time_to_go
        }
    ])
    
    # 최상단 블록으로 이동
    path.extend([
        {
            "id": 9,
            "type": "move",
            "pos": [(l1) * np.cos(theta1), (l1) * np.sin(theta1), 35+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 8,
            "type": "gripper",
            "open": True
        },
        {
            "id": 9,
            "type": "move",
            "pos": [(l1+6) * np.cos(theta1), (l1+6) * np.sin(theta1), 35+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 10,
            "type": "move",
            "pos": [(l1+6) * np.cos(theta1), (l1+6) * np.sin(theta1), 31.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 11,
            "type": "gripper",
            "open": False
        },
        {
            "id": 12,
            "type": "move",
            "pos": [(l1+6) * np.cos(theta1), (l1+6) * np.sin(theta1), 33+object1_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 서랍에 넣기 (첫 번째 블록)
    drawer_x1 = ((l2-1.5)**2 + 4**2)**0.5 * np.cos(theta2 + np.arctan(4/(l2-1.5)))
    drawer_y1 = ((l2-1.5)**2 + 4**2)**0.5 * np.sin(theta2 + np.arctan(4/(l2-1.5)))
    
    path.extend([
        {
            "id": 13,
            "type": "move",
            "pos": [drawer_x1, drawer_y1, 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 14,
            "type": "move",
            "pos": [drawer_x1, drawer_y1, 19+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 15,
            "type": "gripper",
            "open": True
        },
        {
            "id": 16,
            "type": "move",
            "pos": [drawer_x1, drawer_y1, 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 두 번째 블록으로 이동
    path.extend([
        {
            "id": 17,
            "type": "move",
            "pos": [(l1+4.75) * np.cos(theta1), (l1+4.75) * np.sin(theta1), 32.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 18,
            "type": "move",
            "pos": [(l1+4.75) * np.cos(theta1), (l1+4.75) * np.sin(theta1), 29+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 19,
            "type": "gripper",
            "open": False
        },
        {
            "id": 20,
            "type": "move",
            "pos": [(l1+4.75) * np.cos(theta1), (l1+4.75) * np.sin(theta1), 33+object1_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 서랍에 넣기 (두 번째 블록)
    path.extend([
        {
            "id": 21,
            "type": "move",
            "pos": [(l2-2) * np.cos(theta2), (l2-2) * np.sin(theta2), 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 22,
            "type": "move",
            "pos": [(l2-2) * np.cos(theta2), (l2-2) * np.sin(theta2), 19+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 23,
            "type": "gripper",
            "open": True
        },
        {
            "id": 24,
            "type": "move",
            "pos": [(l2-3) * np.cos(theta2), (l2-3) * np.sin(theta2), 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 세 번째 블록으로 이동
    path.extend([
        {
            "id": 25,
            "type": "move",
            "pos": [(l1+7.25) * np.cos(theta1), (l1+7.25) * np.sin(theta1), 32.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 26,
            "type": "move",
            "pos": [(l1+7.25) * np.cos(theta1), (l1+7.25) * np.sin(theta1), 29+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 27,
            "type": "gripper",
            "open": False
        },
        {
            "id": 28,
            "type": "move",
            "pos": [(l1+7.25) * np.cos(theta1), (l1+7.25) * np.sin(theta1), 33+object1_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 서랍에 넣기 (세 번째 블록)
    drawer_x3 = ((l2-1.5)**2 + 4**2)**0.5 * np.cos(theta2 - np.arctan(4/(l2-1.5)))
    drawer_y3 = ((l2-1.5)**2 + 4**2)**0.5 * np.sin(theta2 - np.arctan(4/(l2-1.5)))
    
    path.extend([
        {
            "id": 29,
            "type": "move",
            "pos": [drawer_x3, drawer_y3, 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 30,
            "type": "move",
            "pos": [drawer_x3, drawer_y3, 19+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 31,
            "type": "gripper",
            "open": True
        },
        {
            "id": 32,
            "type": "move",
            "pos": [drawer_x3, drawer_y3, 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 33,
            "type": "gripper",
            "open": False
        },
    ])
    
    # 서랍 닫기
    path.extend([
        {
            "id": 34,
            "type": "move",
            "pos": [(l2-8) * np.cos(theta2), (l2-8) * np.sin(theta2), 22+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 35.1,
            "type": "move",
            "pos": [(l2-8) * np.cos(theta2), (l2-8) * np.sin(theta2), 18+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 35.2,
            "type": "move",
            "pos": [(l2-5) * np.cos(theta2), (l2-5) * np.sin(theta2), 18+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 35.3,
            "type": "move",
            "pos": [(l2-2) * np.cos(theta2), (l2-2) * np.sin(theta2), 18+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 36,
            "type": "move",
            "pos": [(l2+0.5) * np.cos(theta2), (l2+0.5) * np.sin(theta2), 18+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 37,
            "type": "move",
            "pos": [(l2-1) * np.cos(theta2), (l2-1) * np.sin(theta2), 22+object2_offset],
            "rot": 0,
            "time": time_to_go
        }
    ])
    
    # 네 번째 블록 (탑 쌓기)
    path.extend([
        {
            "id": 38,
            "type": "move",
            "pos": [(l1-1) * np.cos(theta1), (l1-1) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 39,
            "type": "gripper",
            "open": True
        },
        {
            "id": 40,
            "type": "move",
            "pos": [(l1+3.5) * np.cos(theta1), (l1+3.5) * np.sin(theta1), 26.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 41,
            "type": "gripper",
            "open": False
        },
        {
            "id": 42,
            "type": "move",
            "pos": [(l1+3.5) * np.cos(theta1), (l1+3.5) * np.sin(theta1), 30.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 43,
            "type": "move",
            "pos": [(l2+7.5) * np.cos(theta2), (l2+7.5) * np.sin(theta2), 26+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 44,
            "type": "move",
            "pos": [(l2+7.5) * np.cos(theta2), (l2+7.5) * np.sin(theta2), 22+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 45,
            "type": "gripper",
            "open": True
        },
        {
            "id": 46,
            "type": "move",
            "pos": [(l2+7.5) * np.cos(theta2), (l2+7.5) * np.sin(theta2), 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 다섯 번째 블록 (탑 쌓기)
    path.extend([
        {
            "id": 47,
            "type": "move",
            "pos": [(l1+1.5) * np.cos(theta1), (l1+1.5) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 48,
            "type": "move",
            "pos": [(l1+6) * np.cos(theta1), (l1+6) * np.sin(theta1), 26.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 49,
            "type": "gripper",
            "open": False
        },
        {
            "id": 50,
            "type": "move",
            "pos": [(l1+6) * np.cos(theta1), (l1+6) * np.sin(theta1), 30.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 51,
            "type": "move",
            "pos": [(l2+5) * np.cos(theta2), (l2+5) * np.sin(theta2), 26+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 52,
            "type": "move",
            "pos": [(l2+5) * np.cos(theta2), (l2+5) * np.sin(theta2), 22+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 53,
            "type": "gripper",
            "open": True
        },
        {
            "id": 54,
            "type": "move",
            "pos": [(l2+5) * np.cos(theta2), (l2+5) * np.sin(theta2), 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 여섯 번째 블록 (탑 쌓기)
    path.extend([
        {
            "id": 55,
            "type": "move",
            "pos": [(l1+4) * np.cos(theta1), (l1+4) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 56,
            "type": "move",
            "pos": [(l1+8.5) * np.cos(theta1), (l1+8.5) * np.sin(theta1), 26.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 57,
            "type": "gripper",
            "open": False
        },
        {
            "id": 58,
            "type": "move",
            "pos": [(l1+8.5) * np.cos(theta1), (l1+8.5) * np.sin(theta1), 30.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 59,
            "type": "move",
            "pos": [(l2+2.5) * np.cos(theta2), (l2+2.5) * np.sin(theta2), 26+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 60,
            "type": "move",
            "pos": [(l2+2.5) * np.cos(theta2), (l2+2.5) * np.sin(theta2), 22+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 61,
            "type": "gripper",
            "open": True
        },  
        {
            "id": 62,
            "type": "move",
            "pos": [(l2+2.5) * np.cos(theta2), (l2+2.5) * np.sin(theta2), 27+object2_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 일곱 번째 블록
    path.extend([
        {
            "id": 63,
            "type": "move",
            "pos": [(l1+2.25) * np.cos(theta1), (l1+2.25) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 64,
            "type": "move",
            "pos": [(l1+2.25) * np.cos(theta1), (l1+2.25) * np.sin(theta1), 24+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 65,
            "type": "gripper",
            "open": False
        },
        {
            "id": 66,
            "type": "move",
            "pos": [(l1+2.25) * np.cos(theta1), (l1+2.25) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 67,
            "type": "move",
            "pos": [(l2+6.25) * np.cos(theta2), (l2+6.25) * np.sin(theta2), 28.5+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 68,
            "type": "move",
            "pos": [(l2+6.25) * np.cos(theta2), (l2+6.25) * np.sin(theta2), 24.5+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 69,
            "type": "gripper",
            "open": True
        },
        {
            "id": 70,
            "type": "move",
            "pos": [(l2+6.25) * np.cos(theta2), (l2+6.25) * np.sin(theta2), 28.5+object2_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 여덟 번째 블록
    path.extend([
        {
            "id": 71,
            "type": "move",
            "pos": [(l1+4.75) * np.cos(theta1), (l1+4.75) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 72,
            "type": "move",
            "pos": [(l1+4.75) * np.cos(theta1), (l1+4.75) * np.sin(theta1), 24+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 73,
            "type": "gripper",
            "open": False
        },
        {
            "id": 74,
            "type": "move",
            "pos": [(l1+4.75) * np.cos(theta1), (l1+4.75) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 75,
            "type": "move",
            "pos": [(l2+3.75) * np.cos(theta2), (l2+3.75) * np.sin(theta2), 28.5+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 76,
            "type": "move",
            "pos": [(l2+3.75) * np.cos(theta2), (l2+3.75) * np.sin(theta2), 24.5+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 77,
            "type": "gripper",
            "open": True
        },
        {
            "id": 78,
            "type": "move",
            "pos": [(l2+3.75) * np.cos(theta2), (l2+3.75) * np.sin(theta2), 28.5+object2_offset],
            "rot": 90,
            "time": time_to_go
        }
    ])
    
    # 아홉 번째 블록
    path.extend([
        {
            "id": 79,
            "type": "move",
            "pos": [(l1+4.75) * np.cos(theta1), (l1+4.75) * np.sin(theta1), 26+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 80,
            "type": "move",
            "pos": [(l1+7.25) * np.cos(theta1), (l1+7.25) * np.sin(theta1), 24+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 81,
            "type": "gripper",
            "open": False
        },
        {
            "id": 82,
            "type": "move",
            "pos": [(l1+7.25) * np.cos(theta1), (l1+7.25) * np.sin(theta1), 27.5+object1_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 83,
            "type": "move",
            "pos": [(l2+5) * np.cos(theta2), (l2+5) * np.sin(theta2), 29.5+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
        {
            "id": 84,
            "type": "move",
            "pos": [(l2+5) * np.cos(theta2), (l2+5) * np.sin(theta2), 27+object2_offset],
            "rot": 90,
            "time": time_to_go,
            "use_cartesian_interpolation": True
        },
        {
            "id": 85,
            "type": "gripper",
            "open": True
        },
        {
            "id": 84,
            "type": "move",
            "pos": [(l2+5) * np.cos(theta2), (l2+5) * np.sin(theta2), 29.5+object2_offset],
            "rot": 90,
            "time": time_to_go
        },
    ])
    
    # Home position
    path.append({
        "id": 86,
        "type": "home"
    })
    
    # 그리퍼 닫기
    path.append({
        "id": 87,
        "type": "gripper",
        "open": False
    })
    
    # JSON 파일로 저장
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(path, f, indent=2, ensure_ascii=False)
    
    print(f"Robot path generated and saved to {output_file}")
    print(f"Total commands: {len(path)}")
    
    return path

def main():
    """메인 함수 - 커맨드라인 인터페이스"""
    parser = argparse.ArgumentParser(description='Generate robot path JSON file')
    parser.add_argument('theta1', type=float, help='First angle parameter (degrees)')
    parser.add_argument('l1', type=float, help='First distance parameter')
    parser.add_argument('theta2', type=float, help='Second angle parameter (degrees)')
    parser.add_argument('l2', type=float, help='Second distance parameter')
    parser.add_argument('time_to_go', type=float, help='Time to go (seconds)')
    parser.add_argument('-o', '--output', type=str, default=None,
                        help='Output JSON file name')
    
    args = parser.parse_args()
    
    # 각도를 라디안으로 변환
    theta1_rad = np.radians(args.theta1)
    theta2_rad = np.radians(args.theta2)
    
    OBJECT1_OFFSET = -6.5
    OBJECT2_OFFSET = 1
    
    REACH_OFFSET = 1
    
    if args.output is None:
        args.output = f"robot_path_{int(args.theta1)}_{int(args.l1)}_{int(args.theta2)}_{int(args.l2)}_{args.time_to_go}_{OBJECT1_OFFSET}_{OBJECT2_OFFSET}_{REACH_OFFSET}.json"
    
    print('args.output', args.output)
    
    output_file = f"GUI/paths/{args.output}"
        
    # 경로 생성
    generate_robot_path(args.l1, args.l2, theta1_rad, theta2_rad, 
                        object1_offset=OBJECT1_OFFSET, object2_offset=OBJECT2_OFFSET,
                        reach_offset=REACH_OFFSET,
                        output_file=output_file, time_to_go=args.time_to_go)

if __name__ == "__main__":
    # 예시 실행
    # 커맨드라인 인수가 없으면 예시 값으로 실행
    import sys
    if len(sys.argv) == 1:
        print("Example usage:")
        print("python robot_path_generator.py 45 25 90 30 1.0")
        print("\nRunning with example values: theta1=45°, l1=25, theta2=90°, l2=30, time_to_go=1.5")
        
        # 예시 값으로 실행
        path = generate_robot_path(
            l1=25, l2=30, 
            theta1=np.radians(45), 
            theta2=np.radians(90),
            output_file="GUI/paths/robot_path_25_30_45_90_1.5.json",
            time_to_go=1.5,
        )
    else:
        main()