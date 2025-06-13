# 4-DOF Stand-Like Robot Control System

4자유도 로봇팔 제어 시스템 - 하드웨어/시뮬레이션 듀얼 모드 지원

## 🚀 주요 기능

- **🎯 정밀한 역운동학**: scipy 최적화 기반 IK 솔버
- **🖼️ 인터랙티브 GUI**: 원 그리기 & 경로 기록/재생
- **🔄 듀얼 모드**: 실제 하드웨어 + 시뮬레이션 동시 실행
- **📊 실시간 시각화**: 3D matplotlib 기반 로봇 링크 표시
- **⚙️ 모듈화 설계**: 레이어별 책임 분리로 유지보수성 극대화

## 🎮 빠른 시작

### 원 그리기 모드

```bash
python run_circle_drawer.py
```

### 경로 기록/재생 모드

```bash
python run_path_recorder.py
```

## 📁 프로젝트 구조

```
📦 4-DOF Stand-Like Robot/
├── 🎨 GUI/                          # GUI 레이어
│   ├── circle_drawer_gui.py          # 원 궤적 생성 인터페이스
│   ├── path_recorder_gui.py          # 경로 기록/재생 인터페이스
│   └── 📁 paths/                     # 저장된 경로 파일들
├── 🔧 motor_controller/              # 모터 레이어
│   ├── real_motor_controller.py      # 실제 Dynamixel 모터 제어
│   └── mock_motor_controller.py      # 시뮬레이션용 모터 제어
├── 🤖 stand_like_robot/              # 로봇 제어 레이어
│   ├── stand_like_robot.py           # 메인 로봇 제어 클래스
│   ├── robot_simulator.py            # 시뮬레이션 Facade
│   ├── kinematic_solver.py           # 순/역운동학 계산 엔진
│   ├── trajectory_planner.py         # 부드러운 궤적 계획
│   └── 📁 config/
│       └── stand_like_robot.yaml     # DH 파라미터 & 모터 설정
├── 🏃 run_circle_drawer.py           # 원 그리기 실행 파일
├── 🏃 run_path_recorder.py           # 경로 기록 실행 파일
└── 📋 requirements.txt               # 의존성 라이브러리
```

## 🏗️ 아키텍처 설계

### 레이어별 책임 분리

```
[GUI Layer]           → 사용자 입력 & 시각화
       ↓ Cartesian (X,Y,Z) + Orientation
[Simulation Facade]   → 3D 렌더링 & 상태 중계
       ↓ Cartesian (X,Y,Z) + Yaw
[Robot Controller]    → IK 계산 & 궤적 계획
       ↓ Joint Radians
[Motor Layer]         → 하드웨어 통신 & 단위 변환
```

### 핵심 클래스

- **`CircleDrawerGUI`**: 원형 궤적 생성 및 실시간 실행
- **`PathRecorderGUI`**: 수동 경로 기록 & 자동 재생
- **`StandLikeRobot`**: 메인 로봇 제어 (IK + 궤적 계획)
- **`RobotSimulator`**: 3D 시각화 & 시뮬레이션 상태 관리
- **`KinematicSolver`**: 순/역운동학 & 관절 제한 검증
- **`TrajectoryPlanner`**: 부드러운 관절 궤적 생성

## ⚙️ 하드웨어 사양

### 로봇 구성

- **4-DOF 로봇팔**: AX0(베이스) → AX1(어깨) → AX2(팔꿈치) → AX3(손목)
- **2-DOF 그리퍼**: XC1, XC2 (좌우 집게)
- **통신**: RS485 기반 Dynamixel 프로토콜

### 물리적 치수

- 베이스 높이: 17mm
- 상완 길이: 10mm
- 하완 길이: 10mm
- 연필 길이: 17mm

## 🔧 설치 및 설정

### 1. 의존성 설치

```bash
pip install -r requirements.txt
```

### 2. 하드웨어 연결 (선택사항)

- Dynamixel 모터를 USB2AX 또는 U2D2로 PC 연결
- `stand_like_robot/config/stand_like_robot.yaml`에서 포트 설정

### 3. 설정 파일 수정

```yaml
# stand_like_robot/config/stand_like_robot.yaml
general_settings:
  baudrate: 115200
  port: "/dev/ttyUSB0" # Windows: "COM3"
```

## 🎯 사용 방법

### 원 그리기 모드

1. `python run_circle_drawer.py` 실행
2. 원의 중심점(X, Y, Z) 입력
3. 반지름 및 점 개수 설정
4. "Generate & Execute Circle" 버튼 클릭

### 경로 기록 모드

1. `python run_path_recorder.py` 실행
2. **기록**: "Start Recording" → 수동 조작 → "Stop Recording"
3. **저장**: "Save Path" → 파일명 입력
4. **재생**: "Load Path" → 파일 선택 → "Play Path"

## 🔍 듀얼 모드 동작

실제 로봇이 연결된 경우:

- **왼쪽 창**: 실시간 3D 시뮬레이션
- **오른쪽 창**: 실제 로봇 제어

실제 로봇이 연결되지 않은 경우:

- **자동 시뮬레이션 모드**: MockMotorController 사용

## 🧮 운동학 모델

### 관절 제한

- **AX0 (베이스)**: -150° ~ +150°
- **AX1 (어깨)**: 0° ~ 90°
- **AX2 (팔꿈치)**: -90° ~ 0°
- **AX3 (손목)**: 0° ~ 90°

## 🛠️ 개발자 가이드

### 새로운 GUI 추가

1. `GUI/` 디렉토리에 새 파일 생성
2. `RobotSimulator` 클래스를 통해 로봇 제어
3. 루트에 실행 스크립트 생성

### 모터 제어 확장

1. `motor_controller/` 에 새 컨트롤러 클래스 추가
2. `RealMotorController` 인터페이스 구현
3. `StandLikeRobot`에서 사용

### DH 파라미터 수정

1. `stand_like_robot/config/stand_like_robot.yaml` 편집
2. `links` 섹션에서 물리적 치수 조정
3. `arm_motors` 섹션에서 관절 제한 설정

## 🐛 문제 해결

### 하드웨어 연결 실패

```bash
# 포트 확인 (Linux)
ls /dev/ttyUSB*

# 포트 확인 (Windows)
# 장치 관리자에서 COM 포트 확인
```

### IK 솔버 실패

- 목표 위치가 작업공간 밖에 있는지 확인
- 관절 제한에 의해 도달 불가능한 영역인지 확인
- `kinematic_solver.py`의 `validate_workspace()` 함수 활용

### GUI 렌더링 문제

```bash
# matplotlib 백엔드 확인
python -c "import matplotlib; print(matplotlib.get_backend())"

# macOS에서 GUI 문제 시
pip install --upgrade matplotlib
```

## 📄 라이선스

MIT License - 교육 및 연구 목적으로 자유롭게 사용 가능

## 🤝 기여

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request
