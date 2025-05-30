- [x] 모터 Initial Joint 설정 (혹은 현재 joint를 init poistion으로 매핑해서 사용)

  - 현재 joint position을 인식해서 init position으로 매핑하는 (환경변수로 세팅하는) .sh 파일

- [ ] 현재 Joint 기반으로 End effector의 Cartesian position 계산 -- Forward kinematics

- [ ] 가야하는 (x, y, z) 입력하면 현재 Cartesian position 기반으로 동선 결정 -- Joint-based Trajectory planning

  - 한 스텝(시간 단위)에 이동할 수 있는 거리를 기반(maximum velocity)으로 동선 결정

- [ ] 이동해야 하는 cartesian velocity를 기반으로 Jacobian을 통한 Joint velocity 계산 -- Inverse Kinematics

- [ ] 계산된 Joint velocity를 기반으로 어느 정도의 Torque를 만들어야할지 계산 -- Inverse Dynamics

- [ ] (선택) Inverse Dynamics에서 제대로 이동하지 않은 값을 보정 -- PID Controller

- [ ] 3~6의 과정을 한 번 거친후 다시 목표 (x, y, z)를 기반으로 동선 결정 -- 적극적 feedback
