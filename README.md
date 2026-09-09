# MuJoCo 종합 2주 학습 커리큘럼

> 환경 구축 → 물리 시뮬레이션 → 매니퓰레이터 제어 → Whole-Body Motion → ROS2 연동 → 모방학습/강화학습 기반 플래닝

---

## 📅 Week 1: 기초 환경 구축 & 시뮬레이션 기술

### Day 1-2: 환경 구축 및 MuJoCo 기초

#### 학습 목표
- MuJoCo 설치 및 개발 환경 구성
- 기본 물리 시뮬레이션 개념 이해 (체인动力학,Contact Dynamics)
- XML 기반 모델 명세(MJCF) 작성 능력

#### 학습 내용
```
[오전] 환경 구축
├── Python 3.10+ 설치
├── MuJoCo 3.x 설치 (mujoco-py 또는 bindings 사용)
│   ├── pip install mujoco
│   ├── mujoco-viewer 패키지 설치
│   └── 예제 모델 로드 테스트
├── 개발 도구 세팅
│   ├── VS Code + Python Extension
│   ├── Jupyter Notebook (시각화 테스트용)
│   └── Git 초기화
│
[오후] MJCF 모델 작성 기초
├── MJCF XML 구조 이해
│   ├── <worldbody>, <body>, <joint>, <geom>
│   ├── <actuator>, <sensor>, <contact>
│   └── <equality>, <tendon> 제약 조건
├── 예제: 간단한PENDULUM, CART-POLE 모델 작성
└── mujoco.viewer를 이용한 실시간 시각화
```

#### 실습 과제
- [ ] Cart-Pole 시뮬레이션 구현 및 시각화
- [ ] 단순 매니퓰레이터(2-3DOF) 모델 MJCF 작성
- [ ] 힘/토크 시뮬레이션 관찰 및 그래프 출력

---

### Day 3-4: Forward/Inverse Dynamics & 시스템 제어 알고리즘 기초

#### 학습 목표
- MuJoCo의 Forward/Inverse Dynamics API 이해
- 기본 제어 알고리즘 구현 (PD, PID, Computed Torque)
- 상태 공간 표현 및 시뮬레이션 파이프라인

#### 학습 내용
```
[오전] Forward/Inverse Dynamics 학습
├── Forward Dynamics: qacc = M⁻¹(τ - C - g)
├── Inverse Dynamics: τ = M*qacc + C + g
├── MuJoCo API: mujoco.mj_forward(), mj_inverse()
├── 직접ativo-역직접 actuator 모델 차이점
│
[오후] 기본 제어 알고리즘 구현
├── PD 제어기 (Joint Space)
│   ├── P gain, D gain 튜닝
│   └── MuJoCo에서의 구현: ctrl[] 설정
├── PID 제어기 구현 및 적분과잉 문제 해결
├── Operational Space Control (OSC) 기초
│   ├── Jacobian 기반 위치/자세 제어
│   └── задача: 2-DOF 매니퓰레이터로 특정 위치 도달
└── Real-time 제어 루프 구현 (mj_step() 기반)
```

#### 실습 과제
- [ ] 2-DOF 매니퓰레이터의 PD 제어 구현 (목표 관절 각도 도달)
- [ ] 3-DOF 로봇 팔의 Inverse Kinematics + PD 제어
- [ ] Operational Space Control로 Cartesian space에서 직선 경로 추적
- [ ] 제어 파라미터 튜닝 및 응답 그래프 분석

---

### Day 5: Contact Dynamics & 환경 상호작용

#### 학습 목표
- Contact detection 및 법선력 계산 이해
- 마찰 모델 (Coulomb friction) 적용
- Grasping/Manipulation 시뮬레이션 기초

#### 학습 내용
```
[오전] Contact Dynamics
├── MuJoCo contact detection 메커니즘
│   ├── <contype>, <conaffinity> 플래그
│   └── 관찰 가능한 센서 값:传感器 normal force, friction force
├── 마찰 모델: Coulomb 마찰
│   ├── <joint friction> 설정
│   └── 매끄러운(mujoco内置) vs 거친 표면 시뮬레이션
│
[오후] Manipulation 시뮬레이션
├── 오브젝트 그립핑 (Grasping) 시뮬레이션
│   ├── 조인트 토크 제어로 grip force 조절
│   └── 오브젝트 stability 관찰
├── push/pull 작업 시뮬레이션
└── 데모: Simple Pick-and-Place 시나리오 구성
```

#### 실습 과제
- [ ] 물체를 집어 올리는 2-finger gripper 시뮬레이션
- [ ] 힘 센서 데이터를 활용한 그립 강도 피드백 제어
- [ ] Pick-and-Place 전체 파이프라인 구현

---

### Day 6-7: 로봇 모델링 및 URDF/MJCF 변환

#### 학습 목표
- URDF → MJCF 변환 방법 습득
- 복잡한 로봇 모델 (매니퓰레이터, 휴머노이드) 구성
- 관절 제약, 엔드이펙터 정의, 센서 부착

#### 학습 내용
```
[오전] URDF/MJCF 변환 및 모델 작성
├── URDF 파일 구조 이해
├── urdf2mjcf 또는 deepmimic 변환 스크립트
├── MuJoCo 모델 구조화
│   ├── Frame 설정 및 좌표 변환
│   ├── Inertial properties 정의
│   └── actuator 구성 (position, velocity, torque)
│
[오후] 복잡한 모델 구축
├── Baxter/Sawyer-like 이양 매니퓰레이터 모델
│   └── 양팔 제어를 위한 구조 설계
├── Franka Emika Panda 모델 구성 (기본)
├── 센서 설정
│   ├──关节传感器 (position, velocity, torque)
│   ├── 힘/토크 센서 (force sensor)
│   ├── IMU 센서 (gyro, accelerometer)
│   └── 터치 센서
└── 모델 검증 및 시각화 테스트
```

#### 실습 과제
- [ ] URDF 파일을 MJCF로 변환 및 검증
- [ ] 7-DOF 매니퓰레이터 모델 완성 (Panda 또는 커스텀)
- [ ] 센서 데이터 수집 파이프라인 구현
- [ ] 단순 휴머노이드 모델 (상체만) 구성

---

## 📅 Week 2: 고급 응용 & 학습 기반 플래닝

### Day 8-9: Whole-Body Motion Planning

#### 학습 목표
- 휴머노이드/매니퓰레이터의 복합 모션 계획
- CoM (Center of Mass) 기반 균형 제어
- 임베디드 최적화 기반 모션 계획 (WBC - Whole-Body Control)

#### 학습 내용
```
[오전] Whole-Body Control 이론
├── 휴머노이드 동역학
│   ├── Unified dynamics: M(q)q̈ + C(q,q̇)q̇ + g(q) = τ + JᵀF
│   ├── CoM 역학 및 Zero Moment Point (ZMP)
│   └── 단순화된 모델: LIPM (Linear Inverted Pendulum Mode)
├── Hierarchical Task Space Control
│   ├── Priority 기반 다중 작업 처리
│   └── null-space projection
│
[오후] Motion Planning 구현
├── Contact Planning
│   ├── footstep planning for humanoid
│   └── multi-contact planning (hand + foot)
├── Trajectory Optimization
│   ├── DDP/iLQR 기초 개념
│   └── MuJoCo에서의 cost function 정의
└── 실습: 휴머노이드 보행 시뮬레이션
    ├── 프리PENDULUM 기반 보행 패턴
    └── 균형 유지 및 fall recovery
```

#### 실습 과제
- [ ] 휴머노이드 기본 보행 (walking gait) 시뮬레이션 구현
- [ ] 양팔 매니퓰레이터의 협동 작업 모션 (예: 상자 들어올리기)
- [ ] Whole-Body Controller 구현 및 단일/복합 작업 우선순위 설정
- [ ] 계단 오르기 또는 장애물 회피 모션 시뮬레이션

---

### Day 10: ROS2 + MuJoCo 연동

#### 학습 목표
- ROS2 Humble 환경 구축
- MuJoCo ↔ ROS2 통신 인터페이스 구축
- RViz2를 이용한 실시간 시각화

#### 학습 내용
```
[오전] ROS2 기초
├── ROS2 Humble 설치 (Ubuntu 22.04 권장, Windows WSL2 사용 가능)
├── 기본 개념复习
│   ├── Topic, Service, Action
│   ├── Publisher, Subscriber
│   ├── Launch 파일 작성
│   └── Colcon 빌드 시스템
├── 예제 패키지 생성 및 테스트
│
[오후] MuJoCo-ROS2 브릿지 구축
├── 시뮬레이션 데이터 → ROS2 메시지 변환
│   ├── JointState 메시지 발행
│   ├── TF 트랜스폼 발행
│   ├── Sensor 메시지 (Lidar, Camera)
│   └── Odometry/Imu 메시지
├── ROS2 컨트롤러에서 MuJoCo 제어
│   ├── Subscribing to cmd_vel/command topics
│   └── control_msgs 활용
└── RViz2 시각화
    ├── URDF + TF visualization
    └── Custom marker 발행
```

#### 실습 과제
- [ ] ROS2 패키지에서 MuJoCo 시뮬레이터 실행 및 토픽 확인
- [ ] MuJoCo 시뮬레이션 결과를 RViz2에서 실시간 표시
- [ ] ros2_control 인터페이스를 이용한 원격 제어 구현
- [ ] 커스텀 메시지 정의 및 센서 데이터 ROS2 퍼블리시

---

### Day 11-12: 모방학습 (Imitation Learning) 기반 플래닝

#### 학습 목표
- Expert demonstration 데이터 수집 및 전처리
- Behavior Cloning 구현
- DAgger 및 관련 기법 이해

#### 학습 내용
```
[오전] Demonstration 데이터 수집
├── MuJoCo에서의 demonstration recording
│   ├── state, action, time 시퀀스 저장
│   ├── 하이퍼파라미터 튜닝으로 expert policy 생성
│   └── 데이터 포맷: HDF5, NumPy, ROS bag
├── 데이터 전처리
│   ├── 정규화 (normalization)
│   ├── 시계열 데이터 슬라이싱
│   └── 어그멘테이션 (augmentation)
│
[오후] 모방학습 구현
├── Behavior Cloning (BC)
│   ├── Neural network policy: s → a
│   ├── PyTorch/TensorFlow 기반 구현
│   ├── 손실 함수: MSE, nll
│   └── MuJoCo 환경에서의 평가
├── DAgger (Dataset Aggregation)
│   ├── 초기화: expert policy π* 수집
│   ├── 반복: 학습된 policy π의 실행 → expert 라벨링
│   └── 점진적 개선
└── 실습: simple reaching task with imitation
    ├── 2-DOF 또는 3-DOF reaching demonstration
    └── BC 정확도 비교 ( 다양한 데이터 크기)
```

#### 실습 과제
- [ ] Expert policy (PD/OSC 기반)로 demonstration 데이터 100+ 트래젝토리 생성
- [ ] Behavior Cloning 모델 구현 및 평가
- [ ] DAgger 구현 및 BC 대비 성능 비교
- [ ] 시각화: 예측 vs 실제 action 비교 그래프

---

### Day 13-14: 강화학습 (RL) 기반 모션 플래닝

#### 학습 목표
- MuJoCo에서의 RL 환경 구성
- 기본 RL 알고리즘 (PPO, SAC) 구현
- Sim-to-Real 전략 이해 및 적용

#### 학습 내용
```
[오전] RL 환경 구성 및 기본 알고리즘
├── Gymnasium (OpenAI Gym) 인터페이스
│   ├── 커스텀 환경 생성 (gymnasium.Env)
│   ├── 관찰/행동 공간 정의
│   └── 보상 함수 설계
├── 기본 RL 알고리즘
│   ├── PPO (Proximal Policy Optimization) 개념
│   │   ├── 클리핑 objective
│   │   └── multi-step return
│   ├── SAC (Soft Actor-Critic) 개념
│   │   ├── Maximum entropy RL
│   │   └── Q-function based
│   └── Stable-Baselines3 또는 CleanRL 활용
│
[오후] 실전 RL 학습 파이프라인
├── MuJoCo 환경에서의 학습
│   ├── 보상 함수 설계 예시
│   │   ├── Task reward: goal reaching, manipulation success
│   │   ├── Regularization: energy penalty, smoothness
│   │   └── Shaping reward: progress toward goal
│   ├── 관찰 공간 설계
│   │   ├── Low-level: joint position, velocity
│   │   └── High-level: task-specific state
│   └── 학습 하이퍼파라미터 튜닝
├── Whole-Body RL
│   ├── HybrID (Hybrid Policy + Dynamics)
│   ├── MJP (MimicJump) 스타일의 보행 학습
│   └── Multi-task RL: 동시에 여러 동작 학습
└── Sim-to-Real
    ├── Domain Randomization 기법
    │   ├── 물리 파라미터 randomization
    │   ├── 시각적 변형 (texture, lighting)
    │   └── Latent dynamics randomization
    └── Real-world 적용 전략
```

#### 실습 과제
- [ ] 커스텀 MuJoCo RL 환경 생성 (gymnasium 인터페이스)
- [ ] PPO로 Cart-Pole 학습 및 렌더링
- [ ] SAC로 Robotic Reaching Task 학습
- [ ] Domain Randomization 적용 및 robustness 평가
- [ ] (보너스) 하이퍼파라미터 튜닝 실험 및 비교 리포트 작성

---

### Day 14: 종합 프로젝트 및 정리

#### 프로젝트 옵션 (택 1)

```
Option A: 양팔 매니퓰레이터 협동 작업
├── MJCF로 양팔 로봇 모델 구성
├── ROS2로 통신 인터페이스 구축
├── 시뮬레이션에서 두 물체 동시 조작
├── Behavior Cloning으로 expert demonstration 학습
└── RL로 자체 정책 개선

Option B: 휴머노이드 보행 + 과제 수행
├── 휴머노이드 모델 구성 (standing, walking)
├── Whole-Body Control 기반 보행
├── ROS2로 상태 모니터링 및 제어 인터페이스
├── RL로 보행 안정성 향상
└── 간단한 manipulation 과제 추가 (예: 물건 집기)

Option C: 시뮬레이션-to-現實 브릿지
├── 기존 로봇 모델 (URDF)으로 MuJoCo 시뮬레이션
├── ROS2 통신 및 RViz2 시각화
├── RL policy 학습 및 domain randomization
├── 실제 로봇 포트(ros2_control) 코드 생성
└── 실제 환경과의 성능 차이 분석
```

---

## 📚 추천 학습 자료

### 핵심 라이브러리/도구
| 도구 | 용도 | 설치 |
|------|------|------|
| `mujoco` | 물리 시뮬레이션 | `pip install mujoco` |
| `mujoco-python-viewer` | 시각화 | `pip install mujoco-python-viewer` |
| `gymnasium` | RL 환경 인터페이스 | `pip install gymnasium[mujoco]` |
| `stable-baselines3` | RL 알고리즘 | `pip install stable-baselines3` |
| `pytorch` | 딥러닝 프레임워크 | `pip install torch` |
| `ros-humble-*` | ROS2 패키지 | `sudo apt install ros-humble-*` |
| `robot_state_publisher` | URDF 시각화 | `sudo apt install ros-humble-robot-state-publisher` |

### 도서 및 논문
1. **MuJoCo Documentation** - https://mujoco.org
2. **Deep Reinforcement Learning: Hands-On** - Maxim Lapan
3. **Modern Robotics** - Kevin Lynch (자유 전자책)
4. **Reinforcement Learning: An Introduction** - Sutton & Barto
5. **Learning to Walk in Minutes Using Massively Parallel Deep RL** - Duan et al., 2021 (MuJoCo 보행 학습)
6. **Simpler: Generalized Skill Discovery for Simple Manipulation** - 2023

### GitHub 리포지토리
- `google-deepmind/mujoco` - 공식 MuJoCo
- `google-deepmind/deepmind-control-suite` - 제어 벤치마크
- `NVIDIA-Omniverse/IsaacGymEnvs` - GPU 기반 시뮬레이션
- `leggedrobotics/legged_gym` - 로봇 다리 학습 프레임워크

---

## ⏰ 일별 타임라인 요약

| Day | 주제 | 핵심 산출물 |
|-----|------|-------------|
| 1-2 | 환경 구축 & MJCF 기초 | Cart-Pole 시뮬레이션 |
| 3-4 | Dynamics & 기본 제어 | PD/PID 제어기 구현 |
| 5 | Contact Dynamics | Pick-and-Place 시뮬레이션 |
| 6-7 | URDF/MJCF 변환 & 모델링 | 7-DOF 매니퓰레이터 모델 |
| 8-9 | Whole-Body Motion Planning | 휴머노이드 보행 시뮬레이션 |
| 10 | ROS2 + MuJoCo 연동 | ROS2 브릿지 + RViz2 시각화 |
| 11-12 | 모방학습 | Behavior Cloning 구현 |
| 13-14 | 강화학습 & 종합 프로젝트 | RL policy + 종합 시스템 |

---

## 💡 학습 팁

1. **環境 구축 우선**: 환경이 제대로 돌아가지 않으면 모든 것이 느려집니다. Day 1-2를 충분히 투자하세요.
2. **시각화 활용**: MuJoCo의 실시간 렌더링은 직관적입니다. 시뮬레이션을 자주 확인하세요.
3. **단계적 확장**: 2-DOF → 7-DOF → 휴머노이드 순서로 복잡도를 높이세요.
4. **제어 vs 학습**: 전통적 제어(_PD, OSC)를 먼저 이해하면 RL의 장점이 명확해집니다.
5. **ROS2는 WSL2**: Windows 사용자는 WSL2(Ubuntu 22.04)에서 ROS2를 설치하는 것을 강력히 권장합니다.
6. **코드 버전 관리**: Git으로 매일 커밋하며 진행 과정을 기록하세요.
7. **문서화**: 실험 결과와 파라미터를 기록하면 나중에 큰 도움이 됩니다.

---

*생성일: 2026-09-09*
*학습 기간: 2주 (14일)*
*권장 시간: 매일 4-6시간*
