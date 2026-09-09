# Day 3-4: Forward/Inverse Dynamics 및 기본 제어 알고리즘

## 학습 목표
- MuJoCo Dynamics API (Forward/Inverse) 활용 능력
- PD, PID, Computed Torque 제어 알고리즘 구현
- Operational Space Control (OSC) 기초 이해
- 실시간 제어 루프 구현

---

## 1. Dynamics 이론 기초

### 1.1 Forward Dynamics

Forward Dynamics는 관절 토크(τ)로부터 가속도(q̈)를 계산합니다.

```
M(q)q̈ + C(q,q̇)q̇ + g(q) = τ + JᵀF

여기서:
- M(q): 관성 행렬 (Mass matrix)
- C(q,q̇): 코리올리/원심력 벡터
- g(q): 중력 벡터
- τ: 관절 토크
- J: 자코비안
- F: 끝점 외력
```

### 1.2 Inverse Dynamics

Inverse Dynamics는 원하는 가속도(q̈)로부터 필요한 토크(τ)를 계산합니다.

```
τ = M(q)q̈_desired + C(q,q̇)q̇ + g(q)
```

---

## 2. MuJoCo Dynamics API

### 2.1 Forward Dynamics 계산

```python
import mujoco
import numpy as np

# 모델/데이터 로드
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

# 상태 설정
data.qpos[0] = np.deg2rad(30)  # 첫 번째 관절 각도
data.qvel[0] = 0.5  # 관절 속도

# 관절 토크 설정
data.qfrc_applied[0] = 10.0  # 외부 토크 적용

# Forward dynamics 실행
mujoco.mj_forward(model, data)

# 결과 확인
print("가속도 (qacc):", data.qacc)
print("관성 행렬 (M):", data.qM.reshape(model.nv, model.nv))
```

### 2.2 Inverse Dynamics 계산

```python
# desired 가속도 설정
qacc_desired = np.zeros(model.nv)
qacc_desired[0] = 5.0  # 원하는 관절 가속도

# 가속도 설정
data.qacc[:] = qacc_desired

# Inverse dynamics 실행
mujoco.mj_inverse(model, data)

# 필요한 토크 확인
print("필요한 토크:", data.qfrc_inverse)
```

### 2.3 MJCF에서 dynamics 설정

```xml
<option>
    <flag contact="enable"/>
</option>

<default>
    <joint damping="0.1" armature="0.1"/>
    <geom condim="3" friction="1 0.5 0.01"/>
</default>

<actuator>
    <!--Gain 설정으로 dynamics 특성 제어 -->
    <position joint="j1" kp="100" kv="10" damping="0.5"/>
</actuator>
```

---

## 3. 기본 제어 알고리즘

### 3.1 PD 제어기 (Joint Space)

```python
import mujoco
import mujoco.viewer
import numpy as np

xml = """
<mujoco>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    
    <worldbody>
        <light diffuse="0.8 0.8 0.8" pos="0 0 3"/>
        <geom type="plane" size="5 5 0.1"/>
        
        <body name="link1" pos="0 0 0.5">
            <geom type="cylinder" size="0.05 0.3" rgba="1 0 0 1"/>
            <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
            
            <body name="link2" pos="0 0 0.3">
                <geom type="cylinder" size="0.04 0.25" rgba="0 1 0 1"/>
                <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
            </body>
        </body>
    </worldbody>
    
    <actuator>
        <general joint="joint1" gainprm="1" biasprm="0 0 0"/>
        <general joint="joint2" gainprm="1" biasprm="0 0 0"/>
    </actuator>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# PD 제어기 클래스
class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
    
    def compute(self, current_pos, current_vel, target_pos, target_vel=0):
        error_pos = target_pos - current_pos
        error_vel = target_vel - current_vel
        return self.kp * error_pos + self.kd * error_vel

# 컨트롤러 초기화
controller = PDController(kp=50, kd=10)

# 목표 관절 각도
target_angles = np.array([np.deg2rad(45), np.deg2rad(-30)])

# 시뮬레이션 루프
def control_step(model, data):
    current_pos = data.qpos[:2]
    current_vel = data.qvel[:2]
    
    torques = controller.compute(current_pos, current_vel, target_angles)
    
    data.ctrl[0] = torques[0]
    data.ctrl[1] = torques[1]
    
    mujoco.mj_step(model, data)

mujoco.viewer.launch(model, data, control_step)
```

### 3.2 PID 제어기

```python
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = None
    
    def compute(self, current_pos, current_vel, target_pos, dt):
        if self.integral is None:
            self.integral = np.zeros_like(current_pos)
        
        error_pos = target_pos - current_pos
        error_vel = -current_vel
        
        # 적분항 업데이트
        self.integral += error_pos * dt
        
        # 적분항 바운딩 (anti-windup)
        self.integral = np.clip(self.integral, -10, 10)
        
        return (self.kp * error_pos + 
                self.ki * self.integral + 
                self.kd * error_vel)

# PID 컨트롤러
pid_controller = PIDController(kp=50, ki=5, kd=10)
dt = model.opt.timestep

def pid_control_step(model, data):
    current_pos = data.qpos[:2]
    current_vel = data.qvel[:2]
    
    torques = pid_controller.compute(current_pos, current_vel, target_angles, dt)
    
    data.ctrl[0] = torques[0]
    data.ctrl[1] = torques[1]
    
    mujoco.mj_step(model, data)
```

### 3.3 Computed Torque Control

```python
class ComputedTorqueController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
    
    def compute(self, model, data, target_pos, target_vel, target_acc):
        # Forward dynamics를 이용해 M, C, g 계산
        mujoco.mj_forward(model, data)
        
        # 현재 상태
        q = data.qpos[:2]
        qd = data.qvel[:2]
        
        # desired 가속도
        qdd_desired = target_acc + self.kp * (target_pos - q) + self.kd * (target_vel - qd)
        
        # 관성 행렬
        M = data.qM[:2, :2].reshape(2, 2)
        
        # Gravity 및 Coriolis
        mujoco.mj_forward(model, data)
        Cg = data.qfrc_bias[:2]
        
        # Computed torque
        tau = M @ qdd_desired + Cg
        
        return tau

ct_controller = ComputedTorqueController(kp=100, kd=20)
```

---

## 4. Operational Space Control (OSC)

### 4.1 이론

Operational Space Control는 Cartesian space에서 직접 제어합니다.

```
F = Λ * (ẍ_desired - ẋ̇_desired * kd + (x_desired - x) * kp)
τ = Jᵀ * F

여기서:
- Λ = (J * M⁻¹ * Jᵀ)⁻¹: operational space inertia matrix
- J: 자코비안 행렬
- x: 엔드이펙터 위치
```

### 4.2 OSC 구현

```python
class OperationalSpaceController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
    
    def compute(self, model, data, target_pos, target_vel=np.zeros(3)):
        # Forward dynamics
        mujoco.mj_forward(model, data)
        
        # 엔드이펙터 위치/속도 계산
        ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        
        # 위치
        ee_pos = data.site_xpos[ee_site_id]
        
        # 속도 (자코비안 활용)
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, jacp, None, ee_site_id)
        ee_vel = jacp @ data.qvel
        
        # 자코비안
        J = jacp[:, :2]  # 관절 2개만
        
        # 관성 행렬
        M_inv = np.linalg.inv(data.qM[:2, :2].reshape(2, 2))
        
        # Operational space inertia
        Lambda_inv = J @ M_inv @ J.T
        Lambda = np.linalg.inv(Lambda_inv)
        
        # 위치/속도 오차
        pos_error = target_pos - ee_pos
        vel_error = target_vel - ee_vel
        
        # 제어력 계산
        F = Lambda @ (self.kp * pos_error + self.kd * vel_error)
        
        # 관절 토크 변환
        tau = J.T @ F
        
        return tau
```

---

## 5. 실시간 제어 루프 구현

### 5.1 기본 구조

```python
import mujoco
import mujoco.viewer
import time

class RealTimeController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.pd = PDController(kp=50, kd=10)
        self.target = np.zeros(2)
    
    def control_loop(self):
        """단일 제어 스텝"""
        # 센서 데이터 읽기
        current_pos = self.data.qpos[:2]
        current_vel = self.data.qvel[:2]
        
        # 제어 입력 계산
        control = self.pd.compute(current_pos, current_vel, self.target)
        
        # 액추레이터에 적용
        self.data.ctrl[0] = control[0]
        self.data.ctrl[1] = control[1]
        
        # 시뮬레이션 스텝
        mujoco.mj_step(self.model, self.data)
    
    def run(self, duration=10):
        """지정된 시간 동안 실행"""
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.control_loop()
            
            # 실시간 동기화 (선택사항)
            # real_elapsed = time.time() - start_time
            # sim_time = self.data.time
            # if real_elapsed < sim_time:
            #     time.sleep(sim_time - real_elapsed)

# 실행
controller = RealTimeController(model, data)
controller.run(duration=10)
```

### 5.2 시각화와 통합

```python
import mujoco.viewer

def step_callback(model, data):
    """뷰어에서 매 스텝 호출되는 콜백"""
    controller.control_loop()

# 인터랙티브 뷰어 실행
mujoco.viewer.launch(model, data, step_callback)
```

---

## 6. 실습 과제

### 과제 1: PD 제어기 튜닝
- [ ] kp, kd 값을 변경하면서 응답 관찰
- [ ] 과도한 kp = 진동, 과도한 kd = 둔감
- [ ] 파라미터에 따른 settling time 그래프 출력

```python
# 실험 스크립트 예시
kp_values = [10, 30, 50, 100, 200]
kd_values = [1, 5, 10, 20, 50]

results = []
for kp in kp_values:
    for kd in kd_values:
        # 시뮬레이션 실행
        # settling time 측정
        results.append({'kp': kp, 'kd': kd, 'settling_time': ...})

# 히트맵으로 시각화
```

### 과제 2: 3-DOF 매니퓰레이터 IK + PD
- [ ] 3-DOF 매니퓰레이터 모델 생성
- [ ] Inverse Kinematics 함수 작성
- [ ] IK 결과를 PD 제어로 추적

```python
import numpy as np
from scipy.optimize import minimize

def inverse_kinematics(target_pos, current_q):
    """3-DOF 매니퓰레이터의 IK"""
    def objective(q):
        # Forward kinematics로 엔드이펙터 위치 계산
        ee_pos = forward_kinematics(q)
        return np.linalg.norm(ee_pos - target_pos)**2
    
    result = minimize(objective, current_q, method='L-BFGS-B')
    return result.x
```

### 과제 3: OSC로 Cartesian space 제어
- [ ] 위의 OSC 구현을 완성
- [ ] 엔드이펙터를 직선으로 이동시키는 테스트
- [ ] Joint space PD와 OSC 성능 비교

---

## 7. API 빠른 참조

```python
# Forward dynamics
mujoco.mj_forward(model, data)  # forward 계산
mujoco.mj_step(model, data)     # 시뮬레이션 스텝

# Inverse dynamics
mujoco.mj_inverse(model, data)  # inverse 계산

# 자코비안
jacp = np.zeros((3, model.nv))
jacr = np.zeros((3, model.nv))
mujoco.mj_jacBody(model, data, jacp, jacr, body_id)

# 상태 정보
qpos = data.qpos          # 관절 위치
qvel = data.qvel          # 관절 속도
qacc = data.qacc          # 관절 가속도
ctrl = data.ctrl          # 액추레이터 입력
qfrc = data.qfrc_applied  # 외부 힘

# 센서
sensor_data = data.sensordata
```

---

## 8. 다음 단계 미리보기

Day 5에서는:
- Contact Dynamics (접촉 역학)
- 마찰 모델 (Coulomb friction)
- 그립핑/매니퓰레이션 시뮬레이션

**오늘의 핵심 포인트**: Forward/Inverse Dynamics를 이해하고, PD/OSC 제어기를 구현할 수 있어야 합니다.

---

*학습 시간: 약 8-10시간 (2일)*
*난이도: ★★☆☆☆ (기초-중급)*
