# Day 8-9: Whole-Body Motion Planning

## 학습 목표
- 휴머노이드/매니퓰레이터의 복합 모션 계획 이론 이해
- CoM (Center of Mass) 기반 균형 제어
- Hierarchical Task Space Control 구현
- 보행 패턴 생성 및 시뮬레이션

---

## 1. Whole-Body Dynamics 이론

### 1.1 통합 역학 방정식

```
M(q)q̈ + C(q,q̇)q̇ + g(q) = Sᵀτ + Jᵀ_c F_c

여기서:
- M(q): 관성 행렬 (n×n)
- C(q,q̇): 코리올리/원심력 벡터 (n×1)
- g(q): 중력 벡터 (n×1)
- S: 선택 행렬 (자유도 선택)
- τ: 관절 토크 ( actuator 수)
- J_c: contact 자코비안
- F_c: contact 힘
```

### 1.2 CoM (Center of Mass) 역학

```
x_com = Σ(m_i * x_i) / M_total
ẋ_com = Σ(m_i * ẋ_i) / M_total
ẍ_com = Σ(m_i * ẍ_i) / M_total

동역학:
M_total * ẍ_com = Σ F_ext
```

### 1.3 Zero Moment Point (ZMP)

```
ZMP 위치:
x_zmp = x_com - (z_com * ẍ_com) / (g + z̈_com)
y_zmp = y_com - (z_com * ẍ_com) / (g + z̈_com)

안정 조건:
ZMP가 support polygon 내부에 있어야 함
```

---

## 2. LIPM (Linear Inverted Pendulum Mode)

### 2.1 이론

LIPM은 휴머노이드 보행의 단순화된 모델입니다.

```
상태 방정식:
ẋ = v
v = (g/h) * x  (단방향)

전이 행렬:
x[k+1] = A * x[k] + B * u[k]

A = [1  dt]
    [g/h*dt  1]

B = [0]
    [dt]
```

### 2.2 LIPM 시뮬레이션

```python
import numpy as np
import matplotlib.pyplot as plt

class LIPM:
    def __init__(self, height=0.8, dt=0.01):
        self.h = height  # CoM 높이
        self.g = 9.81
        self.dt = dt
        
        # 상태: [x, ẋ]
        self.state = np.zeros(2)
    
    def step(self, foot_position):
        """단일 스텝"""
        x, xd = self.state
        
        # 가속도 계산
        xdd = (self.g / self.h) * (x - foot_position)
        
        # 적분
        xd_new = xd + xdd * self.dt
        x_new = x + xd_new * self.dt
        
        self.state = np.array([x_new, xd_new])
        return self.state
    
    def generate_trajectory(self, foot_positions, duration):
        """보행 궤적 생성"""
        trajectory = []
        steps = int(duration / self.dt)
        
        for i in range(steps):
            t = i * self.dt
            foot_idx = int(t / (duration / len(foot_positions)))
            foot_idx = min(foot_idx, len(foot_positions) - 1)
            
            self.step(foot_positions[foot_idx])
            trajectory.append(self.state.copy())
        
        return np.array(trajectory)

# 보행 시뮬레이션
lipm = LIPM(height=0.8, dt=0.01)

# 발 위치 (좌우 교대로)
foot_positions = [
    np.array([0, 0]),      # 왼쪽
    np.array([0.15, 0]),   # 오른쪽
    np.array([0.3, 0]),    # 왼쪽
    np.array([0.45, 0]),   # 오른쪽
]

trajectory = lipm.generate_trajectory(foot_positions, duration=4.0)

# 시각화
plt.figure(figsize=(10, 5))
plt.plot(trajectory[:, 0], label='CoM x-position')
plt.xlabel('Time step')
plt.ylabel('Position (m)')
plt.title('LIPM CoM Trajectory')
plt.legend()
plt.grid(True)
plt.show()
```

---

## 3. Footstep Planning

### 3.1 발 위치 계획

```python
import numpy as np

class FootstepPlanner:
    def __init__(self, step_length=0.15, step_width=0.2, step_time=0.5):
        self.step_length = step_length
        self.step_width = step_width
        self.step_time = step_time
        
        # 초기 상태
        self.current_pos = np.array([0.0, 0.0])
        self.current_heading = 0.0  # radian
    
    def plan_footsteps(self, num_steps, direction='forward'):
        """보행 발자국 계획"""
        footsteps = []
        
        for i in range(num_steps):
            if direction == 'forward':
                # 전진 보행
                dx = self.step_length * np.cos(self.current_heading)
                dy = self.step_length * np.sin(self.current_heading)
            elif direction == 'left':
                # 좌회전
                self.current_heading += np.pi / 6
                dx = self.step_length * np.cos(self.current_heading)
                dy = self.step_length * np.sin(self.current_heading)
            elif direction == 'right':
                # 우회전
                self.current_heading -= np.pi / 6
                dx = self.step_length * np.cos(self.current_heading)
                dy = self.step_length * np.sin(self.current_heading)
            
            # 발 위치 계산
            if i % 2 == 0:  # 왼쪽
                foot_pos = self.current_pos + np.array([dx, dy + self.step_width/2])
            else:  # 오른쪽
                foot_pos = self.current_pos + np.array([dx, dy - self.step_width/2])
            
            footsteps.append({
                'position': foot_pos,
                'time': i * self.step_time,
                'side': 'left' if i % 2 == 0 else 'right'
            })
            
            self.current_pos = self.current_pos + np.array([dx, dy])
        
        return footsteps
    
    def interpolate_trajectory(self, footsteps, dt=0.01):
        """부드러운 궤적 보간"""
        trajectory = []
        
        for i in range(len(footsteps) - 1):
            start = footsteps[i]['position']
            end = footsteps[i + 1]['position']
            start_time = footsteps[i]['time']
            end_time = footsteps[i + 1]['time']
            
            # 선형 보간
            steps = int((end_time - start_time) / dt)
            for j in range(steps):
                t = j / steps
                pos = (1 - t) * start + t * end
                trajectory.append({
                    'time': start_time + j * dt,
                    'position': pos
                })
        
        return trajectory

# 사용 예시
planner = FootstepPlanner(step_length=0.15, step_width=0.2, step_time=0.5)
footsteps = planner.plan_footsteps(num_steps=10, direction='forward')
```

---

## 4. Hierarchical Task Space Control

### 4.1 이론

여러 작업을 우선순위에 따라 동시 처리합니다.

```
작업 우선순위:
1. Balance (균형 유지) - 최고 우선순위
2. CoM tracking
3. End-effector tracking
4. Posture (자세)

수학적 표현:
τ = Σ (J_iᵀ * F_i)

J_i: 작업 i의 자코비안
F_i: 작업 i의 제어력
```

### 4.2 구현

```python
import numpy as np
import mujoco

class WholeBodyController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.n_joints = model.nu
        
        # 작업 가중치
        self.task_weights = {
            'balance': 1000,      # 최고 우선
            'com': 500,
            'ee': 100,
            'posture': 10
        }
    
    def compute_com_jacobian(self):
        """CoM 자코비안 계산"""
        mass = self.model.body_mass
        com_pos = np.zeros(3)
        total_mass = np.sum(mass)
        
        for i in range(self.model.nbody):
            body_mass = mass[i]
            body_pos = self.data.xipos[i]
            com_pos += body_mass * body_pos
        
        com_pos /= total_mass
        
        # CoM 자코비안 (단순화)
        # 실제로는 mj_jacBody를 사용
        J_com = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, J_com, None, 0)  # body 0 = base
        
        return J_com, com_pos
    
    def compute_ee_jacobian(self, body_id):
        """엔드이펙터 자코비안 계산"""
        J_ee = np.zeros((6, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, J_ee[:3], J_ee[3:], body_id)
        return J_ee
    
    def balance_control(self, target_com):
        """균형 제어 (CoM 위치 제어)"""
        J_com, current_com = self.compute_com_jacobian()
        
        # CoM 오차
        error = target_com - current_com
        
        # 제어력 계산
        kp = self.task_weights['balance']
        F_com = kp * error[:2]  # x, y만 제어 (z는 고정)
        
        # 관절 토크 변환
        tau = J_com[:2].T @ F_com
        
        return tau
    
    def ee_control(self, body_id, target_pos, target_vel=None):
        """엔드이펙터 제어"""
        J_ee = self.compute_ee_jacobian(body_id)
        
        # 현재 위치
        current_pos = self.data.xipos[body_id]
        
        # 위치 오차
        pos_error = target_pos - current_pos
        
        # 제어력
        kp = self.task_weights['ee']
        F_ee = kp * pos_error
        
        # 속도 제어 (선택적)
        if target_vel is not None:
            current_vel = J_ee[:3] @ self.data.qvel
            kd = self.task_weights['ee'] * 0.1
            F_ee += kd * (target_vel - current_vel)
        
        # 관절 토크 변환
        tau = J_ee[:3].T @ F_ee
        
        return tau
    
    def posture_control(self, target_q):
        """자세 제어 (관절 각도)"""
        current_q = self.data.qpos[:self.n_joints]
        
        kp = self.task_weights['posture']
        kd = self.task_weights['posture'] * 0.1
        
        tau = kp * (target_q - current_q) - kd * self.data.qvel[:self.n_joints]
        
        return tau
    
    def hierarchical_control(self, tasks):
        """계층적 제어"""
        # 전체 토크 벡터
        tau_total = np.zeros(self.n_joints)
        
        # 잔여 자유도 (null space)
        N = np.eye(self.model.nv)
        
        for task_name, task_fn, target in tasks:
            # 작업 자코비안 계산
            J, current = task_fn()
            
            # 현재 null space에서의 자코비안
            J_N = J @ N
            
            # 제어력 계산
            error = target - current
            kp = self.task_weights[task_name]
            F = kp * error
            
            # 관절 토크
            tau_task = J_N.T @ F
            
            # 토크 추가
            tau_total[:self.n_joints] += tau_task[:self.n_joints]
            
            # Null space 업데이트
            N = N - np.linalg.pinv(J_N) @ J_N
        
        return tau_total
```

---

## 5. 보행 시뮬레이션

### 5.1 휴머노이드 보행 시스템

```python
import mujoco
import mujoco.viewer
import numpy as np
import time

class WalkingSystem:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.controller = WholeBodyController(self.model, self.data)
        self.footstep_planner = FootstepPlanner()
        
        # 보행 파라미터
        self.step_time = 0.5  # 초/스텝
        self.step_height = 0.05  # 발 높이
        
        # 상태
        self.current_step = 0
        self.footsteps = []
        
    def generate_walking_pattern(self, num_steps=10):
        """보행 패턴 생성"""
        self.footsteps = self.footstep_planner.plan_footsteps(num_steps)
        return self.footsteps
    
    def compute_support_polygon(self):
        """지지 다각형 계산"""
        # 현재 지지 발 위치
        support_foot = self.get_support_foot()
        
        # 지지 다각형 (간단화)
        polygon = [
            support_foot + np.array([0.1, 0.1]),
            support_foot + np.array([0.1, -0.1]),
            support_foot + np.array([-0.1, -0.1]),
            support_foot + np.array([-0.1, 0.1])
        ]
        
        return polygon
    
    def get_support_foot(self):
        """현재 지지 발 반환"""
        if self.current_step % 2 == 0:
            return self.data.xpos[self.model.body_name2id('left_foot')]
        else:
            return self.data.xpos[self.model.body_name2id('right_foot')]
    
    def swing_foot_trajectory(self, target_pos, duration):
        """스윙 발 궤적 생성"""
        current_pos = self.get_swing_foot_pos()
        
        # 5차 다항식 보간
        trajectory = []
        steps = int(duration / self.model.opt.timestep)
        
        for i in range(steps):
            t = i / steps
            
            # 5차 다항식
            s = 10 * t**3 - 15 * t**4 + 6 * t**5
            
            # 높이 프로필 (반원형)
            height_profile = 4 * self.step_height * t * (1 - t)
            
            pos = (1 - s) * current_pos + s * target_pos
            pos[2] += height_profile
            
            trajectory.append(pos)
        
        return trajectory
    
    def get_swing_foot_pos(self):
        """스윙 발 위치"""
        if self.current_step % 2 == 0:
            return self.data.xpos[self.model.body_name2id('right_foot')].copy()
        else:
            return self.data.xpos[self.model.body_name2id('left_foot')].copy()
    
    def step(self):
        """단일 보행 스텝"""
        if self.current_step >= len(self.footsteps):
            return False
        
        target = self.footsteps[self.current_step]['position']
        
        # 스윙 발 궤적
        trajectory = self.swing_foot_trajectory(target, self.step_time)
        
        # 궤적 추적
        for pos in trajectory:
            # 엔드이펙터 제어
            foot_body = 'right_foot' if self.current_step % 2 == 0 else 'left_foot'
            body_id = self.model.body_name2id(foot_body)
            
            tau = self.controller.ee_control(body_id, pos)
            
            # 액추레이터에 적용
            self.data.ctrl[:self.model.nu] = tau[:self.model.nu]
            
            # 시뮬레이션 스텝
            mujoco.mj_step(self.model, self.data)
        
        self.current_step += 1
        return True
    
    def run(self, duration=10):
        """보행 실행"""
        self.generate_walking_pattern(num_steps=20)
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            if not self.step():
                break
            
            time.sleep(self.model.opt.timestep)
    
    def visualize(self):
        """시각화"""
        def step_callback(model, data):
            self.step()
        
        mujoco.viewer.launch(self.model, self.data, step_callback)

# 실행
walking_system = WalkingSystem("humanoid.xml")
walking_system.visualize()
```

---

## 6. 실습 과제

### 과제 1: LIPM 보행 시뮬레이션
- [ ] LIPM 모델로 기본 보행 궤적 생성
- [ ] 다양한 보행 속도 테스트
- [ ] 안정성 분석 (ZMP 그래프 출력)

### 과제 2: 발자국 계획
- [ ] FootstepPlanner로 다양한 경로 계획
- [ ] 직선, 곡선, 계단 보행 패턴
- [ ] 장애물 회피 발자국 계획

### 과제 3: Whole-Body Control
- [ ] 위의 Hierarchical Controller 구현
- [ ] 균형 유지 + 엔드이펙터 동시 제어
- [ ] 우선순위 변경 실험

### 과제 4: 보행 시뮬레이션
- [ ] humanoid 모델에서 기본 보행 구현
- [ ] 보행 안정성 평가
- [ ] fall recovery 로직 추가

---

## 7. 다음 단계 미리보기

Day 10에서는:
- ROS2 Humble 환경 구축
- MuJoCo ↔ ROS2 통신 인터페이스
- RViz2 시각화

**오늘의 핵심 포인트**: Whole-Body Control의 이론과 구현을 이해하고, 기본적인 보행 시뮬레이션을 수행할 수 있어야 합니다.

---

*학습 시간: 약 8-10시간 (2일)*
*난이도: ★★★★☆ (중상급)*
