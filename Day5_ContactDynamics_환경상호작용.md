# Day 5: Contact Dynamics 및 환경 상호작용

## 학습 목표
- MuJoCo의 Contact Detection 메커니즘 이해
- Coulomb 마찰 모델 적용
- 힘 기반 제어 및 그립핑 시뮬레이션
- Pick-and-Place 전체 파이프라인 구현

---

## 1. Contact Dynamics 이론

### 1.1 접촉 검출

MuJoCo는 geometric contact detection을 사용합니다.

```
접촉 조건:
1. 두 geom이 겹침 (penetration)
2. contactfriction 설정이 0이 아님
3. contype/conaffinity 비트가 일치

접촉 정보:
- contactpos: 접촉점 위치
- contactframe: 접촉 프레임 (법선, 마찰 방향)
- contactforce: 접촉력 (normal + friction)
```

### 1.2 Coulomb 마찰 모델

```
F_tangent ≤ μ * F_normal

여기서:
- μ: 마찰 계수
- F_normal: 법선력
- F_tangent: 접선력 (마찰력)
```

MuJoCo에서 설정:
```xml
<geom condim="3" friction="1 0.5 0.01"/>
<!-- condim: 3=3D contact, friction=[torsional, rolling, sliding] -->
```

---

## 2. Contact 센서 및 모니터링

### 2.1 Contact 정보 읽기

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("contact_env.xml")
data = mujoco.MjData(model)

# 시뮬레이션 실행
mujoco.mj_step(model, data)

# 접촉 정보 출력
print(f"활성 접촉 수: {data.ncon}")

for i in range(data.ncon):
    contact = data.contact[i]
    print(f"\n접촉 {i}:")
    print(f"  geom1: {contact.geom1}, geom2: {contact.geom2}")
    print(f"  위치: {contact.pos}")
    print(f"  법선력: {contact.frame[:3]}")
    
    # 힘 계산
    force = np.zeros(6)
    mujoco.mj_contactForce(model, data, i, force)
    print(f"  접촉력: {force[:3]} (normal), {force[3:]} (friction)")
```

### 2.2 Contact 시각화

```xml
<option>
    <flag contact="enable" contactpoint="enable" contactforce="enable"/>
</option>

<visual>
    <rgba haze="0.15 0.25 0.35 1"/>
</visual>
```

---

## 3. 마찰 모델 적용

### 3.1 마찰 설정 예시

```xml
<!-- 바닥 (높은 마찰) -->
<geom name="floor" type="plane" size="5 5 0.1" 
      friction="2 0.5 0.01" condim="3"/>

<!-- 물체 (낮은 마찰) -->
<geom name="box" type="box" size="0.05 0.05 0.05" 
      friction="0.5 0.1 0.01" mass="0.1"/>

<!-- 로봇 손 (중간 마찰) -->
<geom name="gripper_finger" type="box" size="0.02 0.01 0.05" 
      friction="1 0.5 0.01"/>
```

### 3.2 마찰 계수 실험

```python
import matplotlib.pyplot as plt

# 마찰 계수별 물체 미끄러짐 테스트
friction_values = [0.1, 0.5, 1.0, 2.0]
displacement = []

for mu in friction_values:
    # MJCF에서 마찰 계수 변경 후 시뮬레이션
    xml = f"""
    <mujoco>
        <option timestep="0.01"/>
        <worldbody>
            <geom type="plane" size="5 5 0.1" friction="{mu} 0.5 0.01"/>
            <body pos="0 0 0.1">
                <geom type="box" size="0.05 0.05 0.05" mass="1"/>
                <freejoint/>
            </body>
        </worldbody>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    
    # 초기 속도 부여
    data.qvel[0] = 2.0  # x 방향 속도
    
    # 시뮬레이션
    for _ in range(100):
        mujoco.mj_step(model, data)
    
    displacement.append(data.qpos[0])

# 그래프
plt.plot(friction_values, displacement)
plt.xlabel("마찰 계수 (μ)")
plt.ylabel("최종 위치 (m)")
plt.title("마찰 계수에 따른 물체 이동 거리")
plt.grid(True)
plt.savefig("friction_experiment.png")
```

---

## 4. Grasping 시뮬레이션

### 4.1 Two-Finger Gripper 모델

```xml
<mujoco model="gripper_env">
    <option timestep="0.002" gravity="0 0 -9.81"/>
    
    <default>
        <joint damping="0.1"/>
        <geom condim="3"/>
    </default>
    
    <worldbody>
        <light diffuse="0.8 0.8 0.8" pos="0 0 3"/>
        <geom type="plane" size="5 5 0.1" friction="1 0.5 0.01"/>
        
        <!-- 물체 -->
        <body name="box" pos="0 0 0.05">
            <geom type="box" size="0.03 0.03 0.03" rgba="0 1 0 1" mass="0.1"/>
            <freejoint name="box_free"/>
        </body>
        
        <!-- 그리퍼 베이스 -->
        <body name="gripper_base" pos="0 0 0.2">
            <geom type="cylinder" size="0.05 0.05" rgba="0.5 0.5 0.5 1"/>
            
            <!-- 왼쪽 손가락 -->
            <body name="left_finger" pos="0 0.05 0">
                <geom type="box" size="0.01 0.02 0.05" rgba="1 0 0 1" 
                      friction="1 0.5 0.01"/>
                <joint name="left_finger_joint" type="slide" axis="0 -1 0" 
                       range="-0.03 0.03"/>
            </body>
            
            <!-- 오른쪽 손가락 -->
            <body name="right_finger" pos="0 -0.05 0">
                <geom type="box" size="0.01 0.02 0.05" rgba="0 0 1 1" 
                      friction="1 0.5 0.01"/>
                <joint name="right_finger_joint" type="slide" axis="0 1 0" 
                       range="-0.03 0.03"/>
            </body>
        </body>
    </worldbody>
    
    <actuator>
        <position joint="left_finger_joint" kp="100" ctrlrange="-0.03 0"/>
        <position joint="right_finger_joint" kp="100" ctrlrange="-0.03 0"/>
    </actuator>
    
    <sensor>
        <touch site="left_finger_touch"/>
        <touch site="right_finger_touch"/>
    </sensor>
</mujoco>
```

### 4.2 그립핑 제어 로직

```python
import mujoco
import mujoco.viewer
import numpy as np

class GripperController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.grip_force = 0
        self.is_grasping = False
    
    def detect_grasp(self, threshold=0.5):
        """접촉력을 이용한 그립 감지"""
        total_force = 0
        
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            total_force += np.linalg.norm(force[:3])
        
        self.is_grasping = total_force > threshold
        return self.is_grasping
    
    def close_gripper(self, target_force=5.0):
        """그립 닫기 (힘 제어)"""
        current_force = self.get_contact_force()
        
        # 피드백 제어로 목표 힘 달성
        error = target_force - current_force
        adjustment = 0.001 * error
        
        # 손가락 위치 조절
        self.data.ctrl[0] = max(-0.03, self.data.ctrl[0] - adjustment)
        self.data.ctrl[1] = max(-0.03, self.data.ctrl[1] - adjustment)
    
    def get_contact_force(self):
        """현재 접촉력 합산"""
        total_force = 0
        for i in range(self.data.ncon):
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            total_force += np.linalg.norm(force[:3])
        return total_force
    
    def release_gripper(self):
        """그립 열기"""
        self.data.ctrl[0] = 0
        self.data.ctrl[1] = 0
```

---

## 5. Pick-and-Place 파이프라인

### 5.1 전체 시스템 구현

```python
import mujoco
import mujoco.viewer
import numpy as np
import time

class PickAndPlaceSystem:
    def __init__(self, model_path):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.gripper = GripperController(self.model, self.data)
        
        # 상태 머신
        self.state = "APPROACH"
        self.target_box_pos = np.array([0.1, 0, 0.05])
        self.target_place_pos = np.array([-0.1, 0, 0.05])
    
    def run(self):
        """메인 실행 루프"""
        while True:
            self.step()
            
            if self.state == "DONE":
                break
            
            mujoco.mj_step(self.model, self.data)
            time.sleep(self.model.opt.timestep)
    
    def step(self):
        """상태별 처리"""
        box_pos = self.data.qpos[3:6]  # 박스 위치
        
        if self.state == "APPROACH":
            # 박스 위로 접근
            self.move_to(self.target_box_pos + np.array([0, 0, 0.1]))
            
            if np.linalg.norm(box_pos[:2] - self.target_box_pos[:2]) < 0.02:
                self.state = "DESCEND"
        
        elif self.state == "DESCEND":
            # 아래로 내려가기
            self.move_to(self.target_box_pos + np.array([0, 0, 0.03]))
            
            if abs(box_pos[2] - self.target_box_pos[2]) < 0.01:
                self.state = "GRASP"
        
        elif self.state == "GRASP":
            # 그립 닫기
            self.gripper.close_gripper(target_force=5.0)
            
            if self.gripper.detect_grasp():
                self.state = "LIFT"
        
        elif self.state == "LIFT":
            # 들어올리기
            self.move_to(self.target_box_pos + np.array([0, 0, 0.15]))
            
            if box_pos[2] > 0.14:
                self.state = "MOVE"
        
        elif self.state == "MOVE":
            # 목표 위치로 이동
            self.move_to(self.target_place_pos + np.array([0, 0, 0.15]))
            
            if np.linalg.norm(box_pos[:2] - self.target_place_pos[:2]) < 0.02:
                self.state = "PLACE"
        
        elif self.state == "PLACE":
            # 내려놓기
            self.move_to(self.target_place_pos + np.array([0, 0, 0.03]))
            
            if abs(box_pos[2] - self.target_place_pos[2]) < 0.01:
                self.state = "RELEASE"
        
        elif self.state == "RELEASE":
            # 그립 열기
            self.gripper.release_gripper()
            
            if not self.gripper.detect_grasp():
                self.state = "DONE"
    
    def move_to(self, target_pos):
        """위치 제어 (간단한 PD)"""
        current_pos = self.data.qpos[:3]
        error = target_pos - current_pos
        
        # 간단한 위치 제어
        self.data.ctrl[0:3] = 50 * error

# 실행
system = PickAndPlaceSystem("gripper_env.xml")
system.run()
```

---

## 6. 힘 센서 활용

### 6.1 힘 센서 읽기

```xml
<sensor>
    <!-- 법선력 센서 -->
    <force sensor="force_sensor"/>
    
    <!-- 토크 센서 -->
    <torque sensor="torque_sensor"/>
    
    <!-- 관절 힘 -->
    <jointforce joint="joint1"/>
    
    <!-- 관절 토크 -->
    <jointtorque joint="joint2"/>
</sensor>

<site name="force_sensor" pos="0 0 0" size="0.01"/>
```

### 6.2 힘 기반 제어

```python
class ForceController:
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.force_sensor_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, "force_sensor"
        )
    
    def get_force(self):
        """센서 읽기"""
        # 센서 인덱스 계산
        sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "force_sensor"
        )
        sensor_adr = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        
        force = self.data.sensordata[sensor_adr:sensor_adr+dim]
        return force
    
    def maintain_force(self, target_force, kp=10):
        """목표 힘 유지 제어"""
        current_force = self.get_force()
        error = target_force - current_force
        
        # 힘 오차를 위치 보정으로 변환
        position_correction = kp * error
        
        return position_correction
```

---

## 7. 실습 과제

### 과제 1: 물체 집기 (Grasping)
- [ ] 위의 Two-Finger Gripper 모델 완성
- [ ] 다양한 크기/질량의 물체 테스트
- [ ] 그립 성공률 측정 (10회 시도)

### 과제 2: 힘 제어 그립핑
- [ ] 목표 힘을 설정하고 안정적으로 물체 잡기
- [ ] 너무 약한 힘: 미끄러짐, 너무 강한 힘: 물체 파괴 시뮬레이션
- [ ] 최적 그립 힘 찾기 실험

### 과제 3: Pick-and-Place 완성
- [ ] 위의 파이프라인을 실제 모델에 적용
- [ ] 물체를 다른 위치로 옮기기
- [ ] 이동 중 물체 낙하 시 recovery 로직 추가

### 과제 4: Push 작업
- [ ] 물체를 미는 (push) 작업 구현
- [ ] 마찰을 이용한 정밀 위치 제어
- [ ] 여러 물체 순서대로 밀기

---

## 8. 다음 단계 미리보기

Day 6-7에서는:
- URDF → MJCF 변환
- 복잡한 로봇 모델 (7-DOF 매니퓰레이터) 구성
- 센서 패키지 통합

**오늘의 핵심 포인트**: Contact Dynamics를 이해하고, 힘 기반 제어로 물체를 조작할 수 있어야 합니다.

---

*학습 시간: 약 6-8시간 (1일)*
*난이도: ★★★☆☆ (중급)*
