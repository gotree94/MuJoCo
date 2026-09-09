# Day 1-2: MuJoCo 환경 구축 및 MJCF 기초

## 학습 목표
- MuJoCo 개발 환경 완전 구성
- Python bindings를 통한 시뮬레이션 실행
- MJCF XML 모델 작성 능력 습득
- 기본 시뮬레이션 시각화 및 데이터 수집

---

## 1. 환경 구축 (Environment Setup)

### 1.1 시스템 요구사항

```
- OS: Windows 10/11 (WSL2 권장) 또는 Ubuntu 22.04
- Python: 3.10 이상
- GPU: 권장 (CUDA 11.8+, 시각화 가속)
- RAM: 최소 8GB (16GB 권장)
```

### 1.2 MuJoCo 설치

#### 방법 1: pip 설치 (권장)
```bash
# MuJoCo Python 바인딩 설치
pip install mujoco

# 시각화 뷰어 설치
pip install mujoco-python-viewer

# 확인
python -c "import mujoco; print(mujoco.__version__)"
```

#### 방법 2: 소스 빌드
```bash
# Linux/WSL2
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Python 빌드
cd ../python
pip install -e .
```

### 1.3 개발 도구 세팅

```bash
# Jupyter Notebook (시각화 테스트용)
pip install jupyter matplotlib numpy

# Git 초기화
git init
echo "*.pyc" >> .gitignore
echo "__pycache__/" >> .gitignore
echo ".vscode/" >> .gitignore
git add .
git commit -m "Initial commit: MuJoCo project setup"
```

### 1.4 VS Code 확장 프로그램
- Python (Microsoft)
- Pylance
- Jupyter
- GitLens

---

## 2. MJCF 모델링 기초

### 2.1 MJCF XML 구조

MuJoCo는 MJCF (MuJoCo XML Format)를 사용합니다.

```xml
<mujoco model="example">
    <!-- 기본 설정 -->
    <option timestep="0.002" gravity="0 0 -9.81"/>
    
    <default>
        <joint damping="0.1"/>
        <geom condim="3"/>
    </default>

    <asset>
        <texture name="texplane" type="2d" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2" width="512" height="512"/>
        <material name="matplane" texture="texplane" texrepeat="5 5"/>
    </asset>

    <worldbody>
        <!-- 바닥면 -->
        <geom name="floor" type="plane" size="5 5 0.1" material="matplane"/>
        
        <!-- 빛 -->
        <light diffuse="0.8 0.8 0.8" pos="0 0 4"/>
        
        <!-- 본(body) 정의 -->
        <body name="link1" pos="0 0 0.5">
            <geom type="cylinder" size="0.05 0.3" rgba="1 0 0 1"/>
            <joint name="joint1" type="hinge" axis="0 0 1" range="-180 180"/>
            
            <body name="link2" pos="0 0 0.3">
                <geom type="cylinder" size="0.04 0.25" rgba="0 1 0 1"/>
                <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
            </body>
        </body>
    </worldbody>

    <!-- 액추레이터 -->
    <actuator>
        <position joint="joint1" kp="100"/>
        <position joint="joint2" kp="100"/>
    </actuator>

    <!-- 센서 -->
    <sensor>
        <jointpos joint="joint1"/>
        <jointvel joint="joint1"/>
        <force sensor="force_sensor"/>
    </sensor>
</mujoco>
```

### 2.2 핵심 요소 설명

| 요소 | 설명 |
|------|------|
| `<option>` | 시뮬레이션 옵션 (시간 간격, 중력 등) |
| `<worldbody>` | 모든 본(body)의 최상위 컨테이너 |
| `<body>` | 개별 림크 정의 (위치, 관성 등) |
| `<geom>` | 기하학적 형태 (cylinder, box, sphere 등) |
| `<joint>` | 조인트 정의 (hinge, slide, ball) |
| `<actuator>` | 구동기 정의 (position, velocity, torque) |
| `<sensor>` | 센서 정의 (위치, 속도, 힘 등) |
| `<default>` | 기본값 설정 |

---

## 3. 첫 번째 시뮬레이션

### 3.1 예제 1: 단순 펜듈럼

```python
import mujoco
import mujoco.viewer
import numpy as np

# MJCF 모델 문자열
xml = """
<mujoco>
    <option timestep="0.01" gravity="0 0 -9.81"/>
    
    <worldbody>
        <light diffuse="0.8 0.8 0.8" pos="0 0 3"/>
        <geom type="plane" size="2 2 0.1" rgba="0.9 0.9 0.9 1"/>
        
        <body name="pendulum" pos="0 0 1">
            <joint name="pivot" type="hinge" axis="0 1 0" range="-180 180"/>
            <geom type="sphere" size="0.1" rgba="1 0 0 1" mass="1"/>
            <geom type="cylinder" size="0.02 0.5" fromto="0 0 0 0 0 -0.5" rgba="0.5 0.5 0.5 1"/>
        </body>
    </worldbody>
</mujoco>
"""

# 모델 로드
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 초기 각도 설정 (45도)
data.qpos[0] = np.deg2rad(45)

# 시뮬레이션 루프
mujoco.viewer.launch(model, data)
```

### 3.2 예제 2: Cart-Pole

```python
import mujoco
import mujoco.viewer
import numpy as np

xml = """
<mujoco model="cart-pole">
    <option timestep="0.01" gravity="0 0 -9.81"/>
    
    <worldbody>
        <light diffuse="0.8 0.8 0.8" pos="0 0 3"/>
        <geom type="plane" size="5 0.1 0.1" rgba="0.9 0.9 0.9 1"/>
        
        <!-- 카트 -->
        <body name="cart" pos="0 0 0.3">
            <geom type="box" size="0.2 0.1 0.1" rgba="0.2 0.6 1 1" mass="1"/>
            <joint name="slider" type="slide" axis="1 0 0" range="-2 2"/>
            
            <!-- 폴 -->
            <body name="pole" pos="0 0 0">
                <joint name="hinge" type="hinge" axis="0 1 0" range="-180 180"/>
                <geom type="cylinder" size="0.03 0.5" fromto="0 0 0 0 0 1" rgba="1 0.5 0 1" mass="0.1"/>
                <geom type="sphere" size="0.05" pos="0 0 1" rgba="1 0 0 1"/>
            </body>
        </body>
    </worldbody>
    
    <actuator>
        <position joint="slider" kp="100" ctrlrange="-50 50"/>
    </actuator>
    
    <sensor>
        <jointpos joint="slider"/>
        <jointpos joint="hinge"/>
    </sensor>
</mujoco>
"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# 폴을 약간 기울임
data.qpos[1] = np.deg2rad(10)

mujoco.viewer.launch(model, data)
```

---

## 4. MJCF 심화 요소

### 4.1 조인트 타입

```xml
<!--HINGE (회전 조인트) -->
<joint name="revolute" type="hinge" axis="0 0 1" range="-180 180" damping="0.1"/>

<!-- SLIDE (선형 조인트) -->
<joint name="prismatic" type="slide" axis="1 0 0" range="-1 1"/>

<!-- BALL (_ball-and-socket 조인트) -->
<joint name="free" type="ball"/>
```

### 4.2 기하학적 형태 (Geom)

```xml
<!--원통 -->
<geom type="cylinder" size="0.05 0.3"/>

<!--상자 -->
<geom type="box" size="0.1 0.1 0.1"/>

<!--구 -->
<geom type="sphere" size="0.05"/>

<!--Capsule (원통 + 반구) -->
<geom type="capsule" size="0.05 0.2"/>

<!--메시 (OBJ 파일) -->
<geom type="mesh" mesh="robot_arm" scale="1 1 1"/>
```

### 4.3 액추레이터 타입

```xml
<!-- 위치 제어 -->
<position joint="joint1" kp="100" ctrlrange="-100 100"/>

<!-- 속도 제어 -->
<velocity joint="joint2" kv="10" ctrlrange="-50 50"/>

<!-- 토크 제어 -->
<general joint="joint3" gainprm="1" biasprm="0 0 0" ctrlrange="-20 20"/>

<!-- muscle (Bio-inspired) -->
<muscle joint="joint1" gain="1" bias="0" range="0 1"/>
```

---

## 5. 실습 과제

### 과제 1: Cart-Pole 시뮬레이션
- [ ] 위의 Cart-Pole 모델을 실행하고 시각화
- [ ] 폴의 초기 각도를 0°, 10°, 30°로 변경하며 관찰
- [ ] 카트에 힘을 인가하여 폴을 세우는 간단한 제어 구현

```python
# 힌트: PD 제어기
def simple_controller(data, target_angle=0):
    angle = data.qpos[1]  # 폴 각도
    angular_vel = data.qvel[1]  # 폴 각속도
    
    # PD 제어
    kp = 50
    kd = 10
    control = kp * (target_angle - angle) - kd * angular_vel
    
    data.ctrl[0] = control
```

### 과제 2: 2-DOF 매니퓰레이터
- [ ] 다음 모델을 MJCF로 작성:

```
    Link 1 (0.3m)
       |
    Joint 1 (HINGE, Z축)
       |
    Link 2 (0.25m)
       |
    Joint 2 (HINGE, Z축)
       |
    End Effector
```

- [ ] 각 관절 범위: -180° ~ 180°
- [ ] 엔드이펙터에 작은 구체(phisical=0.02) 추가

### 과제 3: 시각화 및 데이터 수집
- [ ] 시뮬레이션 중 관절 위치(qpos)를 리스트에 저장
- [ ] matplotlib으로 시간에 따른 관절 각도 그래프 출력
- [ ] 액추레이터 제어값(ctrl)도 함께 그래프 출력

---

## 6. 핵심 API 참고

```python
import mujoco

# 모델 로드
model = mujoco.MjModel.from_xml_string(xml)  # 문자열에서
model = mujoco.MjModel.from_xml_path("model.xml")  # 파일에서

# 데이터 객체
data = mujoco.MjData(model)

# 시뮬레이션 스텝
mujoco.mj_step(model, data)

# forward dynamics 계산
mujoco.mj_forward(model, data)

# 현재 상태 출력
print("관절 위치:", data.qpos)
print("관절 속도:", data.qvel)
print("액추레이터 힘:", data.ctrl)

# 상태 설정
data.qpos[0] = value  # 관절 위치 설정
data.ctrl[0] = value  # 액추레이터 입력 설정

# 시각화
mujoco.viewer.launch(model, data)  # 인터랙티브 뷰어
```

---

## 7. 다음 단계 미리보기

Day 3-4에서는:
- Forward/Inverse Dynamics API深入理解
- PD/PID/Computed Torque 제어 알고리즘 구현
- Operational Space Control (OSC) 기초

**오늘의 핵심 포인트**: MJCF XML 구조를 이해하고, 기본 시뮬레이션을 Python에서 실행할 수 있어야 합니다.

---

*학습 시간: 약 6-8시간 (2일)*
*난이도: ★☆☆☆☆ (입문)*
