# Day 6-7: URDF/MJCF 변환 및 로봇 모델링

## 학습 목표
- URDF 파일 구조 이해 및 MJCF 변환 능력
- 복잡한 로봇 모델 (매니퓰레이터, 휴머노이드) 구성
- 관절 제약, 엔드이펙터, 센서 설정
- 모델 검증 및 시각화

---

## 1. URDF vs MJCF 비교

| 특성 | URDF | MJCF |
|------|------|------|
| 출처 | ROS/ROS2 표준 | MuJoCo 전용 |
| 제약 조건 | 제한적 | 풍부 (equality, tendon 등) |
| 액추레이터 | 기본적 | 다양한 타입 지원 |
| 센서 | 제한적 | 풍부한 센서 지원 |
| 시뮬레이션 | Gazebo, Isaac | MuJoCo 전용 |

### 1.1 URDF 기본 구조

```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <!-- 링크 정의 -->
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.2"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1.0"/>
            <inertia ixx="0.01" iyy="0.01" izz="0.01" 
                     ixy="0" ixz="0" iyz="0"/>
        </inertial>
    </link>
    
    <!-- 조인트 정의 -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
    </joint>
</robot>
```

---

## 2. URDF → MJCF 변환

### 2.1 방법 1: Python 스크립트 (수동)

```python
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

def urdf_to_mjcf(urdf_path, output_path):
    """URDF를 MJCF로 변환하는 기본 스크립트"""
    
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # MJCF 템플릿
    mjcf = """<?xml version="1.0" ?>
<mujoco model="converted_robot">
    <option timestep="0.002" gravity="0 0 -9.81"/>
    
    <default>
        <joint damping="0.1"/>
        <geom condim="3"/>
    </default>
    
    <worldbody>
        <light diffuse="0.8 0.8 0.8" pos="0 0 3"/>
        <geom type="plane" size="5 5 0.1" rgba="0.9 0.9 0.9 1"/>
"""
    
    # URDF 파싱
    links = {}
    joints = {}
    
    for link in root.findall('link'):
        link_name = link.get('name')
        links[link_name] = {
            'visual': link.find('visual'),
            'collision': link.find('collision'),
            'inertial': link.find('inertial')
        }
    
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        joints[joint_name] = {
            'type': joint.get('type'),
            'parent': joint.find('parent').get('link'),
            'child': joint.find('child').get('link'),
            'origin': joint.find('origin'),
            'axis': joint.find('axis'),
            'limit': joint.find('limit')
        }
    
    # 기본 링크 (base_link) 처리
    base_link = 'base_link'
    mjcf += f'        <body name="{base_link}" pos="0 0 0.5">\n'
    
    # 링크별 geom 추가
    def add_link_geom(link_name, indent=12):
        link = links[link_name]
        visual = link['visual']
        
        if visual is not None:
            geom = visual.find('geometry')
            if geom is not None:
                if geom.find('cylinder') is not None:
                    cyl = geom.find('cylinder')
                    radius = cyl.get('radius', '0.05')
                    length = cyl.get('length', '0.1')
                    return f'{" " * indent}<geom type="cylinder" size="{radius} {float(length)/2}" rgba="0.5 0.5 0.5 1"/>\n'
                elif geom.find('box') is not None:
                    box = geom.find('box')
                    size = box.get('size', '0.05 0.05 0.05')
                    sizes = [float(s)/2 for s in size.split()]
                    return f'{" " * indent}<geom type="box" size="{" ".join(map(str, sizes))}" rgba="0.5 0.5 0.5 1"/>\n'
        return ""
    
    mjcf += add_link_geom(base_link)
    
    # 조인트 및 자식 링크 추가
    for joint_name, joint in joints.items():
        if joint['parent'] == base_link:
            child = joint['child']
            
            # 조인트 타입 변환
            joint_type = joint['type']
            if joint_type == 'revolute':
                mjcf_joint = 'hinge'
            elif joint_type == 'prismatic':
                mjcf_joint = 'slide'
            else:
                mjcf_joint = 'hinge'
            
            # 위치 설정
            origin = joint['origin']
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0')
            else:
                xyz = '0 0 0'
            
            # 축 설정
            axis = joint['axis']
            if axis is not None:
                axis_xyz = axis.get('xyz', '0 0 1')
            else:
                axis_xyz = '0 0 1'
            
            # 제한
            limit = joint['limit']
            if limit is not None:
                lower = float(limit.get('lower', '-3.14'))
                upper = float(limit.get('upper', '3.14'))
            else:
                lower, upper = -3.14, 3.14
            
            mjcf += f'            <body name="{child}" pos="{xyz}">\n'
            mjcf += f'                <joint name="{joint_name}" type="{mjcf_joint}" axis="{axis_xyz}" range="{lower} {upper}"/>\n'
            mjcf += add_link_geom(child, 16)
            mjcf += '            </body>\n'
    
    mjcf += """        </body>
    </worldbody>
    
    <actuator>
"""
    
    # 액추레이터 추가
    for joint_name, joint in joints.items():
        if joint['type'] in ['revolute', 'prismatic']:
            mjcf += f'        <position joint="{joint_name}" kp="100"/>\n'
    
    mjcf += """    </actuator>
</mujoco>
"""
    
    # 저장
    with open(output_path, 'w') as f:
        f.write(mjcf)
    
    print(f"변환 완료: {output_path}")

# 사용 예시
# urdf_to_mjcf("robot.urdf", "robot.xml")
```

### 2.2 방법 2: 온라인 변환 도구

1. **urdf2mjcf** (공식): https://github.com/google-deepmind/mujoco/tree/main/python/mujoco/urdf
2. **Mujoco URDF Parser**: https://mujoco.org/book/models.html#URDF

```bash
# 공식 변환기 사용
pip install urdf-parserpy

python -c "
from mujoco import urdf
model = urdf.load('robot.urdf')
# model은 MjModel 객체
"
```

### 2.3 방법 3: 로봇 저작 도구

```bash
# Roboworks, Fusion360에서 직접 MJCF 내보내기
# 또는 ROS의 xacro → urdf → mjcf 파이프라인
xacro robot.xacro > robot.urdf
# 그런 다음 urdf2mjcf 사용
```

---

## 3. 7-DOF 매니퓰레이터 모델 구축

### 3.1 Franka Emika Panda 유사 모델

```xml
<?xml version="1.0" ?>
<mujoco model="7dof_manipulator">
    <option timestep="0.002" gravity="0 0 -9.81"/>
    
    <default>
        <joint damping="0.5" armature="0.1"/>
        <geom condim="3" friction="1 0.5 0.01"/>
        <position kp="200" kv="20"/>
    </default>
    
    <asset>
        <texture name="texplane" type="2d" builtin="checker" 
                 rgb1="0.2 0.3 0.4" rgb2="0.1 0.15 0.2" width="512" height="512"/>
        <material name="matplane" texture="texplane" texrepeat="5 5"/>
        <material name="robot_mat" rgba="0.8 0.8 0.8 1"/>
    </asset>
    
    <worldbody>
        <light diffuse="0.8 0.8 0.8" pos="0 0 4" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="5 5 0.1" material="matplane"/>
        
        <!-- 베이스 -->
        <body name="base" pos="0 0 0">
            <geom type="cylinder" size="0.08 0.05" material="robot_mat"/>
            
            <!-- Joint 1: Base rotation -->
            <body name="link1" pos="0 0 0.05">
                <joint name="joint1" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
                <geom type="cylinder" size="0.06 0.04" material="robot_mat"/>
                
                <!-- Joint 2: Shoulder -->
                <body name="link2" pos="0 0 0.04">
                    <joint name="joint2" type="hinge" axis="0 1 0" range="-1.7628 1.7628"/>
                    <geom type="capsule" fromto="0 0 0 0 0 0.15" size="0.04" material="robot_mat"/>
                    
                    <!-- Joint 3: Elbow 1 -->
                    <body name="link3" pos="0 0 0.15">
                        <joint name="joint3" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
                        <geom type="cylinder" size="0.035 0.03" material="robot_mat"/>
                        
                        <!-- Joint 4: Elbow 2 -->
                        <body name="link4" pos="0 0 0.03">
                            <joint name="joint4" type="hinge" axis="0 1 0" range="-3.0718 -0.0698"/>
                            <geom type="capsule" fromto="0 0 0 0 0 0.12" size="0.035" material="robot_mat"/>
                            
                            <!-- Joint 5: Wrist 1 -->
                            <body name="link5" pos="0 0 0.12">
                                <joint name="joint5" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
                                <geom type="cylinder" size="0.03 0.025" material="robot_mat"/>
                                
                                <!-- Joint 6: Wrist 2 -->
                                <body name="link6" pos="0 0 0.025">
                                    <joint name="joint6" type="hinge" axis="0 1 0" range="-0.0175 3.7525"/>
                                    <geom type="capsule" fromto="0 0 0 0 0 0.08" size="0.025" material="robot_mat"/>
                                    
                                    <!-- Joint 7: Wrist 3 -->
                                    <body name="link7" pos="0 0 0.08">
                                        <joint name="joint7" type="hinge" axis="0 0 1" range="-2.8973 2.8973"/>
                                        <geom type="cylinder" size="0.02 0.02" material="robot_mat"/>
                                        
                                        <!-- End Effector -->
                                        <body name="ee_link" pos="0 0 0.04">
                                            <geom type="sphere" size="0.015" rgba="1 0 0 1"/>
                                            <site name="ee_site" pos="0 0 0.02" size="0.005" rgba="1 0 0 0.5"/>
                                        </body>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>
    
    <actuator>
        <position joint="joint1"/>
        <position joint="joint2"/>
        <position joint="joint3"/>
        <position joint="joint4"/>
        <position joint="joint5"/>
        <position joint="joint6"/>
        <position joint="joint7"/>
    </actuator>
    
    <sensor>
        <jointpos joint="joint1"/>
        <jointpos joint="joint2"/>
        <jointpos joint="joint3"/>
        <jointpos joint="joint4"/>
        <jointpos joint="joint5"/>
        <jointpos joint="joint6"/>
        <jointpos joint="joint7"/>
        
        <jointvel joint="joint1"/>
        <jointvel joint="joint2"/>
        <jointvel joint="joint3"/>
        <jointvel joint="joint4"/>
        <jointvel joint="joint5"/>
        <jointvel joint="joint6"/>
        <jointvel joint="joint7"/>
        
        <force site="ee_site"/>
    </sensor>
</mujoco>
```

### 3.2关节 제약 조건 설정

```xml
<!-- 관절 제한 상세 설정 -->
<joint name="joint1" type="hinge" 
       axis="0 0 1" 
       range="-2.8973 2.8973"
       damping="0.5"
       armature="0.1"
       frictionloss="0.1"
       limited="true"/>

<!-- tendon을 이용한 복합 제약 -->
<tendon>
    <spatial name="tendon1" limited="true" range="0 0.5">
        <site site="site1"/>
        <site site="site2"/>
    </spatial>
</tendon>

<!-- equality 제약 -->
<equality>
    <joint joint1="joint1" joint2="joint2" polycoef="0 1 0 0 0"/>
</equality>
```

---

## 4. 센서 패키지 통합

### 4.1 센서 정의

```xml
<sensor>
    <!-- 관절 위치 센서 -->
    <jointpos joint="joint1" name="joint1_pos"/>
    <jointpos joint="joint2" name="joint2_pos"/>
    
    <!-- 관절 속도 센서 -->
    <jointvel joint="joint1" name="joint1_vel"/>
    
    <!-- 관절 토크 센서 -->
    <jointtorque joint="joint1" name="joint1_torque"/>
    
    <!-- 힘 센서 -->
    <force site="ee_site" name="ee_force"/>
    
    <!-- 토크 센서 -->
    <torque site="ee_site" name="ee_torque"/>
    
    <!-- IMU 센서 -->
    <framequat objtype="body" objname="link7" name="ee_quat"/>
    <gyro objtype="body" objname="link7" name="ee_gyro"/>
    <accelerometer objtype="body" objname="link7" name="ee_accel"/>
    
    <!-- 터치 센서 -->
    <touch site="touch_site" name="touch"/>
</sensor>

<!-- 센서 사이트 정의 -->
<site name="touch_site" pos="0 0 0" size="0.01" type="sphere"/>
```

### 4.2 센서 데이터 읽기

```python
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("7dof_robot.xml")
data = mujoco.MjData(model)

mujoco.mj_forward(model, data)

# 센서 데이터 읽기
def get_sensor_data(model, data, sensor_name):
    """이름으로 센서 데이터 읽기"""
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    
    if sensor_id == -1:
        return None
    
    adr = model.sensor_adr[sensor_id]
    dim = model.sensor_dim[sensor_id]
    
    return data.sensordata[adr:adr+dim]

# 사용 예시
joint1_pos = get_sensor_data(model, data, "joint1_pos")
ee_force = get_sensor_data(model, data, "ee_force")
print(f"Joint 1 위치: {joint1_pos}")
print(f"엔드이펙터 힘: {ee_force}")
```

---

## 5. 휴머노이드 상체 모델

### 5.1 기본 휴머노이드 상체

```xml
<?xml version="1.0" ?>
<mujoco model="humanoid_upper_body">
    <option timestep="0.002" gravity="0 0 -9.81"/>
    
    <default>
        <joint damping="0.5"/>
        <geom condim="3"/>
    </default>
    
    <worldbody>
        <light diffuse="0.8 0.8 0.8" pos="0 0 3"/>
        <geom type="plane" size="5 5 0.1"/>
        
        <!-- 골반 -->
        <body name="pelvis" pos="0 0 0.9">
            <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.1" rgba="0.8 0.6 0.4 1"/>
            
            <!-- 척추 -->
            <body name="spine" pos="0 0 0.1">
                <joint name="spine_pan" type="hinge" axis="0 0 1" range="-0.5 0.5"/>
                <joint name="spine_tilt" type="hinge" axis="1 0 0" range="-0.5 0.5"/>
                <geom type="capsule" fromto="0 0 0 0 0 0.2" size="0.08" rgba="0.8 0.6 0.4 1"/>
                
                <!-- 가슴 -->
                <body name="chest" pos="0 0 0.2">
                    <geom type="capsule" fromto="-0.15 0 0 0.15 0 0" size="0.06" rgba="0.8 0.6 0.4 1"/>
                    
                    <!-- 왼쪽 어깨 -->
                    <body name="left_shoulder" pos="-0.15 0 0">
                        <joint name="l_shoulder_pan" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
                        <joint name="l_shoulder_lift" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                        <geom type="sphere" size="0.05" rgba="0.8 0.6 0.4 1"/>
                        
                        <!-- 왼쪽 팔 -->
                        <body name="left_upper_arm" pos="-0.05 0 0">
                            <joint name="l_elbow" type="hinge" axis="0 1 0" range="0 2.5"/>
                            <geom type="capsule" fromto="0 0 0 0 0 -0.25" size="0.04" rgba="0.8 0.6 0.4 1"/>
                            
                            <!-- 왼쪽 전완 -->
                            <body name="left_forearm" pos="0 0 -0.25">
                                <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.03" rgba="0.8 0.6 0.4 1"/>
                                
                                <!-- 왼쪽 손 -->
                                <body name="left_hand" pos="0 0 -0.2">
                                    <geom type="box" size="0.04 0.03 0.05" rgba="0.8 0.6 0.4 1"/>
                                    <site name="l_hand_site" pos="0 0 -0.05" size="0.01"/>
                                </body>
                            </body>
                        </body>
                    </body>
                    
                    <!-- 오른쪽 어깨 (대칭) -->
                    <body name="right_shoulder" pos="0.15 0 0">
                        <joint name="r_shoulder_pan" type="hinge" axis="0 1 0" range="-3.14 3.14"/>
                        <joint name="r_shoulder_lift" type="hinge" axis="0 0 1" range="-3.14 3.14"/>
                        <geom type="sphere" size="0.05" rgba="0.8 0.6 0.4 1"/>
                        
                        <body name="right_upper_arm" pos="0.05 0 0">
                            <joint name="r_elbow" type="hinge" axis="0 1 0" range="0 2.5"/>
                            <geom type="capsule" fromto="0 0 0 0 0 -0.25" size="0.04" rgba="0.8 0.6 0.4 1"/>
                            
                            <body name="right_forearm" pos="0 0 -0.25">
                                <geom type="capsule" fromto="0 0 0 0 0 -0.2" size="0.03" rgba="0.8 0.6 0.4 1"/>
                                
                                <body name="right_hand" pos="0 0 -0.2">
                                    <geom type="box" size="0.04 0.03 0.05" rgba="0.8 0.6 0.4 1"/>
                                    <site name="r_hand_site" pos="0 0 -0.05" size="0.01"/>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>
    
    <actuator>
        <position joint="spine_pan" kp="100"/>
        <position joint="spine_tilt" kp="100"/>
        <position joint="l_shoulder_pan" kp="50"/>
        <position joint="l_shoulder_lift" kp="50"/>
        <position joint="l_elbow" kp="50"/>
        <position joint="r_shoulder_pan" kp="50"/>
        <position joint="r_shoulder_lift" kp="50"/>
        <position joint="r_elbow" kp="50"/>
    </actuator>
    
    <sensor>
        <jointpos joint="spine_pan"/>
        <jointpos joint="l_shoulder_pan"/>
        <jointpos joint="l_elbow"/>
        <force site="l_hand_site"/>
        <force site="r_hand_site"/>
    </sensor>
</mujoco>
```

---

## 6. 모델 검증

### 6.1 검증 체크리스트

```python
import mujoco
import numpy as np

def validate_model(model_path):
    """모델 검증 함수"""
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("=" * 50)
    print("모델 검증 시작")
    print("=" * 50)
    
    # 1. 기본 정보
    print(f"\n[기본 정보]")
    print(f"관절 수 (nq): {model.nq}")
    print(f"속도 수 (nv): {model.nv}")
    print(f"바디 수: {model.nbody}")
    print(f"조인트 수: {model.njnt}")
    print(f"지오metry 수: {model.ngeom}")
    print(f"액추레이터 수: {model.nu}")
    print(f"센서 수: {model.nsensor}")
    
    # 2. 관성 행렬 확인
    mujoco.mj_forward(model, data)
    M = data.qM.reshape(model.nv, model.nv)
    
    print(f"\n[관성 행렬]")
    print(f"대각선 원소 (질량): {np.diag(M)}")
    print(f"행렬 조건수: {np.linalg.cond(M):.2f}")
    
    if np.any(np.diag(M) <= 0):
        print("⚠️  경고: 음수 또는 영(0) 질량이 있습니다!")
    
    # 3. 자유도 확인
    print(f"\n[자유도]")
    print(f"자유 관절 (freejoint) 수: {model.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_CONSTRAINT}")
    
    # 4. 액추레이터 범위 확인
    print(f"\n[액추레이터]")
    for i in range(model.nu):
        ctrl_range = model.actuator_ctrlrange[i]
        print(f"  액추레이터 {i}: [{ctrl_range[0]:.2f}, {ctrl_range[1]:.2f}]")
    
    # 5. 센서 확인
    print(f"\n[센서]")
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        print(f"  센서 {i}: {name}")
    
    # 6. 시뮬레이션 테스트
    print(f"\n[시뮬레이션 테스트]")
    try:
        for _ in range(100):
            mujoco.mj_step(model, data)
        print("✅ 100 스텝 시뮬레이션 성공")
    except Exception as e:
        print(f"❌ 시뮬레이션 에러: {e}")
    
    # 7. 시각화 테스트
    print(f"\n[시각화 테스트]")
    try:
        import mujoco.viewer
        print("✅ mujoco.viewer 임포트 성공")
    except:
        print("⚠️  mujoco.viewer 사용 불가")
    
    print("\n" + "=" * 50)
    print("검증 완료")
    print("=" * 50)

# 실행
validate_model("7dof_robot.xml")
```

### 6.2 시각화 테스트

```python
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("7dof_robot.xml")
data = mujoco.MjData(model)

# 관절을 랜덤하게 움직여보기
for i in range(model.nu):
    data.ctrl[i] = np.random.uniform(-0.5, 0.5)

mujoco.viewer.launch(model, data)
```

---

## 7. 실습 과제

### 과제 1: URDF → MJCF 변환
- [ ] 위의 Python 변환 스크립트를 완성
- [ ] 단순 URDF 파일을 MJCF로 변환
- [ ] 변환된 모델 검증 및 시각화

### 과제 2: 7-DOF 매니퓰레이터 완성
- [ ] 위의 7-DOF 모델을 완성하고 테스트
- [ ] 모든 관절을逐一 제어하여 동작 확인
- [ ] 엔드이펙터 궤적 시각화

### 과제 3: 센서 데이터 수집
- [ ] 모든 센서의 데이터를 로깅하는 스크립트 작성
- [ ] 시뮬레이션 중 실시간으로 센서 데이터 그래프 출력
- [ ] 센서 데이터를 파일로 저장 (CSV, HDF5)

### 과제 4: 커스텀 로봇 모델
- [ ] 자신만의 로봇 모델을 설계 (최소 4-DOF)
- [ ] 모델 검증 함수로 테스트
- [ ] 기본 동작 (팔 뻗기, 집기 등) 시뮬레이션

---

## 8. 다음 단계 미리보기

Day 8-9에서는:
- Whole-Body Motion Planning
- 휴머노이드 보행 역학
- CoM/ZMP 기반 균형 제어

**오늘의 핵심 포인트**: URDF를 MJCF로 변환하고, 복잡한 로봇 모델을 구성할 수 있어야 합니다.

---

*학습 시간: 약 8-10시간 (2일)*
*난이도: ★★★☆☆ (중급)*
