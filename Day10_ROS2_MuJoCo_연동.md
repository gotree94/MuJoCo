# Day 10: ROS2 + MuJoCo 연동

## 학습 목표
- ROS2 Humble 환경 구축
- MuJoCo ↔ ROS2 통신 인터페이스 구축
- 실시간 시각화 및 모니터링
- ros2_control 기반 제어 시스템 통합

---

## 1. ROS2 기초 복습

### 1.1 ROS2 핵심 개념

```
ROS2 아키텍처:
- Node: 기본 실행 단위
- Topic: 비동기 통신 채널
- Service: 동기 통신
- Action: 장기 작업 (피드백 포함)
- Parameter: 노드 설정값
- Launch: 여러 노드 동시 실행
```

### 1.2 ROS2 Humble 설치 (Ubuntu 22.04)

```bash
# 기본 의존성
sudo apt update && sudo apt install -y curl gnupg lsb-release

# ROS2 Humble 키 추가
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# ROS2 Humble 데스크탑 (전체 설치)
sudo apt install -y ros-humble-desktop

# 개발 도구
sudo apt install -y ros-humble-dev python3-colcon-common-extensions

# 환경 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 확인
ros2 --help
```

### 1.3 WSL2에서 ROS2 실행 (Windows 사용자)

```bash
# WSL2에서 Ubuntu 22.04 실행 후 위의 설치 진행
# Windows Terminal에서:
wsl -d Ubuntu-22.04

# 또는 PowerShell에서:
wsl --exec bash -c "source /opt/ros/humble/setup.bash && ros2 run demo_nodes_cpp talker"
```

---

## 2. MuJoCo-ROS2 브릿지 구축

### 2.1 아키텍처

```
MuJoCo 시뮬레이터
    │
    ▼
ROS2 Bridge Node
    │
    ├── Publisher: JointState (/joint_states)
    ├── Publisher: TF (/tf)
    ├── Publisher: Odometry (/odom)
    ├── Publisher: Sensor Data (/sensor/*)
    ├── Subscriber: cmd_vel (/cmd_vel)
    └── Subscriber: JointCommand (/joint_command)
```

### 2.2 기본 브릿지 노드 구현

```python
#!/usr/bin/env python3
"""
MuJoCo-ROS2 Bridge Node
MuJoCo 시뮬레이션 데이터를 ROS2 토픽으로 발행
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import mujoco
import numpy as np
import threading
import time

# ROS2 메시지
from sensor_msgs.msg import JointState, Imu, Touch
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class MuJoCoROS2Bridge(Node):
    def __init__(self, model_path):
        super().__init__('mujoco_ros2_bridge')
        
        # MuJoCo 초기화
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # QoS 프로파일
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publisher
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', qos)
        
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', qos)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, qos)
        
        # 타이머 (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        # 시뮬레이션 스레드
        self.sim_thread = threading.Thread(target=self.simulation_loop)
        self.sim_thread.daemon = True
        self.sim_thread.start()
        
        self.get_logger().info('MuJoCo-ROS2 Bridge started')
    
    def cmd_vel_callback(self, msg):
        """속도 명령 수신"""
        # 로봇 속도 설정
        self.data.ctrl[0] = msg.linear.x
        self.data.ctrl[1] = msg.angular.z
    
    def timer_callback(self):
        """주기적 데이터 발행"""
        # JointState 메시지
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # 관절 이름 및 값
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                joint_state.name.append(name)
                joint_state.position.append(float(self.data.qpos[i]))
                joint_state.velocity.append(float(self.data.qvel[i]))
                joint_state.effort.append(float(self.data.ctrl[i]))
        
        self.joint_state_pub.publish(joint_state)
        
        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # 위치
        odom.pose.pose.position.x = float(self.data.qpos[0])
        odom.pose.pose.position.y = float(self.data.qpos[1])
        odom.pose.pose.position.z = float(self.data.qpos[2])
        
        # 속도
        odom.twist.twist.linear.x = float(self.data.qvel[0])
        odom.twist.twist.linear.y = float(self.data.qvel[1])
        odom.twist.twist.angular.z = float(self.data.qvel[2])
        
        self.odom_pub.publish(odom)
        
        # TF 발행
        self.publish_tf()
    
    def publish_tf(self):
        """TF 트랜스폼 발행"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = float(self.data.qpos[0])
        t.transform.translation.y = float(self.data.qpos[1])
        t.transform.translation.z = float(self.data.qpos[2])
        
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)
    
    def simulation_loop(self):
        """MuJoCo 시뮬레이션 루프"""
        while rclpy.ok():
            mujoco.mj_step(self.model, self.data)
            time.sleep(self.model.opt.timestep)

def main(args=None):
    rclpy.init(args=args)
    
    node = MuJoCoROS2Bridge('robot.xml')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.3 ROS2 패키지 구조

```
mujoco_ros2_bridge/
├── package.xml
├── setup.py
├── setup.cfg
├── launch/
│   └── bridge_launch.py
├── mujoco_ros2_bridge/
│   ├── __init__.py
│   └── bridge_node.py
└── models/
    └── robot.xml
```

### 2.4 package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mujoco_ros2_bridge</name>
  <version>0.1.0</version>
  <description>MuJoCo-ROS2 Bridge Package</description>
  <maintainer email="user@todo.com">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 2.5 Launch 파일

```python
# launch/bridge_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mujoco_ros2_bridge',
            executable='bridge_node',
            name='mujoco_bridge',
            output='screen',
            parameters=[{
                'model_path': '/path/to/robot.xml'
            }]
        )
    ])
```

---

## 3. URDF + TF 시각화

### 3.1 Robot State Publisher

```python
# launch/visualization_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='/path/to/robot.urdf'
        ),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[LaunchConfiguration('model')]
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_gui': True}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/path/to/config.rviz']
        )
    ])
```

### 3.2 RViz2 설정

```yaml
# config.rviz
Panels:
  - Class: rviz_common/Displays
    Displays:
      - Class: rviz_default_plugins/TF
        Enabled: true
      - Class: rviz_default_plugins/RobotModel
        Enabled: true
      - Class: rviz_default_plugins/Map
        Enabled: true

Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 5
    Focal Point: [0, 0, 0]
```

---

## 4. 실시간 모니터링

### 4.1 센서 데이터 시각화

```python
#!/usr/bin/env python3
"""
센서 데이터 시각화 노드
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class SensorVisualizer(Node):
    def __init__(self):
        super().__init__('sensor_visualizer')
        
        # 데이터 버퍼
        self.joint_pos_buffer = deque(maxlen=100)
        self.joint_vel_buffer = deque(maxlen=100)
        self.time_buffer = deque(maxlen=100)
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # 타이머 (시각화 업데이트)
        self.timer = self.create_timer(0.1, self.update_plot)
        
        # matplotlib 설정
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 8))
    
    def joint_callback(self, msg):
        """관절 데이터 수신"""
        self.time_buffer.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        self.joint_pos_buffer.append(msg.position)
        self.joint_vel_buffer.append(msg.velocity)
    
    def update_plot(self):
        """그래프 업데이트"""
        if len(self.time_buffer) < 2:
            return
        
        times = list(self.time_buffer)
        
        # 위치 그래프
        self.axes[0].clear()
        positions = np.array(self.joint_pos_buffer)
        for i in range(positions.shape[1]):
            self.axes[0].plot(times, positions[:, i], label=f'Joint {i}')
        self.axes[0].set_xlabel('Time (s)')
        self.axes[0].set_ylabel('Position (rad)')
        self.axes[0].set_title('Joint Positions')
        self.axes[0].legend()
        self.axes[0].grid(True)
        
        # 속도 그래프
        self.axes[1].clear()
        velocities = np.array(self.joint_vel_buffer)
        for i in range(velocities.shape[1]):
            self.axes[1].plot(times, velocities[:, i], label=f'Joint {i}')
        self.axes[1].set_xlabel('Time (s)')
        self.axes[1].set_ylabel('Velocity (rad/s)')
        self.axes[1].set_title('Joint Velocities')
        self.axes[1].legend()
        self.axes[1].grid(True)
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = SensorVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 커스텀 메시지 정의

```bash
# msg/JointCommand.msg
std_msgs/Header header
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
```

---

## 5. ros2_control 통합

### 5.1 ros2_control 구성

```xml
<!-- robot.ros2_control.xacro -->
<ros2_control name="mujoco_system" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystem</plugin>
  </hardware>

  <joint name="joint1">
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <joint name="joint2">
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### 5.2 컨트롤러 로드

```bash
# 컨트롤러 매니저 실행
ros2 run controller_manager controller_manager spawner joint_state_broadcaster
ros2 run controller_manager controller_manager spawner forward_command_controller

# 컨트롤러 확인
ros2 control list_controllers

# 컨트롤러 활성화
ros2 control set_controller_state joint_state_broadcaster active
```

---

## 6. 실습 과제

### 과제 1: 기본 브릿지 구현
- [ ] 위의 MuJoCo-ROS2 Bridge 노드 구현
- [ ] JointState 토픽 발행 확인
- [ ] `ros2 topic echo /joint_states` 로 데이터 확인

### 과제 2: RViz2 시각화
- [ ] Robot State Publisher로 URDF 시각화
- [ ] TF 트랜스폼 확인
- [ ] Joint State Publisher GUI로 관절 제어

### 과제 3: 원격 제어
- [ ] cmd_velSubscriber로 로봇 이동 제어
- [ ] 키보드 teleop으로 제어 (teleop_twist_keyboard)
- [ ] Pi小姐 (Pi controller)를 ROS2 서비스로 구현

### 과제 4: 센서 데이터 수집
- [ ] 센서 데이터를 rosbag2으로 녹화
- [ ] 녹화된 데이터 분석
- [ ] 실시간 데이터 그래프 출력

---

## 7. 디버깅 팁

### 7.1 일반적인 문제 해결

```bash
# 문제: 토픽이 발행되지 않음
ros2 topic list
ros2 topic hz /joint_states

# 문제: TF 누락
ros2 run tf2_tools view_frames

# 문제: 노드 충돌
ros2 node list
ros2 node info /mujoco_bridge
```

### 7.2 로그 확인

```bash
# ROS2 로그 확인
ros2 run rclcpp_components component_container

# MuJoCo 로그
export MUJOCO_LOG=1
```

---

## 8. 다음 단계 미리보기

Day 11-12에서는:
- 모방학습 (Imitation Learning) 기초
- Behavior Cloning 구현
- Demonstration 데이터 수집

**오늘의 핵심 포인트**: ROS2와 MuJoCo를 연결하고, 실시간으로 데이터를 주고받을 수 있어야 합니다.

---

*학습 시간: 약 6-8시간 (1일)*
*난이도: ★★★☆☆ (중급)*
