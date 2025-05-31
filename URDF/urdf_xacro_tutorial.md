
#  URDF e XACRO in ROS 2 – Guida Introduttiva

##  Cos'è l'URDF?

URDF (*Unified Robot Description Format*) è un linguaggio basato su XML che permette di descrivere la struttura fisica e cinematica di un robot, definendo:

- **Link**: le parti rigide del robot (es. un braccio, una ruota, una base).
- **Joint**: le connessioni tra link, che possono essere mobili (es. rotazione) o fisse.
- **Inerzia, massa e collisioni**: proprietà fisiche per simulazione e pianificazione.
- **Mesh e visualizzazione**: modelli 3D (.dae, .stl) per RViz o Gazebo.

URDF è **statico**, e può diventare ripetitivo per robot complessi o modulari.

---

##  Cos'è XACRO?

XACRO (*XML Macro*) è un'estensione di XML che aggiunge **macro, variabili, include, condizioni** per rendere i file URDF più modulari, leggibili e riutilizzabili.

In ROS 2 è **lo standard de facto** per generare URDF da file `.xacro`.

---

##  Mini-tutorial: creiamo un robot semplice in XACRO

Ispirato al video [YouTube: Create URDF Robot Using Xacro](https://youtu.be/CwdbsvcpOHM), ecco come costruire un robot base con due link e un joint.

### 1. Struttura del pacchetto

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --dependencies xacro my_robot_description
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Crea le cartelle:

```bash
cd src/my_robot_description
mkdir urdf launch
```

---

### 2. File `robot.urdf.xacro`

Salva questo contenuto in `urdf/robot.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="simple_robot">

  <xacro:property name="link_length" value="1.0"/>
  <xacro:property name="link_radius" value="0.05"/>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="${link_radius}" length="${link_length}"/>
      </geometry>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <link name="link_1">
    <visual>
      <geometry>
        <cylinder radius="${link_radius}" length="${link_length}"/>
      </geometry>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
  </link>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit effort="10.0" velocity="1.0" lower="-1.57" upper="1.57"/>
  </joint>

</robot>
```

---

### 3. Lanciare RViz per visualizzarlo

Crea il file `launch/view_robot.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_path = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share/my_robot_description/urdf/robot.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_path).read()
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
```

---

### 4. Avviare il robot in RViz

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_description view_robot.launch.py
```

---

##  Comandi utili

- **Convertire XACRO in URDF per debug**:
  ```bash
  ros2 run xacro xacro urdf/robot.urdf.xacro > robot.urdf
  ```

- **Verificare URDF**:
  ```bash
  check_urdf robot.urdf
  ```

- **Visualizzare manualmente in RViz**:
  ```bash
  ros2 run robot_state_publisher robot_state_publisher robot.urdf
  ros2 run rviz2 rviz2
  ```

---

##  Conclusione

- **URDF** descrive la struttura statica del robot.
- **XACRO** rende facile e modulare scrivere URDF.
- Questo approccio è alla base di simulazioni, RViz, MoveIt e Gazebo.

---
