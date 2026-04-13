# 🚁 UAV Workspace — Simulador MuJoCo para Investigación

## Tabla de Contenidos
1. [Descripción General](#descripción-general)
2. [Arquitectura del Sistema](#arquitectura-del-sistema)
3. [Paquetes y sus Relaciones](#paquetes-y-sus-relaciones)
4. [Variables de Control](#variables-de-control)
5. [Topics ROS2](#topics-ros2)
6. [Instalación y Setup](#instalación-y-setup)
7. [Cómo Correr](#cómo-correr)
8. [Cómo Funciona por Dentro](#cómo-funciona-por-dentro)
9. [Guía para Crear tu Propio Controlador](#guía-para-crear-tu-propio-controlador)

---

## Descripción General

Este workspace contiene un simulador de quadrotor **aislado y mínimo** basado en:
- **MuJoCo 3.4.0**: Motor de física 3D (contacto, gravedad, rigid body dynamics)
- **ROS2 Humble**: Comunicación entre simulador y tu código via topics
- **drone_teleop**: Nodo de control interactivo con PD simple (ejemplo de cómo controlar)

```
┌─────────────────────────────────────────────────────────────┐
│                     ESTE WORKSPACE                          │
│                                                             │
│   Solo 4 paquetes + binario MuJoCo                         │
│   SIN: NMPC, acados, trackers, mav_manager                 │
│   TODO el control lo haces TÚ en Python                    │
└─────────────────────────────────────────────────────────────┘
```

---

## Arquitectura del Sistema

```
┌──────────────────────────────────────────────────────────────────┐
│                                                                  │
│   TU CÓDIGO (Python)              SIMULADOR (C++ / MuJoCo)      │
│                                                                  │
│  ┌──────────────┐     Topics     ┌─────────────────────────┐    │
│  │              │    ROS2        │                         │    │
│  │  teleop.py   │◄──────────────│  /quadrotor/odom        │    │
│  │  (o tu       │  nav_msgs/    │  (OdometryPublisher)    │    │
│  │  controlador)│  Odometry     │                         │    │
│  │              │◄──────────────│  /quadrotor/imu         │    │
│  │              │  sensor_msgs/ │  (ImuPublisher)         │    │
│  │              │  Imu          │                         │    │
│  │              │───────────────►  /quadrotor/trpy_cmd    │    │
│  │              │  TRPYCommand  │  (AcroMode plugin)      │    │
│  └──────────────┘               │                         │    │
│                                  │  MuJoCo Engine          │    │
│                                  │  ├── Física (F=ma)      │    │
│                                  │  ├── Contacto           │    │
│                                  │  ├── Gravedad           │    │
│                                  │  └── Integrador RK4     │    │
│                                  └─────────────────────────┘    │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## Paquetes y sus Relaciones

### Estructura del workspace
```
uav_ws/
├── mujoco-3.4.0/              ← Binario MuJoCo (NO es paquete ROS2)
│   └── bin/simulate           ← El ejecutable del simulador
│
└── src/
    ├── drone_teleop/          ← TU paquete de control (Python)
    ├── acp_mujoco_simulator/  ← Modelos XML del quadrotor
    ├── MujocoRosUtils/        ← Plugins C++ que conectan MuJoCo ↔ ROS2
    └── quadrotor_msgs/        ← Definición del mensaje TRPYCommand
```

### Relación entre paquetes

```
drone_teleop (Python, TU código)
│
├── DEPENDE DE → quadrotor_msgs
│                  Necesita: TRPYCommand.msg para enviar comandos
│
├── DEPENDE DE → nav_msgs (paquete del sistema ROS2)
│                  Necesita: Odometry.msg para leer posición
│
├── DEPENDE DE → sensor_msgs (paquete del sistema ROS2)
│                  Necesita: Imu.msg para leer aceleración/giroscopio
│
└── SU LAUNCH USA → acp_mujoco_simulator
                      Necesita: los archivos .xml.xacro del modelo del drone


acp_mujoco_simulator (modelos XML)
│
├── model/scene_acro.xml.xacro          ← Escena completa (suelo, luces, drone)
│   └── INCLUYE → quadrotor_acro_macro.xml.xacro
│                   ├── Geometría del drone (cuerpo, brazos, hélices)
│                   ├── Actuadores (thrust_fz, moment_tx, moment_ty, moment_tz)
│                   └── REFERENCIA plugins de → MujocoRosUtils
│                         ├── AcroMode         (control)
│                         ├── OdometryPublisher (sensado)
│                         └── ImuPublisher      (sensado)
│
└── NO tiene código, solo archivos XML descriptivos


MujocoRosUtils (plugins C++, la "magia")
│
├── plugin/AcroMode.cpp           ← Recibe TRPYCommand, aplica fuerzas al drone
├── plugin/OdometryPublisher.cpp  ← Lee estado de MuJoCo, publica /odom
├── plugin/ImuPublisher.cpp       ← Lee sensores de MuJoCo, publica /imu
│
├── DEPENDE DE → quadrotor_msgs   (para TRPYCommand)
├── DEPENDE DE → MuJoCo 3.4.0    (librería C, via MUJOCO_ROOT_DIR)
└── SE COMPILA COMO → librería .so que MuJoCo carga dinámicamente


quadrotor_msgs (mensajes ROS2)
│
├── msg_ros2/control_cmds/TRPYCommand.msg   ← El mensaje de control
│     Campos:
│       float64       thrust                ← Fuerza eje Z body [N]
│       Vector3       angular_velocity      ← Vel angular deseada [rad/s]
│       float64       roll, pitch, yaw      ← (no usados en AcroMode)
│       float64[4]    pred_input            ← (no usados aquí)
│       Quaternion    quaternion            ← (no usado aquí)
│
└── NO depende de nada más (paquete base)
```

### Orden de compilación (colcon lo maneja solo)
```
1. quadrotor_msgs          ← primero (no depende de nada)
2. MujocoRosUtils          ← segundo (depende de quadrotor_msgs + MuJoCo)
3. acp_mujoco_simulator    ← tercero (depende de MujocoRosUtils para los plugins)
4. drone_teleop            ← último (depende de quadrotor_msgs)
```

---

## Variables de Control

### ¿Qué envías? → `TRPYCommand` en `/quadrotor/trpy_cmd`

| Campo | Tipo | Unidad | Rango | Descripción |
|-------|------|--------|-------|-------------|
| `thrust` | float64 | Newton | 0 — 82 | Fuerza en el eje Z del body frame |
| `angular_velocity.x` | float64 | rad/s | -10 — 10 | Roll rate deseado |
| `angular_velocity.y` | float64 | rad/s | -10 — 10 | Pitch rate deseado |
| `angular_velocity.z` | float64 | rad/s | -10 — 10 | Yaw rate deseado |

### Valores importantes
```
Hover thrust = masa × gravedad = 1.08 × 9.81 ≈ 10.60 N
Thrust máximo = 82 N (≈ 7.7:1 thrust-to-weight ratio)
```

### ¿Qué pasa internamente con tu comando?

```
TÚ ENVÍAS:                    PLUGIN AcroMode:                  MUJOCO:
thrust=10.6 N ──────────────► ctrl[fz] = 10.6 N ──────────────► Fuerza en eje Z body
ωx_des=0.1 rad/s            ┐
ωy_des=0.0 rad/s            ├► Controlador de rate:            ┐
ωz_des=0.0 rad/s            ┘  τ = ω×(J·ω) - J·Kom·(ω-ωd)   ├► Torques en body frame
                                                                ┘
Donde:
  J   = diag(0.00345, 0.00180, 0.00180)  [kg·m²] ← Inercia del drone
  Kom = diag(20, 35, 45)                          ← Ganancias del rate controller
  ω   = velocidad angular ACTUAL del body
  ωd  = velocidad angular DESEADA (lo que tú envías)
```

### ¿Qué lees? → Sensores

**`/quadrotor/odom`** (nav_msgs/Odometry, 240 Hz):
```
pose.pose.position     → x, y, z         [m]     posición en world frame
pose.pose.orientation  → w, x, y, z      [-]     quaternion
twist.twist.linear     → vx, vy, vz      [m/s]   velocidad lineal
twist.twist.angular    → ωx, ωy, ωz      [rad/s] velocidad angular
```

**`/quadrotor/imu`** (sensor_msgs/Imu, 200 Hz):
```
linear_acceleration    → ax, ay, az      [m/s²]  aceleración (incluye gravedad)
angular_velocity       → ωx, ωy, ωz      [rad/s] giroscopio
orientation            → w, x, y, z      [-]     quaternion
```

---

## Topics ROS2

| Topic | Tipo | Dirección | Hz | Publicado por |
|-------|------|-----------|---:|--------------|
| `/quadrotor/odom` | nav_msgs/Odometry | MuJoCo → TU código | 240 | OdometryPublisher |
| `/quadrotor/imu` | sensor_msgs/Imu | MuJoCo → TU código | 200 | ImuPublisher |
| `/quadrotor/trpy_cmd` | quadrotor_msgs/TRPYCommand | TU código → MuJoCo | tú decides | AcroMode |

---

## Instalación y Setup

### Pre-requisitos del sistema
```bash
# ROS2 Humble (ya instalado)
# Dependencias de compilación:
sudo apt install libglfw3-dev libeigen3-dev ros-humble-tf2-ros
```

### Setup del workspace
```bash
cd /home/bryansgue/uav_ws

# Variable de entorno OBLIGATORIA (agregar al ~/.bashrc):
export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws

# Compilar:
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DMUJOCO_ROOT_DIR=/home/bryansgue/uav_ws/mujoco-3.4.0

# Sourcear:
source install/setup.bash
```

### Agregar al ~/.bashrc (una sola vez)
```bash
echo 'export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws' >> ~/.bashrc
```

---

## Cómo Correr

### Requisitos previos (en cada terminal nueva)
```bash
cd /home/bryansgue/uav_ws
source install/setup.bash
# Asegurar que la variable de entorno esté seteada:
export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws
```

> Si ya agregaste `COLCON_UAV_WS_DIR` a tu `~/.bashrc`, no necesitas exportarla manualmente.

### Terminal 1: Simulador MuJoCo

El launch file acepta el argumento `scene:=` para elegir la escena:

```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=<ESCENA>
```

#### Escenas disponibles

| Escena | Comando | Descripcion |
|--------|---------|-------------|
| `payload` **(default)** | `ros2 launch drone_teleop mujoco_only.launch.py` | Drone + payload (esfera azul) con cable de 0.8m, paredes |
| `nopayload` | `ros2 launch drone_teleop mujoco_only.launch.py scene:=nopayload` | Drone libre sin carga ni cable, paredes |
| `motors` | `ros2 launch drone_teleop mujoco_only.launch.py scene:=motors` | Drone con modelo de motores realista (gemelo digital), paredes |
| `motors_nowall` | `ros2 launch drone_teleop mujoco_only.launch.py scene:=motors_nowall` | Drone con modelo de motores realista, sin paredes |

#### Diferencia entre modos SIN y CON modelo de motores

**Sin modelo de motores** (`payload`, `nopayload`):
- Usa `quadrotor_acro_macro.xml.xacro`
- Los actuadores aplican **directamente** fuerza y torques al cuerpo: `[Fz, Tx, Ty, Tz]`
- Modelo "ideal" — sin dinamica de motor

**Con modelo de motores** (`motors`, `motors_nowall`):
- Usa `quadrotor_acro_macro_motors.xml.xacro`
- Tiene helices visuales que giran (4 propellers con joints hinge)
- El plugin `AcroMode` tiene `motor_model="true"`: pasa el wrench deseado por un modelo de motor de primer orden antes de escribir en los actuadores
- Parametros del modelo de motor:
  - `kf = 1.91e-6` — coef. thrust (`F = kf * omega^2`)
  - `km = 2.6e-8` — coef. torque (`tau = km * omega^2`)
  - `motor_tau = 0.02 s` — constante de tiempo del motor
  - `motor_omega_max = 2500 rad/s` — velocidad angular max
  - `arm_length = 0.1 m` — distancia centro-motor
  - `drag_coeff = 0.1` — drag aerodinamico
- Incluye `CollisionPublisher` (publica en `/quadrotor/collision`)

#### Argumentos opcionales del launch

| Argumento | Default | Descripcion |
|-----------|---------|-------------|
| `scene` | `payload` | Escena a cargar (ver tabla arriba) |
| `quad_name` | `quadrotor` | Nombre/prefijo del drone |
| `init_x` | `0.0` | Posicion inicial X [m] |
| `init_y` | `0.0` | Posicion inicial Y [m] |
| `init_z` | `0.02` | Posicion inicial Z [m] |
| `init_yaw` | `0.0` | Yaw inicial [rad] |

Ejemplo con argumentos:
```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors init_x:=1.0 init_z:=0.5
```

#### Alternativa: wrapper script (cierre limpio con Ctrl+C)
```bash
ros2 run drone_teleop mujoco_launch.sh scene:=motors
```

Esto abre la ventana grafica de MuJoCo con el drone en el suelo.

### Terminal 2: Control interactivo
```bash
cd /home/bryansgue/uav_ws
source install/setup.bash
ros2 run drone_teleop teleop
```

### Comandos del CLI:
```
takeoff [H]        → Despegar a H metros (default 1.0)
land               → Aterrizar
goto X Y Z [YAW°]  → Ir a posición
hover              → Mantener posición actual
stop               → EMERGENCIA: apagar motores
thrust T           → Enviar thrust directo en Newtons
cmd T WX WY WZ     → Enviar comando completo directo
state              → Ver posición/velocidad actual
```

### Alternativa: sin CLI, directo desde terminal ROS2
```bash
# Enviar un solo comando de hover:
ros2 topic pub --once /quadrotor/trpy_cmd quadrotor_msgs/msg/TRPYCommand \
  "{thrust: 10.6, angular_velocity: {x: 0.0, y: 0.0, z: 0.0}}"

# Ver odometría:
ros2 topic echo /quadrotor/odom --field pose.pose.position

# Alternativa: correr directamente con python3:
source install/setup.bash
python3 src/drone_teleop/drone_teleop/teleop.py
```

---

## Cómo Funciona por Dentro

### Cadena completa cuando haces `takeoff`:

```
1. teleop.py: takeoff(1.0)
   └─ mode = 'hover', target = [0, 0, 1.0]

2. control_loop() a 100Hz:
   └─ error = target - pos = [0, 0, 1.0] - [0, 0, 0.02]
   └─ PD: accel_deseada = Kp*error - Kd*vel + g
   └─ thrust = masa * accel_deseada.dot(z_body)
   └─ roll_des, pitch_des = f(accel_deseada, yaw)
   └─ wx, wy, wz = Kp_att * (angulo_des - angulo_actual)
   └─ PUBLICA: TRPYCommand(thrust=12.5, wx=0.01, wy=-0.02, wz=0.0)

3. Plugin AcroMode (dentro de MuJoCo, C++):
   └─ Recibe TRPYCommand via subscriber ROS2
   └─ ctrl[fz] = 12.5  (thrust directo)
   └─ Lee ω_actual del body en MuJoCo
   └─ e_ω = ω_actual - [0.01, -0.02, 0.0]
   └─ τ = ω×(J·ω) - J·Kom·e_ω  (rate controller)
   └─ ctrl[tx] = τx, ctrl[ty] = τy, ctrl[tz] = τz

4. MuJoCo physics step (dt=0.003s, integrador RK4):
   └─ F = R * [0, 0, thrust] + m*g  (gravedad)
   └─ τ_total = torques de actuadores
   └─ acc = F/m,  α = J⁻¹·(τ - ω×J·ω)
   └─ Integra: pos += vel*dt, vel += acc*dt, etc.

5. Plugin OdometryPublisher (cada 1/240 s):
   └─ Lee pos, quat, vel de MuJoCo
   └─ Publica en /quadrotor/odom
   └─ teleop.py lo recibe en odom_cb()

→ LOOP: vuelve al paso 2
```

### Parámetros del modelo físico

```
QUADROTOR:
  Masa total:           1.08 kg
  Cuerpo central:       0.85 kg  (box 7×7×3 cm)
  Brazos (x4):          0.035 kg cada uno
  Motores (x4):         0.015 kg cada uno
  Inercia (body):       Ixx=0.00345, Iyy=0.00180, Izz=0.00180 [kg·m²]

ACTUADORES (definidos en quadrotor_acro_macro.xml.xacro):
  body_thrust:    motor eje Z,     rango [0, 82] N
  x_moment:       torque eje X,    rango [-0.3, 0.3] N·m
  y_moment:       torque eje Y,    rango [-0.3, 0.3] N·m
  z_moment:       torque eje Z,    rango [-0.3, 0.3] N·m

RATE CONTROLLER (dentro de AcroMode):
  Kom = diag(20, 35, 45)       ← ganancias proporcionales de rate
  J   = diag(0.00345, 0.00180, 0.00180)  ← inercia usada internamente

SIMULACIÓN:
  Timestep:             0.003 s (333 Hz)
  Integrador:           RK4
  Gravedad:             [0, 0, -9.81] m/s²
```

---

## Guía para Crear tu Propio Controlador

### Opción 1: Modificar teleop.py
Edita `src/drone_teleop/drone_teleop/teleop.py` directamente. 
Como usamos `--symlink-install`, los cambios aplican sin recompilar.

### Opción 2: Crear un script Python nuevo
```python
#!/usr/bin/env python3
"""mi_controlador.py — Ejemplo mínimo"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TRPYCommand

class MiControlador(Node):
    def __init__(self):
        super().__init__('mi_controlador')
        self.pub = self.create_publisher(TRPYCommand, '/quadrotor/trpy_cmd', 10)
        self.sub = self.create_subscription(Odometry, '/quadrotor/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.01, self.control)  # 100 Hz
        self.z = 0.0

    def odom_cb(self, msg):
        self.z = msg.pose.pose.position.z

    def control(self):
        msg = TRPYCommand()
        # Controlador P simple: thrust = hover + Kp*(z_deseado - z_actual)
        msg.thrust = 1.08 * 9.81 + 8.0 * (1.0 - self.z)  # hover a 1m
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(MiControlador())

if __name__ == '__main__':
    main()
```

Correr:
```bash
source /home/bryansgue/uav_ws/install/setup.bash
python3 mi_controlador.py
```

### Opción 3: Crear un paquete ROS2 nuevo
```bash
cd /home/bryansgue/uav_ws/src
ros2 pkg create --build-type ament_python mi_control --dependencies rclpy nav_msgs sensor_msgs quadrotor_msgs
# Editar el archivo en mi_control/mi_control/nodo.py
cd /home/bryansgue/uav_ws
colcon build --symlink-install --packages-select mi_control
source install/setup.bash
ros2 run mi_control nodo
```

---

## Archivos Clave (para referencia rápida)

| Archivo | Qué hace |
|---------|----------|
| `drone_teleop/teleop.py` | Tu nodo de control con PD + CLI interactivo |
| `drone_teleop/launch/mujoco_only.launch.py` | Lanza SOLO MuJoCo (sin controladores) |
| `acp_mujoco_simulator/model/scene_acro.xml.xacro` | Escena: suelo + drone + objetos |
| `acp_mujoco_simulator/model/quadrotor_acro_macro.xml.xacro` | Modelo del drone: geometría + actuadores + plugins |
| `MujocoRosUtils/plugin/AcroMode.cpp` | Plugin que recibe TRPYCommand y aplica fuerzas |
| `MujocoRosUtils/plugin/OdometryPublisher.cpp` | Plugin que publica odometría |
| `MujocoRosUtils/plugin/ImuPublisher.cpp` | Plugin que publica IMU |
| `quadrotor_msgs/msg_ros2/control_cmds/TRPYCommand.msg` | Definición del mensaje de control |

---

## Troubleshooting

| Problema | Solución |
|----------|----------|
| `COLCON_UAV_WS_DIR not set` | `export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws` |
| `MuJoCo simulate not found` | Verificar que `mujoco-3.4.0/bin/simulate` existe |
| `No se recibió /quadrotor/odom` | El simulador no está corriendo. Lanza Terminal 1 primero |
| `quadrotor_msgs not found` | `colcon build` y luego `source install/setup.bash` |
| `MUJOCO_ROOT_DIR not set` | Compilar con: `colcon build --cmake-args -DMUJOCO_ROOT_DIR=.../mujoco-3.4.0` |
# Mujuco_uav_sim
