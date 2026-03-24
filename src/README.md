# UAV Workspace — Guía de Inicio Rápido

Workspace ROS 2 (Humble) para simulación de quadrotor con **MuJoCo 3.4.0**, control en modo Acro y transporte de carga mediante cable.

---

## 📦 Paquetes del workspace

| Paquete | Descripción |
|---|---|
| `MujocoRosUtils` | Plugins C++ de MuJoCo → ROS 2 (odometría, IMU, AcroMode, etc.) |
| `acp_mujoco_simulator` | Modelos XACRO del drone, escenas y mundo |
| `drone_teleop` | Nodo Python de teleoperación + launch files |
| `quadrotor_msgs` | Mensajes y servicios ROS 2 para el quadrotor |

---

## 🔧 Requisitos previos

- ROS 2 Humble
- MuJoCo 3.4.0 descomprimido en `~/uav_ws/mujoco-3.4.0`
- Variable de entorno `COLCON_UAV_WS_DIR` apuntando al workspace

Agrega esto a tu `~/.bashrc`:

```bash
export COLCON_UAV_WS_DIR=/home/$USER/uav_ws
source /opt/ros/humble/setup.bash
source $COLCON_UAV_WS_DIR/install/setup.bash
```

---

## 🏗️ Compilar

```bash
cd ~/uav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DMUJOCO_ROOT_DIR=$COLCON_UAV_WS_DIR/mujoco-3.4.0
```

> ⚠️ El mensaje `CMake Warning: Invalid value used with --config` es **inofensivo** — no afecta el build.

Para recompilar solo un paquete específico:

```bash
colcon build --symlink-install --packages-select mujoco_ros_utils \
  --cmake-args -DMUJOCO_ROOT_DIR=$COLCON_UAV_WS_DIR/mujoco-3.4.0
```

---

## 🚀 Lanzar el simulador

### Terminal 1 — MuJoCo + ROS 2

```bash
source ~/uav_ws/install/setup.bash
ros2 launch drone_teleop mujoco_only.launch.py
```

#### Escenas disponibles (argumento `scene`)

| `scene:=` | Descripción |
|---|---|
| `payload` | Drone + payload, control directo F/τ **(default)** |
| `nopayload` | Drone libre, control directo F/τ, sin carga |
| `motors` | Drone sin payload **CON modelo de motores realista** (gemelo digital) |

```bash
# Con payload (default, actuadores directos)
ros2 launch drone_teleop mujoco_only.launch.py scene:=payload

# Sin payload
ros2 launch drone_teleop mujoco_only.launch.py scene:=nopayload

# Gemelo digital con motores (sin payload)
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors
```

Todos los parámetros opcionales:

```bash
ros2 launch drone_teleop mujoco_only.launch.py \
  scene:=nopayload \
  quad_name:=quadrotor \
  init_x:=0.0 \
  init_y:=0.0 \
  init_z:=0.02 \
  init_yaw:=0.0
```

### Terminal 2 — Teleoperación interactiva

```bash
source ~/uav_ws/install/setup.bash
ros2 run drone_teleop teleop
```

---

## 🎮 Comandos de teleoperación

Una vez dentro del CLI interactivo:

| Comando | Descripción |
|---|---|
| `takeoff [H]` | Despegar a `H` metros (default `1.0 m`) |
| `land` | Aterrizar suavemente |
| `goto X Y Z [YAW°]` | Ir a posición en metros (yaw en grados) |
| `hover` | Mantener posición actual |
| `stop` | ⚠️ EMERGENCIA: apagar motores |
| `thrust T` | Enviar solo thrust en Newtons (sin PD) |
| `cmd T WX WY WZ` | Thrust + velocidades angulares directas |
| `state` | Ver posición, velocidad y modo actual |
| `help` | Mostrar menú de ayuda |
| `quit` | Salir del CLI |

---

## 📡 Tópicos ROS 2 principales

| Tópico | Tipo | Descripción |
|---|---|---|
| `/quadrotor/trpy_cmd` | `quadrotor_msgs/TRPYCommand` | **Entrada de control** — thrust + ω |
| `/quadrotor/odom` | `nav_msgs/Odometry` | Posición y velocidad del drone |
| `/quadrotor/imu` | `sensor_msgs/Imu` | Acelerómetro + giróscopo |
| `/quadrotor/payload/odom` | `nav_msgs/Odometry` | Odometría de la carga (payload) |

### Ver tópicos en tiempo real

```bash
# Odometría del drone
ros2 topic echo /quadrotor/odom

# Estado de la carga
ros2 topic echo /quadrotor/payload/odom

# Frecuencia de publicación
ros2 topic hz /quadrotor/odom
```

---

## 🧠 Arquitectura de control

```
Tu controlador
     │
     │  TRPYCommand
     │  .thrust          → Fuerza en Z-body [N]   (rango: 0–82 N, hover ≈ 10.6 N)
     │  .angular_velocity → Velocidad angular deseada [rad/s]
     │                       .x = roll rate
     │                       .y = pitch rate
     │                       .z = yaw rate
     ▼
┌────────────────────────────────────┐
│   Plugin AcroMode (C++ / MuJoCo)  │
│                                    │
│  Controlador de rates interno:     │
│    τ = ω×(J·ω) - J·Kom·(ω - ωd)  │
│                                    │
│  J   = diag(0.00345, 0.00180, 0.00180) kg·m²  │
│  Kom = diag(20, 35, 45)            │
│  Hz  = 500                         │
└────────────────────────────────────┘
     │
     ▼  Publica
  /quadrotor/odom   (240 Hz)
  /quadrotor/imu    (240 Hz)
```

### Parámetros físicos del drone

| Parámetro | Valor |
|---|---|
| Masa total | `1.08 kg` |
| Thrust hover | `≈ 10.6 N` (masa × 9.81) |
| Thrust máximo | `82.0 N` |
| Torques máximos | `±0.3 N·m` (roll, pitch, yaw) |
| Inercia Jxx | `0.00345 kg·m²` |
| Inercia Jyy | `0.00180 kg·m²` |
| Inercia Jzz | `0.00180 kg·m²` |
| Timestep MuJoCo | `0.003 s` (333 Hz) |
| Integrador | `RK4` |

---

## 🏗️ Escena del simulador (`scene_acro.xml.xacro`)

El mundo incluye:

- **Floor** — plano infinito con textura
- **Drone** — quadrotor con modelo rígido, 4 brazos y 4 thrusters
- **Payload** — esfera azul (`masa = 0.1128 kg`) conectada al drone por un cable (`longitud = 0.8 m`)
- **3 cajas de colores** — obstáculos de referencia (rojo, verde, azul)
- **4 paredes semitransparentes** — delimitan el área de vuelo
- **Ejes de referencia** — X (rojo), Y (verde), Z (azul) en el origen

### Modificar posición inicial del drone

```bash
ros2 launch drone_teleop mujoco_only.launch.py init_x:=1.0 init_y:=0.5 init_z:=0.02
```

---

## ✏️ Primeras ediciones — dónde tocar cada cosa

### 1. Cambiar parámetros físicos del drone

**Archivo:** `acp_mujoco_simulator/model/quadrotor_acro_macro.xml.xacro`

```xml
<!-- Masa del cuerpo central -->
<geom name="${prefix}_core" ... mass=".85"/>

<!-- Rango de thrust del motor (0 a 82 N) -->
<motor ... ctrlrange="0 82.0" .../>

<!-- Torque máximo (±0.3 N·m) -->
<motor ... ctrlrange="-0.3 0.3" .../>
```

### 2. Cambiar las ganancias del controlador AcroMode

**Archivo:** `MujocoRosUtils/plugin/AcroMode.h`

```cpp
double kw_x_{20.0};   // Ganancia roll rate
double kw_y_{35.0};   // Ganancia pitch rate
double kw_z_{45.0};   // Ganancia yaw rate
Eigen::Matrix3d J_ = Eigen::Vector3d(0.00345398, 0.00179687, 0.00179676).asDiagonal();
```

> Después de editar este archivo debes **recompilar** `mujoco_ros_utils`.

### 3. Cambiar las ganancias del controlador PD de teleop

**Archivo:** `drone_teleop/drone_teleop/teleop.py` — función `_control_loop()`

```python
Kp_xy, Kd_xy = 4.0, 2.5   # Posición XY
Kp_z,  Kd_z  = 8.0, 4.0   # Altitud
Kp_att        = 6.0         # Actitud (roll/pitch)
Kp_yaw        = 2.0         # Yaw
```

> Al usar `--symlink-install` en el build, **no necesitas recompilar** para cambios en Python.

### 4. Agregar objetos a la escena

**Archivo:** `acp_mujoco_simulator/model/scene_acro.xml.xacro`

```xml
<!-- Ejemplo: agregar una caja nueva -->
<geom name="mi_caja" type="box" pos="3 0 0.1" size="0.3 0.3 0.1" rgba="1 0 1 1"/>
```

### 5. Cambiar el cable (tendon) del payload

**Archivo:** `acp_mujoco_simulator/model/scene_acro.xml.xacro`

```xml
<tendon>
  <spatial name="rope0" limited="true" range="0 0.8" ...>
    <!-- range="0 LONGITUD_CABLE_EN_METROS" -->
  </spatial>
</tendon>
```

### 6. Enviar comandos desde tu propio nodo Python

```python
from quadrotor_msgs.msg import TRPYCommand

pub = node.create_publisher(TRPYCommand, '/quadrotor/trpy_cmd', 10)

msg = TRPYCommand()
msg.thrust = 10.6          # hover
msg.angular_velocity.x = 0.0
msg.angular_velocity.y = 0.0
msg.angular_velocity.z = 0.0
pub.publish(msg)
```

---

## 🗂️ Estructura de archivos relevantes

```
uav_ws/src/
├── README.md                          ← Este archivo
│
├── drone_teleop/
│   ├── drone_teleop/teleop.py         ← Nodo de teleop + controlador PD
│   └── launch/mujoco_only.launch.py   ← Launch del simulador
│
├── acp_mujoco_simulator/
│   └── model/
│       ├── scene_acro.xml.xacro                ← Escena con payload (directo)
│       ├── scene_acro_nopayload.xml.xacro      ← Escena sin payload (directo)
│       ├── scene_acro_motors.xml.xacro         ← Escena con motor model, sin payload (gemelo digital)
│       ├── quadrotor_acro_macro.xml.xacro      ← Drone: actuadores directos F/τ
│       ├── quadrotor_acro_macro_motors.xml.xacro ← Drone: con motor model
│       └── scene_lowlevel.xml.xacro            ← Escena alternativa (control lowlevel)
│
├── MujocoRosUtils/
│   └── plugin/
│       ├── AcroMode.cpp / .h          ← Plugin controlador de rates + motor model
│       ├── OdometryPublisher.cpp / .h ← Publica /odom
│       ├── ImuPublisher.cpp / .h      ← Publica /imu
│       └── WrenchToActuators.cpp / .h ← Plugin wrench → actuadores
│
└── quadrotor_msgs/
    └── msg_ros2/
        └── TRPYCommand.msg            ← Mensaje de control principal
```

---

## 🔩 Modelo de motores — Gemelo Digital

La escena `motors` agrega una capa realista entre el rate controller y MuJoCo:

```
Tu NMPC / teleop
     │  TRPYCommand { F, ωd }
     ▼
┌──────────────────────────────────────┐
│  Rate Controller (igual que antes)   │
│  τ = ω×(J·ω) - J·Kom·(ω - ωd)      │
│  → produce wrench deseado [F,τx,τy,τz] │
└──────────────┬───────────────────────┘
               │
     ┌─────────▼─────────┐
     │   CAPA NUEVA:     │
     │   Motor Model     │
     │                   │
     │ 1. Mixer (X-config, Betaflight):
     │    [F,τx,τy,τz] → [ω₁²,ω₂²,ω₃²,ω₄²]_des
     │                   │
     │ 2. Dinámica de motor (1er orden):
     │    ω̇ᵢ = (ωᵢ_des - ωᵢ) / τ_motor
     │                   │
     │ 3. Hélice:
     │    Fᵢ = kf · ωᵢ²
     │    τᵢ = km · ωᵢ²
     │                   │
     │ 4. Re-combinar → [F,τx,τy,τz]_actual
     │                   │
     │ 5. Drag aerodinámico (opcional)
     └─────────┬─────────┘
               │
               ▼
         MuJoCo (física)
```

### Layout de motores (Betaflight X-config, vista superior)

```
         Frente
   M4(CCW)    M1(CW)
    (+x,+y)    (+x,-y)
        \      /
         [BODY]
        /      \
   M3(CW)     M2(CCW)
    (-x,+y)    (-x,-y)
```

### Parámetros configurables

Editables en `quadrotor_acro_macro_motors.xml.xacro`:

| Parámetro | Default | Descripción | Cómo obtenerlo del drone real |
|---|---|---|---|
| `arm_length` | `0.1 m` | Distancia centro → motor | Medir con regla |
| `kf` | `1.91e-6` | Coef. thrust $F = k_f \omega^2$ [N/(rad/s)²] | Thrust stand / banco de pruebas |
| `km` | `2.6e-8` | Coef. torque $\tau = k_m \omega^2$ [N·m/(rad/s)²] | ~0.014 × kf (típico) |
| `motor_tau` | `0.02 s` | Constante de tiempo del motor | Step response en banco |
| `motor_omega_max` | `2500 rad/s` | Velocidad angular máxima (~23900 RPM) | Datasheet del motor |
| `drag_coeff` | `0.1` | Drag aerodinámico [N/(m/s)] | Estimación en vuelo |

### Valores de referencia (defaults)

Con los valores por defecto:
- **ω_hover** por motor ≈ 1178 rad/s (~11250 RPM)
- **ω_max** = 2500 rad/s (~23900 RPM)
- **Relación thrust/weight** ≈ 4.5:1
- **Respuesta del motor**: 98% en ~80 ms (4 × τ)

### Activar/desactivar

- `scene:=payload` o `scene:=nopayload` → motor_model **OFF** (comportamiento original)
- `scene:=motors` → motor_model **ON** (gemelo digital)

---

## 🐛 Problemas comunes

| Problema | Solución |
|---|---|
| `COLCON_UAV_WS_DIR no esta seteado` | Exportar la variable en `~/.bashrc` y hacer `source ~/.bashrc` |
| `MuJoCo no encontrado en: .../bin/simulate` | Verificar que `mujoco-3.4.0` está en `~/uav_ws/` |
| `No se recibió /quadrotor/odom` | Lanzar primero el simulador en Terminal 1 antes del teleop |
| `CMake Error: Invalid value used with --config` | Error inofensivo del cmake de MujocoRosUtils, ignorar |
| Drone inestable al despegar | Reducir `Kp_z` o aumentar `Kd_z` en `teleop.py` |
