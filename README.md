# UAV Workspace — Simulador MuJoCo + ROS2

Simulador de quadrotor (MuJoCo 3.4.0 + ROS2 Humble) para investigación de control UAV.

---

## Compilar

```bash
cd /home/bryansgue/uav_ws
source /opt/ros/humble/setup.bash
export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws
colcon build --symlink-install --cmake-args -DMUJOCO_ROOT_DIR=/home/bryansgue/uav_ws/mujoco-3.4.0
source install/setup.bash
```

Cada terminal nueva:
```bash
cd /home/bryansgue/uav_ws
source install/setup.bash
export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws
```

---

## Modos / Escenas

| Escena | Descripción |
|--------|-------------|
| `payload` (default) | Drone + payload con cable 0.8 m + paredes. Modelo ideal (sin dinámica de motores). |
| `nopayload` | Drone libre, sin carga. Modelo ideal. |
| `motors` | Drone + modelo de motores realista (gemelo digital) + paredes. Dinámica real (retardo, saturación, `F=kf*ω²`). |
| `motors_nowall` | Igual que `motors` sin paredes. |
| `gates` | Drone + motores + 7 race gates + paredes. Solo cámara `cinematic`. |

Diferencia clave:
- **Sin motores** (`payload`, `nopayload`): aplica fuerzas/torques ideales.
- **Con motores** (`motors`, `motors_nowall`, `gates`): dinámica real de motores.

---

## Lanzar simulador

### Con interfaz gráfica
```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=<ESCENA>
```

### Headless (sin GUI — entrenamiento / hardware prep / CI)
```bash
# Tuning de controlador (gates atravesables, tiempo real):
ros2 launch drone_teleop mujoco_headless.launch.py scene:=gates gates_collide:=off realtime:=true

# Test real (gates con colisión):
ros2 launch drone_teleop mujoco_headless.launch.py scene:=gates gates_collide:=on realtime:=true
```

### Argumentos opcionales

| Arg | Default | Descripción |
|-----|---------|-------------|
| `scene` | `payload` (`gates` en headless) | Ver tabla de escenas |
| `quad_name` | `quadrotor` | Prefijo de topics |
| `init_x` / `init_y` / `init_z` | `0` / `0` / `0.02` | Pose inicial |
| `init_yaw` | `0` | Yaw inicial (rad) |
| `gates_collide` | `false` | Solo escena `gates`. `true`=drone choca con aros |
| `realtime` | `false` (solo headless) | `true`=sincroniza wall-clock |
| `wind` | `false` | Activa perturbación |
| `turbulence` | `moderate` | `light` / `moderate` / `severe` |
| `mean_wind_speed` | `1.5` | m/s |
| `wind_seed` | `-1` | Semilla RNG (`-1`=aleatoria) |

Ejemplo:
```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors init_x:=1.0 init_z:=0.5
```

Wrapper con cierre limpio (Ctrl+C):
```bash
ros2 run drone_teleop mujoco_launch.sh scene:=motors
```

---

## Perturbaciones (viento + turbulencia + ráfagas)

Nodo `wind_publisher` combina Ornstein-Uhlenbeck (viento medio), filtro Dryden (turbulencia) y ráfagas (1-cos). Publica a `/<quad_name>/external_force`.

```bash
# Activar desde launch (default suave):
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors_nowall wind:=true

# Personalizado:
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors_nowall \
  wind:=true turbulence:=moderate mean_wind_speed:=2.0 wind_seed:=42

# Nodo aislado (más parámetros):
ros2 run drone_teleop wind_publisher --ros-args \
  -p turbulence_level:=moderate -p mean_wind_speed:=2.0 \
  -p drag_coef:=0.08 -p gust_amp_max:=2.5 -p force_limit:=4.0
```

### Intensidades sugeridas (dron 1.08 kg, hover ~10.6 N)

| Nivel | `mean_wind_speed` | `drag_coef` | `gust_amp_max` | `force_limit` | % hover |
|---|---:|---:|---:|---:|---:|
| Suave | 1.5 | 0.04 | 1.5 | 3.0 | 19% |
| Medio | 2.0 | 0.08 | 2.5 | 4.0 | 33% |
| Fuerte | 3.0 | 0.12 | 4.0 | 6.0 | 52% |

### Fuerza manual puntual
```bash
ros2 topic pub --once /quadrotor/external_force mujoco_ros_utils/msg/ExternalForce \
  "{duration: {sec: 0, nanosec: 0}, pos: {x: 0, y: 0, z: 0}, force: {x: 3.0, y: 0.0, z: 0.0}}"
```

---

## Control interactivo

```bash
ros2 run drone_teleop teleop
```

Comandos: `takeoff [H]`, `land`, `goto X Y Z [YAW]`, `hover`, `stop`, `thrust T`, `cmd T WX WY WZ`, `state`.

---

## Topics ROS2

| Topic | Tipo | Dirección | Hz |
|-------|------|-----------|---:|
| `/quadrotor/odom` | `nav_msgs/Odometry` | MuJoCo → tu código | 240 |
| `/quadrotor/imu` | `sensor_msgs/Imu` | MuJoCo → tu código | 200 |
| `/quadrotor/trpy_cmd` | `quadrotor_msgs/TRPYCommand` | tu código → MuJoCo | tu decides |
| `/quadrotor/collision` | `std_msgs/Bool` | MuJoCo → tu código | 100 (solo `motors*`) |
| `/quadrotor/external_force` | `mujoco_ros_utils/ExternalForce` | `wind_publisher` → MuJoCo | 100 (si `wind:=true`) |

---

## Parámetros físicos del modelo

```
QUADROTOR:
  Masa total:           1.08 kg
  Cuerpo central:       0.85 kg  (box 7×7×3 cm)
  Brazos (x4):          0.035 kg cada uno
  Motores (x4):         0.015 kg cada uno
  Inercia (body):       Ixx=0.00345, Iyy=0.00180, Izz=0.00180 [kg·m²]

ACTUADORES (quadrotor_acro_macro.xml.xacro):
  body_thrust:    motor eje Z,     rango [0, 82] N
  x_moment:       torque eje X,    rango [-0.3, 0.3] N·m
  y_moment:       torque eje Y,    rango [-0.3, 0.3] N·m
  z_moment:       torque eje Z,    rango [-0.3, 0.3] N·m

RATE CONTROLLER (AcroMode):
  Kom = diag(20, 35, 45)                 ← ganancias proporcionales de rate
  J   = diag(0.00345, 0.00180, 0.00180)  ← inercia usada internamente

SIMULACIÓN:
  Timestep:             0.003 s (333 Hz)
  Integrador:           RK4
  Gravedad:             [0, 0, -9.81] m/s²
  Hover thrust:         ~10.6 N
  Thrust max:           82 N
```

---

## Aliases recomendados (`~/.bashrc`)

Para replicar el setup en una PC nueva, añadir al `~/.bashrc`:

```bash
# --- UAV workspace ---
export COLCON_UAV_WS_DIR=~/uav_ws
source ~/uav_ws/install/setup.bash
export PATH="$PATH:$HOME/uav_ws/install/drone_teleop/bin"

# Reset del drone en MuJoCo
dronreset() { ros2 service call "/${1:-quadrotor}/sim/reset" std_srvs/srv/Trigger '{}' | tail -2; }

# Launches del simulador (arg posicional = quad_name, default: quadrotor)
sim_gate_collideron()  { ros2 launch drone_teleop mujoco_only.launch.py     scene:=gates gates_collide:=on  quad_name:="${1:-quadrotor}"; }
sim_gate_collideroff() { ros2 launch drone_teleop mujoco_headless.launch.py scene:=gates gates_collide:=off realtime:=true quad_name:="${1:-quadrotor}"; }

# Experimentos de control (requiere paquete `quadrotor_mpc` separado)
mpcc()    { ros2 launch quadrotor_mpc experiment.launch.py ctrl:=mpcc        manual_gains:=true quad_name:="${1:-quadrotor}"; }
dq_mpcc() { ros2 launch quadrotor_mpc experiment.launch.py ctrl:=dq_ablation manual_gains:=true quad_name:="${1:-quadrotor}"; }
```

Recargar: `source ~/.bashrc`.

Uso:

```bash
sim_gate_collideroff           # headless gates atravesables (tuning)
sim_gate_collideron quad2      # GUI gates con colisión, quad_name=quad2
dronreset                      # resetea /quadrotor/sim/reset
dronreset quad2                # resetea /quad2/sim/reset
```

> `mpcc` / `dq_mpcc` apuntan al paquete `quadrotor_mpc`, fuera de este repo. Solo útil si ese workspace está compilado.

---

## Estructura del workspace

- `src/drone_teleop/` — Control Python + launch files
- `src/acp_mujoco_simulator/model/` — Modelos XML/xacro (drone + escenas)
- `src/MujocoRosUtils/plugin/` — Plugins C++ (`AcroMode`, `OdometryPublisher`, `ImuPublisher`, `CollisionPublisher`, `ExternalForce`, `headless_runner`)
- `src/quadrotor_msgs/` — Mensaje `TRPYCommand`
- `mujoco-3.4.0/` — Binario MuJoCo
