# UAV Workspace — Guia rapida

## Que es este proyecto

Simulador de quadrotor basado en MuJoCo 3.4.0 + ROS2 Humble para investigacion de control UAV.
Solo 4 paquetes: `drone_teleop`, `acp_mujoco_simulator`, `MujocoRosUtils`, `quadrotor_msgs`.

## Compilar

```bash
cd /home/bryansgue/uav_ws
source /opt/ros/humble/setup.bash
export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws
colcon build --symlink-install --cmake-args -DMUJOCO_ROOT_DIR=/home/bryansgue/uav_ws/mujoco-3.4.0
source install/setup.bash
```

## Ejecutar el simulador

En cada terminal nueva:
```bash
cd /home/bryansgue/uav_ws
source install/setup.bash
export COLCON_UAV_WS_DIR=/home/bryansgue/uav_ws
```

### Lanzar MuJoCo (Terminal 1)

```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=<ESCENA>
```

Escenas disponibles:

| Escena | Descripcion |
|--------|-------------|
| `payload` (default) | Drone + payload con cable 0.8m + paredes. Modelo ideal (sin dinamica de motores) |
| `nopayload` | Drone libre, sin carga. Modelo ideal |
| `motors` | Drone + modelo de motores realista (gemelo digital) + paredes. CON dinamica de motores |
| `motors_nowall` | Igual que `motors` pero sin paredes |
| `gates` | Drone + modelo de motores + 7 race gates + paredes. Solo camara `cinematic` (chase) |

Camaras disponibles en escena (cycle con `[` / `]` en viewer):
- `camera_pov`: vista fija (escenas distintas a `gates`)
- `cinematic`: trackcom (sigue posicion drone, no rota con el). Unica en `gates`

Diferencia clave:
- **Sin motores** (`payload`, `nopayload`): aplica fuerzas/torques ideales directamente al cuerpo
- **Con motores** (`motors`, `motors_nowall`): simula dinamica real (retardo, saturacion, F=kf*omega^2)

Argumentos opcionales: `quad_name`, `init_x`, `init_y`, `init_z`, `init_yaw`, `gates_collide` (solo escena `gates`, default `false`)

Ejemplo:
```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors init_x:=1.0 init_z:=0.5
```

Alternativa con cierre limpio:
```bash
ros2 run drone_teleop mujoco_launch.sh scene:=motors
```

### Perturbaciones (viento + turbulencia + ráfagas)

Nodo `wind_publisher` publica una fuerza externa continua a `/<quad_name>/external_force` combinando proceso de Ornstein-Uhlenbeck (viento medio), filtro tipo Dryden (turbulencia) y ráfagas discretas (1-cos). El plugin `MujocoRosUtils::ExternalForce` lo aplica en modo `persistent=true`.

Activar desde el launch:
```bash
# Perturbación suave (default):
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors_nowall wind:=true

# Personalizada:
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors_nowall \
     wind:=true turbulence:=moderate mean_wind_speed:=2.0 wind_seed:=42
```

Argumentos del launch: `wind` (bool), `turbulence` (`light|moderate|severe`), `mean_wind_speed` (m/s), `wind_seed` (int).

Nodo por separado (más parámetros):
```bash
ros2 run drone_teleop wind_publisher --ros-args \
  -p turbulence_level:=moderate -p mean_wind_speed:=2.0 \
  -p drag_coef:=0.08 -p gust_amp_max:=2.5 -p force_limit:=4.0
```

Intensidades sugeridas (dron 1.08 kg, hover ~10.6 N):

| Nivel | `mean_wind_speed` | `drag_coef` | `gust_amp_max` | `force_limit` | % hover |
|---|---:|---:|---:|---:|---:|
| Suave | 1.5 | 0.04 | 1.5 | 3.0 | 19% |
| Medio | 2.0 | 0.08 | 2.5 | 4.0 | 33% |
| Fuerte | 3.0 | 0.12 | 4.0 | 6.0 | 52% |

Fuerza manual puntual:
```bash
ros2 topic pub --once /quadrotor/external_force mujoco_ros_utils/msg/ExternalForce \
  "{duration: {sec: 0, nanosec: 0}, pos: {x: 0, y: 0, z: 0}, force: {x: 3.0, y: 0.0, z: 0.0}}"
# publicar force=0 para limpiar
```

### Lanzar control interactivo (Terminal 2)

```bash
ros2 run drone_teleop teleop
```

Comandos: `takeoff [H]`, `land`, `goto X Y Z [YAW]`, `hover`, `stop`, `thrust T`, `cmd T WX WY WZ`, `state`

## Estructura clave

- `src/drone_teleop/` — Paquete de control Python (teleop, launch files)
- `src/acp_mujoco_simulator/model/` — Modelos XML/xacro del drone y escenas
- `src/MujocoRosUtils/plugin/` — Plugins C++ (AcroMode, OdometryPublisher, ImuPublisher, CollisionPublisher)
- `src/quadrotor_msgs/` — Definicion del mensaje TRPYCommand
- `mujoco-3.4.0/` — Binario MuJoCo

## Topics ROS2

| Topic | Tipo | Direccion | Hz |
|-------|------|-----------|---:|
| `/quadrotor/odom` | nav_msgs/Odometry | MuJoCo -> tu codigo | 240 |
| `/quadrotor/imu` | sensor_msgs/Imu | MuJoCo -> tu codigo | 200 |
| `/quadrotor/trpy_cmd` | quadrotor_msgs/TRPYCommand | tu codigo -> MuJoCo | tu decides |
| `/quadrotor/collision` | std_msgs/Bool | MuJoCo -> tu codigo | 100 (solo modo motors) |
| `/quadrotor/external_force` | mujoco_ros_utils/ExternalForce | wind_publisher -> MuJoCo | 100 (si `wind:=true`) |

## Parametros fisicos

- Masa total: 1.08 kg
- Hover thrust: ~10.6 N
- Thrust max: 82 N
- Timestep: 0.003 s (RK4)
- Rate controller gains: Kom = diag(20, 35, 45)
