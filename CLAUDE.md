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

Diferencia clave:
- **Sin motores** (`payload`, `nopayload`): aplica fuerzas/torques ideales directamente al cuerpo
- **Con motores** (`motors`, `motors_nowall`): simula dinamica real (retardo, saturacion, F=kf*omega^2)

Argumentos opcionales: `quad_name`, `init_x`, `init_y`, `init_z`, `init_yaw`

Ejemplo:
```bash
ros2 launch drone_teleop mujoco_only.launch.py scene:=motors init_x:=1.0 init_z:=0.5
```

Alternativa con cierre limpio:
```bash
ros2 run drone_teleop mujoco_launch.sh scene:=motors
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

## Parametros fisicos

- Masa total: 1.08 kg
- Hover thrust: ~10.6 N
- Thrust max: 82 N
- Timestep: 0.003 s (RK4)
- Rate controller gains: Kom = diag(20, 35, 45)
