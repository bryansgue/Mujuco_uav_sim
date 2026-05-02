"""
Launch SOLO el simulador MuJoCo — sin NMPC, sin trackers, sin mav_manager.
Solo el drone en el mundo, listo para recibir comandos en /quadrotor/trpy_cmd.

Escenas disponibles (argumento 'scene'):
  payload    — drone + esfera azul colgando con cable de 0.8 m  (default)
  nopayload  — drone libre, sin carga ni cable
  motors     — drone + payload CON modelo de motores realista (gemelo digital)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

# Mapa de nombre de escena → archivo xacro
SCENES = {
    "payload":        "scene_acro.xml.xacro",
    "nopayload":      "scene_acro_nopayload.xml.xacro",
    "motors":         "scene_acro_motors.xml.xacro",
    "motors_nowall":  "scene_acro_motors_nowall.xml.xacro",
    "gates":          "scene_acro_motors_gates.xml.xacro",
}


def mujoco_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("acp_mujoco_simulator")
    colcon_ws_dir = os.environ.get("COLCON_UAV_WS_DIR", "")
    if not colcon_ws_dir:
        raise RuntimeError("COLCON_UAV_WS_DIR no esta seteado!")

    mujoco_bin = os.path.join(colcon_ws_dir, "mujoco-3.4.0", "bin", "simulate")
    if not os.path.isfile(mujoco_bin):
        raise RuntimeError(f"MuJoCo no encontrado en: {mujoco_bin}")

    quad_name = LaunchConfiguration("quad_name").perform(context)
    init_x    = LaunchConfiguration("init_x").perform(context)
    init_y    = LaunchConfiguration("init_y").perform(context)
    init_z    = LaunchConfiguration("init_z").perform(context)
    init_yaw  = LaunchConfiguration("init_yaw").perform(context)
    scene     = LaunchConfiguration("scene").perform(context)
    wind            = LaunchConfiguration("wind").perform(context).lower() in ("true", "1", "yes")
    turbulence      = LaunchConfiguration("turbulence").perform(context)
    mean_wind_speed = LaunchConfiguration("mean_wind_speed").perform(context)
    wind_seed       = LaunchConfiguration("wind_seed").perform(context)
    gates_collide   = LaunchConfiguration("gates_collide").perform(context).lower() in ("true", "1", "yes", "on")

    scene_file = SCENES.get(scene)
    if scene_file is None:
        raise RuntimeError(
            f"Escena '{scene}' no reconocida. Opciones válidas: {list(SCENES.keys())}"
        )

    xacro_file = os.path.join(pkg_share, "model", scene_file)
    doc = xacro.process_file(xacro_file, mappings={
        "quad_name":      quad_name,
        "init_x":         init_x,
        "init_y":         init_y,
        "init_z":         init_z,
        "init_yaw":       init_yaw,
        "gates_collide":  "1" if gates_collide else "0",
    })

    temp_xml = "/tmp/drone_teleop_scene.xml"
    with open(temp_xml, "w") as f:
        f.write(doc.toxml())

    mujoco_process = ExecuteProcess(
        cmd=[mujoco_bin, temp_xml],
        cwd=os.path.dirname(mujoco_bin),
        output="screen",
        sigterm_timeout="3",
        sigkill_timeout="2",
    )

    # Shut down the entire launch when MuJoCo window is closed
    shutdown_on_mujoco_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=mujoco_process,
            on_exit=[Shutdown(reason="MuJoCo cerrado")],
        )
    )

    actions = [mujoco_process, shutdown_on_mujoco_exit]

    if wind:
        wind_node = Node(
            package="drone_teleop",
            executable="wind_publisher",
            name="wind_publisher",
            output="screen",
            parameters=[{
                "quad_name": quad_name,
                "turbulence_level": turbulence,
                "mean_wind_speed": float(mean_wind_speed),
                "seed": int(wind_seed),
            }],
        )
        actions.append(wind_node)

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("quad_name",  default_value="quadrotor"),
        DeclareLaunchArgument("init_x",     default_value="0.0"),
        DeclareLaunchArgument("init_y",     default_value="0.0"),
        DeclareLaunchArgument("init_z",     default_value="0.02"),
        DeclareLaunchArgument("init_yaw",   default_value="0.0"),
        DeclareLaunchArgument("scene",      default_value="payload",
                              description="Escena a cargar: 'payload' o 'nopayload'"),
        DeclareLaunchArgument("wind",            default_value="false",
                              description="Si 'true', arranca wind_publisher (perturbaciones continuas)"),
        DeclareLaunchArgument("turbulence",      default_value="moderate",
                              description="Nivel de turbulencia: light | moderate | severe"),
        DeclareLaunchArgument("mean_wind_speed", default_value="1.5",
                              description="Velocidad media del viento [m/s]"),
        DeclareLaunchArgument("wind_seed",       default_value="-1",
                              description="Semilla RNG (-1 = aleatoria)"),
        DeclareLaunchArgument("gates_collide",   default_value="false",
                              description="Activar colision en los gates (solo escena 'gates'). true|false"),
        OpaqueFunction(function=mujoco_setup),
    ])
