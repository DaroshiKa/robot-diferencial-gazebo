"""Un solo launch: Gazebo + pista + robot MicroROS + bridge + cinemática + AEB + mux + modo + vueltas + RViz.

    ros2 launch robot_diferencial simulacion.launch.py                         # teleop, pista propia
    ros2 launch robot_diferencial simulacion.launch.py modo:=auto              # seguimiento de pared
    ros2 launch robot_diferencial simulacion.launch.py modo:=auto pista:=competencia
    ros2 launch robot_diferencial simulacion.launch.py pista:=competencia mundo:=/ruta/al/world.sdf

Argumentos:
  modo:=teleop|auto        teleop abre teleop_twist_keyboard en una terminal aparte (xterm o
                           gnome-terminal) publicando en /cmd_vel_teleop; auto usa /cmd_vel_auto.
  pista:=propia|competencia
                           competencia busca el paquete pista_competencia del profesor
                           (worlds/*.sdf o *.world); si no está instalado usa la RÉPLICA para pruebas.
  mundo:=<ruta .sdf>       fuerza un world concreto (tiene prioridad sobre pista).
  gui:=true|false          interfaz de Gazebo (false = solo servidor).
  rviz:=true|false         camara:=true|false     n_vueltas:=3     salida:=<carpeta resultados>
  x:=-0.60 y:=0.0 yaw:=0.0 pose inicial (detrás de la línea de salida/meta en x = 0).
  b_odometria:=0.134       vía del nodo de cinemática (geométrica; probar la efectiva b_ef).
  csv:=<ruta>              CSV del nodo de cinemática (por defecto <salida>/cinematica.csv).
  Resultados por defecto en ~/resultados_robot_diferencial/<fecha>_<pista>_<modo>/ ;
  luego: ros2 run diferencial_control analizar_odometria <esa carpeta>

Cadena de comandos (el AEB siempre está y tiene la última palabra):
  /cmd_vel_teleop ─┐
                   ├─ mux_cmd ─ /cmd_vel_mux ─ aeb ─ /cmd_vel ─ DiffDrive (Gazebo)
  /cmd_vel_auto  ──┘
"""
import glob
import os
import shutil
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NOMBRE_ROBOT = 'robot_diferencial'


def _buscar_world_competencia():
    """World del paquete pista_competencia (entregado por el profesor) y rutas de recursos."""
    try:
        share = get_package_share_directory('pista_competencia')
    except PackageNotFoundError:
        return None, []
    candidatos = []
    for patron in ('worlds/*.sdf', 'worlds/*.world', '*.sdf', '*.world', '**/*.world', '**/*.sdf'):
        candidatos += sorted(glob.glob(os.path.join(share, patron), recursive=True))
    candidatos = [c for c in candidatos if not c.endswith('model.sdf')]
    # orden: models/ primero; la carpeta padre solo si el propio paquete es un modelo (model.config).
    # (Gazebo se detiene en la primera carpeta con el nombre del modelo, aunque no tenga model.config)
    rutas = [os.path.join(share, 'models'), share]
    if os.path.exists(os.path.join(share, 'model.config')):
        rutas.append(os.path.dirname(share))
    return (candidatos[0] if candidatos else None), [r for r in rutas if os.path.isdir(r)]


_PLUGINS = [
    ('gz-sim-physics-system', 'gz::sim::systems::Physics', ''),
    ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands', ''),
    ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster', ''),
    ('gz-sim-sensors-system', 'gz::sim::systems::Sensors', '<render_engine>ogre2</render_engine>'),
]


def _asegurar_plugins(world_file):
    """El LiDAR (gpu_lidar) necesita el sistema Sensors en el WORLD. Si el world (p. ej. el del
    profesor) no lo declara, se escribe una copia con los sistemas que falten, junto al original
    (para no romper rutas relativas) o en /tmp si no hay permiso de escritura."""
    import re
    try:
        with open(world_file) as f:
            texto = f.read()
    except OSError:
        return world_file, []
    faltan = [p for p in _PLUGINS if p[0] not in texto and p[1] not in texto]
    # si el world no declara NINGÚN plugin, Gazebo carga sus valores por defecto (sin Sensors)
    if not faltan:
        return world_file, []
    m = re.search(r'<world[^>]*>', texto)
    if m is None:
        return world_file, []
    bloque = ''.join(f'\n    <plugin filename="{a}" name="{b}">{c}</plugin>' for a, b, c in faltan)
    nuevo = texto[:m.end()] + bloque + texto[m.end():]
    base = os.path.splitext(os.path.basename(world_file))[0] + '_con_sensores.sdf'
    for carpeta in (os.path.dirname(os.path.abspath(world_file)), '/tmp'):
        try:
            ruta = os.path.join(carpeta, base)
            with open(ruta, 'w') as f:
                f.write(nuevo)
            return ruta, [p[1].split('::')[-1] for p in faltan]
        except OSError:
            continue
    return world_file, []


def _terminal():
    if shutil.which('xterm'):
        return 'xterm -fa Monospace -fs 11 -geometry 90x24 -T teleop -e'
    if shutil.which('gnome-terminal'):
        return 'gnome-terminal --wait --title=teleop --'
    if shutil.which('konsole'):
        return 'konsole -e'
    return None


def _configurar(context):
    pkg = get_package_share_directory('robot_diferencial')
    modo = LaunchConfiguration('modo').perform(context)
    pista = LaunchConfiguration('pista').perform(context)
    mundo = LaunchConfiguration('mundo').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    camara = LaunchConfiguration('camara').perform(context).lower() == 'true'
    x, y, yaw = (LaunchConfiguration(k).perform(context) for k in ('x', 'y', 'yaw'))
    if modo not in ('teleop', 'auto'):
        raise RuntimeError(f"modo debe ser 'teleop' o 'auto' (se recibió '{modo}')")
    if pista not in ('propia', 'competencia'):
        raise RuntimeError(f"pista debe ser 'propia' o 'competencia' (se recibió '{pista}')")

    acciones = [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))]
    if mundo:
        world_file = mundo
        acciones.append(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(os.path.abspath(mundo))))
    elif pista == 'propia':
        world_file = os.path.join(pkg, 'worlds', 'pista_propia.sdf')
    else:
        world_file, rutas = _buscar_world_competencia()
        for r in rutas:
            acciones.append(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', r))
        if world_file is None:
            world_file = os.path.join(pkg, 'worlds', 'pista_competencia_replica.sdf')
            acciones.append(LogInfo(msg='[AVISO] No se encontró el paquete pista_competencia: se usa la '
                                        'RÉPLICA de pruebas (worlds/pista_competencia_replica.sdf). Para la '
                                        'competencia, compile el paquete del profesor en el mismo workspace '
                                        'o use mundo:=<ruta al world>.'))
    salida = LaunchConfiguration('salida').perform(context)
    if not salida:
        salida = os.path.join(os.path.expanduser('~'), 'resultados_robot_diferencial',
                              time.strftime('%Y%m%d_%H%M%S_') + f'{pista}_{modo}')
    os.makedirs(salida, exist_ok=True)
    csv_cin = LaunchConfiguration('csv').perform(context) or os.path.join(salida, 'cinematica.csv')
    world_file, agregados = _asegurar_plugins(world_file)
    if agregados:
        acciones.append(LogInfo(msg=f'[AVISO] el world no declaraba {agregados}: se usa una copia con '
                                    f'esos sistemas ({world_file})'))
    acciones.append(LogInfo(msg=f'[robot_diferencial] modo={modo} pista={pista} world={world_file}\n'
                                f'[robot_diferencial] resultados -> {salida}'))

    xacro_file = os.path.join(pkg, 'urdf', 'diferencial.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, f' camara:={"true" if camara else "false"}']), value_type=str)

    gz_args = f'-r {"" if gui else "-s "}{world_file}'
    acciones.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()))

    sim = {'use_sim_time': True}
    acciones.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                         parameters=[{'robot_description': robot_description, **sim}], output='screen'))
    # el frame odom del DiffDrive nace en la pose inicial: world -> odom estático (para RViz)
    acciones.append(Node(package='tf2_ros', executable='static_transform_publisher',
                         arguments=['--x', x, '--y', y, '--yaw', yaw, '--frame-id', 'world',
                                    '--child-frame-id', 'odom'], parameters=[sim]))
    acciones.append(Node(package='ros_gz_sim', executable='create', output='screen',
                         arguments=['-name', NOMBRE_ROBOT, '-topic', 'robot_description',
                                    '-x', x, '-y', y, '-z', '0.03', '-Y', yaw]))

    # ]  ROS -> Gazebo     [  Gazebo -> ROS
    puentes = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
    ]
    remap = []
    if camara:
        puentes += ['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                    '/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
        remap = [('/camera/image_raw/camera_info', '/camera/camera_info')]
    acciones.append(Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=puentes,
                         remappings=remap, parameters=[sim], output='screen'))

    # ---- nodos propios ----
    acciones.append(Node(package='diferencial_control', executable='cinematica_diferencial',
                         parameters=[{**sim, 'csv': csv_cin,
                                      'separacion_ruedas': float(LaunchConfiguration('b_odometria').perform(context))}],
                         output='screen'))
    acciones.append(Node(package='diferencial_control', executable='mux_cmd',
                         parameters=[{**sim, 'modo': modo}], output='screen'))
    acciones.append(Node(package='diferencial_control', executable='aeb',
                         parameters=[{**sim, 'csv': os.path.join(salida, 'aeb.csv')}], output='screen'))
    acciones.append(Node(package='diferencial_control', executable='seguidor_pared',
                         parameters=[os.path.join(pkg, 'config', 'seguidor_pared.yaml'),
                                     {**sim, 'csv': os.path.join(salida, 'seguidor.csv')}],
                         output='screen'))
    acciones.append(Node(package='diferencial_control', executable='vueltas', output='screen',
                         parameters=[{**sim, 'n_vueltas': int(LaunchConfiguration('n_vueltas').perform(context)),
                                      'salida': salida,
                                      'etiqueta': f'{pista}_{modo}'}]))

    if modo == 'teleop':
        term = _terminal()
        if term is None:
            acciones.append(LogInfo(msg='[AVISO] No hay xterm/gnome-terminal: abra otra terminal y ejecute\n'
                                        '  ros2 run teleop_twist_keyboard teleop_twist_keyboard '
                                        '--ros-args -r cmd_vel:=/cmd_vel_teleop'))
        else:
            acciones.append(Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
                                 prefix=term, output='screen',
                                 parameters=[{'speed': 0.5, 'turn': 1.5}],
                                 remappings=[('cmd_vel', '/cmd_vel_teleop')]))
    return acciones


def generate_launch_description():
    pkg = get_package_share_directory('robot_diferencial')
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='teleop', description="'teleop' | 'auto'"),
        DeclareLaunchArgument('pista', default_value='propia', description="'propia' | 'competencia'"),
        DeclareLaunchArgument('mundo', default_value='', description='world .sdf (opcional, prioridad)'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('camara', default_value='true'),
        DeclareLaunchArgument('n_vueltas', default_value='3'),
        DeclareLaunchArgument('salida', default_value='', description='carpeta de resultados'),
        DeclareLaunchArgument('csv', default_value='', description='CSV del nodo de cinemática'),
        DeclareLaunchArgument('b_odometria', default_value='0.134',
                              description='vía usada por el nodo de cinemática (0.134 geométrica; '
                                          'la efectiva se estima con analizar_odometria)'),
        DeclareLaunchArgument('x', default_value='-0.60'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=_configurar),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'control.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
