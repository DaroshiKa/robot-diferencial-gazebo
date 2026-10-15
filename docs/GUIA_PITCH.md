# Guía para el pitch (8 min + 8 min de preguntas)

Todos los integrantes deben responder sobre: **origen de los parámetros, función de cada bridge, relación entre frames y relación entre w y q̇.** Respuestas base (adáptelas):

## Origen de los parámetros
| Parámetro | Valor | De dónde sale |
|---|---|---|
| r, b | 0.05 m, 0.30 m | Decisión de diseño: ancho total 0.34 m < estrechamiento 0.8 m; v_max 1 m/s ⇒ 20 rad/s |
| `wheel_radius`, `wheel_separation` del DiffDrive | iguales a r, b | Se toman de las mismas propiedades xacro (una sola fuente de verdad) |
| LiDAR 800 muestras, 10 Hz, 0.15–12 m, 360° | clase RPLIDAR A1 | 8000 muestras/s ÷ 10 vueltas/s; hoja de datos SLAMTEC |
| Cámara 640×480, 30 FPS, 60° | VGA típico | suficiente para ver muros/obstáculos; costo de render bajo |
| Ancho de pista 1.2 m / estrechamiento 0.8 m | 3.5× / 2.3× el ancho del robot | margen para teleoperación con teclado |
| Inercias | fórmulas de sólidos uniformes | caja, cilindro, esfera (ver README) |

## Función de cada entrada del bridge (`ros_gz_bridge parameter_bridge`)
| Topic | Dirección | Para qué |
|---|---|---|
| `/clock` | Gz → ROS | tiempo de simulación (`use_sim_time`) |
| `/cmd_vel` | ROS → Gz | comando de velocidad (teleop) hacia DiffDrive |
| `/odom` | Gz → ROS | odometría del plugin DiffDrive |
| `/tf` | Gz → ROS | transformada `odom → base_link` del DiffDrive |
| `/world/pista/model/robot_diferencial/joint_state` → `/joint_states` | Gz → ROS | posiciones/velocidades de las ruedas (entrada de la cinemática y de robot_state_publisher) |
| `/scan` | Gz → ROS | LaserScan del LiDAR |
| `/camera/image_raw`, `/camera/camera_info` | Gz → ROS | imagen e intrínsecos de la cámara |

## Árbol de frames
`odom → base_link` (DiffDrive, vía bridge) → `rueda_izq`, `rueda_der` (robot_state_publisher con `/joint_states`), `caster_link`, `lidar_link`, `camara_link → camara_optica_link` (estáticos desde el URDF).
El LiDAR publica en `lidar_link`; imagen y `camera_info` en `camara_optica_link` (z adelante). RViz usa `odom` como fixed frame.

## Relación entre w y q̇
`u = A w` (A = [[r/2, r/2],[r/b, −r/b]]), `q̇ = J(θ) u`; ⇒ `q̇ = J(θ) A w`. Recta: ωD = ωI. Giro en sitio: ωD = −ωI. La dependencia de θ en J hace que el mismo w produzca ẋ, ẏ distintos según la orientación.

## Preguntas probables
- ¿Por qué el error entre odometrías no es cero? (integración, velocidades vs posiciones, tiempos; ver README)
- ¿Qué mide realmente ese error? (consistencia entre dos estimadores, no el error contra la pose real)
- ¿Por qué el caster está delante y sin fricción? ¿Qué pasa si se pone atrás o con fricción?
- ¿Por qué `camara_optica_link` y no `camara_link`? (convención óptica de ROS)
- ¿Cómo se diseñó la pista y cómo se garantizó que el robot cabe? (generador con verificaciones)
