#!/usr/bin/env bash
# Arma el paquete ROS 2 `pista_competencia` a partir de los dos archivos que entregó el profesor
# (el STL de la pista y el world .sdf), para probar en la pista REAL y no en la réplica.
#
# El world incluye la pista como <uri>model://pista_competencia</uri>, así que Gazebo necesita
# la carpeta models/pista_competencia/ con model.config, model.sdf y meshes/pista_competencia.stl.
#
# Uso (desde la raíz del workspace, la carpeta que tiene src/):
#   bash src/robot_diferencial/tools/crear_paquete_pista_competencia.sh <pista.stl> <pista.sdf>
#   colcon build --symlink-install && source install/setup.bash
#   ros2 launch robot_diferencial simulacion.launch.py modo:=auto pista:=competencia
#
# Si el profesor entrega el paquete completo, no hace falta: basta copiarlo en src/.
set -euo pipefail

if [ $# -ne 2 ]; then
  echo "uso: $0 <pista_competencia.stl> <pista_competencia.sdf>" >&2
  exit 1
fi
STL=$1
SDF=$2
[ -f "$STL" ] || { echo "no existe $STL" >&2; exit 1; }
[ -f "$SDF" ] || { echo "no existe $SDF" >&2; exit 1; }
[ -d src ] || { echo "ejecute desde la raíz del workspace (donde está src/)" >&2; exit 1; }

P=src/pista_competencia
M=$P/models/pista_competencia
mkdir -p "$P/worlds" "$M/meshes"
cp "$SDF" "$P/worlds/pista_competencia.sdf"
cp "$STL" "$M/meshes/pista_competencia.stl"

cat > "$M/model.config" <<'EOF'
<?xml version="1.0"?>
<model>
  <name>pista_competencia</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Pista de competencia (STL del profesor) con colisión</description>
</model>
EOF

cat > "$M/model.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="pista_competencia">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><mesh><uri>model://pista_competencia/meshes/pista_competencia.stl</uri></mesh></geometry>
      </collision>
      <visual name="visual">
        <geometry><mesh><uri>model://pista_competencia/meshes/pista_competencia.stl</uri></mesh></geometry>
        <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
EOF

cat > "$P/CMakeLists.txt" <<'EOF'
cmake_minimum_required(VERSION 3.10)
project(pista_competencia)
find_package(ament_cmake REQUIRED)
install(DIRECTORY worlds models DESTINATION share/${PROJECT_NAME})
ament_package()
EOF

cat > "$P/package.xml" <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>pista_competencia</name>
  <version>1.0.0</version>
  <description>Pista de competencia (archivos del profesor) empaquetada para Gazebo Harmonic</description>
  <maintainer email="jaime.landinez@eia.edu.co">Jaime Landinez</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

echo "listo: $P (compile con colcon build --symlink-install)"
