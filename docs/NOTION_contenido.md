# Contenido base para el Notion (pegar y completar)

> Este texto es el punto de partida: **revise cada decisión y reescríbala con sus palabras**; el reporte exige justificar con criterios de ingeniería y referencias, y todos los integrantes deben poder defenderlo en el pitch.

## 1. Enlace al repositorio
`[COMPLETAR: URL del repo de GitHub]`

## 2. Modelo del robot
- Estructura: `base_link` (punto medio del eje) → 2 ruedas continuas (`joint_rueda_izq/der`, eje y) + caster fijo + `lidar_link` + `camara_link` → `camara_optica_link`.
- Parámetros (xacro): `r = 0.05 m`, `b = 0.30 m`, chasis 0.34 × 0.20 × 0.08 m (2.0 kg), ruedas 0.15 kg, caster r = 0.025 m.
- Inercias: caja `m(a²+b²)/12`; cilindro (eje y) `Iyy = mr²/2`, `Ixx = Izz = m(3r²+h²)/12`; esfera `2mr²/5`.
- Decisiones: ancho total 0.34 m para pasar por el estrechamiento de 0.8 m; caster delantero sin fricción; CG por delante del eje para no volcar hacia atrás; colisiones con geometrías primitivas.
- Tabla completa de parámetros y justificaciones: ver README, sección “Modelo del robot”.

## 3. Cinemática de las llantas
Diagrama w → u → q̇:

    w = [ωD, ωI]ᵀ  --A-->  u = [v, ω]ᵀ  --J(θ)-->  q̇ = [ẋ, ẏ, θ̇]ᵀ

    A = [[r/2, r/2], [r/b, -r/b]]        J(θ) = [[cosθ, 0], [sinθ, 0], [0, 1]]

**Derivación (completar con el dibujo):** la velocidad del centro del eje es el promedio de las velocidades de contacto de las ruedas,
`v = (vD + vI)/2 = r(ωD + ωI)/2`; la velocidad angular es la diferencia dividida por la separación, `ω = (vD − vI)/b = r(ωD − ωI)/b`.
Sin deslizamiento lateral (restricción no holónoma): `−ẋ sinθ + ẏ cosθ = 0`, que se cumple con `J(θ)`.
Referencia: Siegwart et al., cap. 3.

Implementación: nodo `cinematica_diferencial` (lee `/joint_states`, integra por punto medio, publica `/odom_cinematica` y `/error_odometria`).

## 4. Comparación con el /odom de DiffDrive (resultados)
`[COMPLETAR tras correr en su máquina]` Tabla: distancia recorrida, error de posición medio/máx/final (mm y % de la distancia), error de θ (°).
Datos: lanzar con `csv:=/tmp/odometrias.csv` y graficar `e_pos(t)` y `e_theta(t)`.
Análisis: ver README, sección “Origen esperado de las diferencias”.

## 5. LiDAR y cámara
Ver tabla del README (valores, justificación, frames `lidar_link` y `camara_optica_link`). Captura de RViz2 con `/scan` y la imagen: `[COMPLETAR]`.

## 6. Pista
Imagen `src/robot_diferencial/docs/pista_preview.png`; proceso: binarizar imagen de referencia → esqueletizar → suavizar → escalar 0.08 m/px → muros por desplazamiento de ±w/2 de la línea central.
Ancho 1.20 m, estrechamiento 0.80 m, 7 obstáculos, lazo de 78 m, spawn en Start/Finish.

## 7. Integración y teleoperación
Un solo launch; teleop en otra terminal. Video/captura del recorrido completo: `[COMPLETAR]`.

## 8. Referencias
Ver README, sección “Referencias”.

---

# Archivo de entrega (Moodle)

Nombre: `GXX_PI_AVYY_NNAA_NNAA.txt` (XX = número del ET, YY = número de la entrega, NNAA = iniciales de nombre y apellidos de cada autor separadas por guiones bajos).
Contenido (texto plano):

    Notion: <enlace al Notion>
    Repositorio: <enlace al repositorio de GitHub>
