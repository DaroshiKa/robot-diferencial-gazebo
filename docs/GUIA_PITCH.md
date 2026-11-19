# Guía para el pitch (8 min de presentación + 8 min de preguntas)

Todos deben poder responder sobre **el cálculo del TTC y su umbral, la sintonización del PID y el crecimiento
del error de odometría**. Detalle y fuentes en el README.

## Guion sugerido (8 min)

| Min | Qué mostrar | Material |
|---|---|---|
| 0:00–1:00 | Robot MicroROS: tabla de medidas, skid-steer con DiffDrive de 2 juntas por lado, MS200, límites | Gazebo + `ros2 topic hz /scan` (10 Hz), echo de un `/scan` (450 rangos) |
| 1:00–2:30 | AEB: fórmula, contorno, franja de barrido, umbral(v), retención | `docs/resultados/aeb_obstaculo_recta.png` y demo con `obstaculo.launch.py` |
| 2:30–4:00 | Seguimiento de pared: dos rayos, D_t+1, modelo ωn/ζ, D_des = min(D_nom, W/2), v según curvatura | `docs/resultados/propia_seguidor.png` |
| 4:00–5:00 | Pista propia: secciones, CAD (STEP abierto en Fusion/SolidWorks), STL con colisión | `docs/pista_propia.png`, `cad/pista_propia.step` |
| 5:00–6:30 | Tiempos y errores de cierre por vuelta; vía efectiva | tabla del nodo `vueltas`, `propia_odometria_*.png` |
| 6:30–8:00 | Integración: un launch, `modo:=`, `pista:=`, AEB en la cadena | demo en vivo |

## TTC y umbral

- **Fórmula:** para cada rayo, `ṙ_i = −v cos θ_i` (si el obstáculo está quieto, la distancia por ese rayo se acorta
  a la componente de la velocidad en esa dirección); `TTC_i = r_i / max(−ṙ_i, 0)`. Rayos que no se acortan
  (cos θ ≤ 0 avanzando) dan TTC = ∞. Se frena si el mínimo es menor que el umbral.
- **¿Qué r_i?** La distancia del **contorno** del robot al obstáculo por ese rayo (r_i del LiDAR menos la distancia del
  LiDAR al borde del rectángulo). Al frente son ~10 cm: sin esto el robot frenaría 10 cm tarde.
- **¿Por qué no frena en el estrechamiento?** Solo se evalúan los puntos dentro de la franja que el robot barrerá
  (ancho del robot + 1.5 cm, siguiendo el arco R = v/ω). En 0.40 m centrado quedan 12 cm a cada lado: los muros
  paralelos están fuera de la franja. En una curva, el muro exterior queda fuera del arco que el robot ya recorre.
  Ejemplo numérico sin franja: estrechamiento centrado (muros a 0.20 m del LiDAR), v = 1 m/s ⇒ el rayo a 45° mide
  0.283 m, menos 0.108 m de contorno = 0.175 m, y se acorta a v·cos 45° = 0.71 m/s ⇒ TTC ≈ 0.25 s < 0.53 s:
  frenaría siempre en el estrechamiento.
- **Umbral(v) = v/(2·a_f) + t_lat + t_m.** El primer término es la distancia de frenado v²/(2a_f) expresada como tiempo
  a velocidad v; a_f = 1.6 m/s² (80 % de los 2 m/s² del DiffDrive). t_lat = 0.12 s: el barrido puede tener hasta
  0.1 s de antigüedad (10 Hz) más el transporte. t_m = 0.10 s de margen. A 1 m/s: 0.31 + 0.12 + 0.10 = 0.53 s
  ⇒ dispara a ~0.53 m del contorno, frena en ~0.31 m (+ latencia) ⇒ queda a ~0.2–0.27 m (medido en Gazebo: 27 cm).
- **¿Por qué no un umbral fijo?** Uno fijo para 1 m/s es exagerado a 0.2 m/s (frena a 10 cm de cualquier cosa al girar)
  y uno fijo para baja velocidad no alcanza a frenar a 1 m/s.
- **Prioridad:** el AEB es el último nodo antes de `/cmd_vel`; teleop y seguidor pasan por el mux y luego por él.
  Sin `/scan` por 0.5 s ⇒ v = 0.
- **¿Por qué el robot se queda quieto y no rodea la caja?** Mientras el comando pida avanzar hacia el obstáculo, v = ω = 0.
  Se suelta solo si el camino está libre para ir a **v_max** con histéresis 1.3: si se soltara con la velocidad que pide
  el seguidor (que baja a 0.15 m/s frente a la caja), el robot se arrastraría hasta ella. Retroceder o girar en el sitio
  sí se permite (teleop).
- **Margen medido:** en 6 vueltas el mínimo TTC fue 0.74 s (umbral 0.53 s) y hubo 0 frenados falsos.

## Sintonización del PID

- **Geometría:** b a +90° (izquierda), a a +45°. α = arctan((a cos θ − b)/(a sin θ)) es el ángulo entre el rumbo y el
  muro (α > 0 = alejándose); D_t = b cos α; D_t+1 = D_t + l sin α (dónde estará el robot tras avanzar l).
  e = D_des − D_t+1 y ω = −PID(e) (muro a la izquierda: demasiado cerca ⇒ girar a la derecha).
- **Modelo:** ḋ = vα, α̇ = −ω, ω = −Kp(D_des − d − lα) ⇒ d̈ + Kp·l·ḋ + v·Kp·d = v·Kp·D_des ⇒
  ωn = √(v·Kp), ζ = (l/2)√(Kp/v). **l actúa como derivativo**. Kp = 10, l = 0.45 ⇒ ωn = 3.2 rad/s, ζ = 0.71 a 1 m/s.
- **Skid-steer:** el robot gira χ ≈ 1.2–1.6 veces menos de lo pedido ⇒ ganancia efectiva Kp/χ ⇒ ζ ≈ 0.56–0.65. Por eso
  no se bajó Kp.
- **Ki = 0.3** (sesgos lentos; anti-windup por integración condicional y límite), **Kd = 0.3** con filtro de 0.1 s
  (amortigua el retardo de 0.1 s del barrido sin amplificar el ruido de 1 cm). Mediana de 5 rayos por medición.
- **¿Por qué D_nom = 0.40 y D_des = min(D_nom, W/2)?** En 0.80 m eso es ir centrado (32 cm de holgura por lado) y en el
  estrechamiento de 0.40 m sigue centrado (12 cm). Con D_nom = 0.30 el robot tenía que desplazarse 10 cm al entrar al
  estrechamiento y se torcía: el AEB lo frenaba. W usa las distancias **proyectadas** (D_t+1) de ambos lados, así ve
  venir el estrechamiento.
- **Velocidad:** v = min(1, a_lat/|ω|, √(2a(d_frente − 0.30))): a_lat = 1.2 m/s² limita la aceleración lateral
  (v·ω = v²κ); la distancia de frente anticipa las curvas cerradas. En la horquilla baja a 0.3–0.6 m/s.
- **Cómo se buscó:** simulador 2D propio (`tools/sim2d.py`, mismo LiDAR, límites del DiffDrive, latencia y deslizamiento)
  barriendo Kp, l, D_nom, lado, χ ∈ [1, 1.7], latencias y semillas; luego validación en Gazebo. Error RMS en la pista
  propia: 6.3 cm.

## Crecimiento del error de odometría

- ωD, ωI = promedio de las dos ruedas de cada lado ⇒ u = A w ⇒ q̇ = J(θ)u, punto medio. q(0) = pose real.
- **Con b geométrica (0.134 m)** el error de rumbo crece (χ − 1)·360° ≈ 165° por vuelta: el skid-steer desliza
  lateralmente al girar, así que las ruedas "dicen" que giró ~1.48 veces más de lo real. El DiffDrive comete el mismo
  error (mismo modelo). Error de cierre: 853 cm/163°, 256 cm/−35°, 774 cm/127° (no es monótono porque el rumbo da
  vueltas: la posición se aleja y se acerca).
- **Con la vía efectiva b_ef = χ·b ≈ 0.197 m** (χ por mínimos cuadrados con la vuelta 1; 1.46 y 1.48 en las dos pistas):
  47 → 100 → 153 cm y −2 → −5 → −12°: crece ~linealmente (~1.4 % de la distancia). Lo que queda: χ no es constante
  (1.6 en sitio, 1.2 avanzando) y cualquier error de rumbo se convierte en error lateral proporcional a lo recorrido.
  La odometría sin una referencia externa (LiDAR/mapa, IMU) **no tiene cómo corregirse**: el error se acumula.
- Mejoras posibles (preguntas típicas): χ en función de la curvatura (modelo ICR de Mandow), fusionar un IMU para el rumbo
  (EKF con robot_localization), o corregir con el LiDAR (scan matching).

## Preguntas probables

- ¿Por qué 450 muestras y `max_angle = π − 2π/450`? 450 rayos **distintos** en 360° (0.8°); con ±π se repetiría el de 180°.
- ¿De dónde salen 29.9 rad/s² de aceleración angular? Misma aceleración tangencial (2 m/s²) en la banda de rodadura: 2a/b.
- ¿Por qué el chasis tiene 0.706 kg? Es la masa total (0.906) menos ruedas, LiDAR y cámara: la suma da exacto.
- ¿Cómo se mide el tiempo? Con la pose real y `/clock`: del primer cruce de x = 0 (hacia +x, |y| < 0.6) al cruce n.º 4.
- ¿Qué pasa si el world del profesor no tiene el sistema Sensors? El launch genera una copia con los sistemas que falten.
- ¿Usa mapa o waypoints? No: cada comando depende solo del barrido actual (ni siquiera de la odometría).
- ¿A qué frecuencia llegan las velocidades de las ruedas? `/joint_states` a 100 Hz (`<update_rate>` del plugin). Por
  defecto Gazebo lo publica en cada paso de física (1000 Hz) y eso, junto con `/clock`, saturaba los nodos: el AEB llegó
  a quedarse >1 s sin LiDAR. A 4 rad/s el rumbo cambia 0.04 rad entre muestras: 100 Hz sobra para integrar.
- ¿Probaron en la pista oficial? Sí, con el world y el STL entregados: 112.54 s en 3 vueltas, 0 frenados, holgura mín.
  8.3 cm; caja en la recta → se detiene a ~26 cm.
