# geoalg

Álgebra geométrica en Rust — geometría, robótica, cálculo y compatibilidad con el mundo real.

## Qué hace

- **Geometría:** puntos, rectas, planos, esferas, distancias, incidencias.
- **Movimiento:** motores (rotación + traslación en un solo objeto), rotors,
  translators, interpolación suave de poses (`between`, `slerp`, `exp`/`log`).
- **Robótica:** modelo de robot, cinemática directa (FK), herramientas (TCP),
  trayectorias, límites articulares, detección de choques.
- **Cálculo:** derivadas numéricas, velocidades de pose (twist), jacobian,
  integración de velocidad → pose, ODE (Euler, RK4).
- **Compatibilidad:** matrices 4×4, cuaterniones, ángulos de Euler, pose
  (ROS/glTF), integración con `glam` (Bevy, juegos Rust).

## Qué NO hace

- No es un motor de renderizado (solo tiene cámara y proyección).
- No tiene detección de colisiones compleja (solo segmento-segmento).
- No tiene planificación de trayectorias con obstáculos (solo interpolación).
- No soporta CGA ni Minkowski todavía (solo PGA 3D).

## Instalación

Agrega `geoalg` en tu `Cargo.toml`:

```toml
[dependencies]
geoalg = { git = "https://github.com/071476/geoalg.git" }
geoalg-robotics = { git = "https://github.com/071476/geoalg.git" }
geoalg-calculus = { git = "https://github.com/071476/geoalg.git" }
geoalg-compat = { git = "https://github.com/071476/geoalg.git" }


---

## Licencias

Proyecto con **doble licencia**, al estilo de Rust y Bevy:

- **MIT** — máxima libertad, incluye uso comercial.
- **Apache-2.0** — incluye protección de patentes explícita.

Puedes elegir cualquiera de las dos (o ambas). Ver [LICENSE-MIT](LICENSE-MIT) y [LICENSE-APACHE](LICENSE-APACHE).

Los contribuidores aceptan que sus contribuciones se publiquen bajo ambas licencias (estándar de la comunidad Rust).