# geoalg

Álgebra geométrica en Rust — geometría, robótica, cálculo y compatibilidad con el mundo real.

## Qué hace

- **Geometría:** puntos, rectas, planos, esferas, distancias, incidencias.
- **Movimiento:** motores, rotors, translators, interpolación suave de poses.
- **Robótica:** modelo de robot, cinemática directa (FK), trayectorias, detección de choques.
- **Cálculo:** derivadas numéricas, velocidades de pose, jacobian, integración ODE.
- **Compatibilidad:** matrices 4×4, cuaterniones, integración con `glam`.

## Qué NO hace

- No es un motor de renderizado completo.
- No tiene detección de colisiones compleja.
- Solo soporta PGA 3D por ahora.

## Instalación

Agrega `geoalg` en tu `Cargo.toml` desde GitHub o mediante rutas locales para desarrollo.
