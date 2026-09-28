# geoalg

Álgebra geométrica en Rust — geometría, robótica, cálculo y compatibilidad con el mundo real.

## Qué hace

- **Geometría:** puntos, rectas, planos, esferas, distancias, incidencias.
- **Movimiento:** motores (rotación + traslación), rotors, translators, interpolación suave.
- **Robótica:** modelo de robot, cinemática directa (FK), trayectorias, detección de choques.
- **Cálculo:** derivadas numéricas, velocidades de pose, jacobian, integración ODE.
- **Compatibilidad:** matrices 4x4, cuaterniones, integración con glam.

## Qué NO hace

- No es un motor de renderizado.
- No tiene detección de colisiones compleja.
- Solo soporta PGA 3D por ahora.

## Instalación

Agrega geoalg en tu Cargo.toml:

    [dependencies]
    geoalg = { git = "https://github.com/071476/geoalg.git" }
    geoalg-robotics = { git = "https://github.com/071476/geoalg.git" }
    geoalg-calculus = { git = "https://github.com/071476/geoalg.git" }
    geoalg-compat = { git = "https://github.com/071476/geoalg.git" }

## Licencias

Proyecto con doble licencia: MIT y Apache-2.0.
