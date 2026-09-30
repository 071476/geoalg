# geoalg

Algebra geometrica en Rust — geometria, robotica, calculo y compatibilidad con el mundo real.

## Que hace

- **Geometria:** puntos, rectas, planos, esferas, distancias, incidencias.
- **Movimiento:** motores (rotacion + traslacion), rotors, translators, interpolacion suave.
- **Robotica:** modelo de robot, cinematica directa (FK), trayectorias, deteccion de choques.
- **Calculo:** derivadas numericas, velocidades de pose, jacobian, integracion ODE.
- **Compatibilidad:** matrices 4x4, cuaterniones, integracion con glam, Unity, Unreal, WebAssembly.

## Que NO hace

- No es un motor de renderizado.
- No tiene deteccion de colisiones compleja.
- Solo soporta PGA 3D por ahora.

## Instalacion

Agrega geoalg en tu Cargo.toml:

    [dependencies]
    geoalg = { git = "https://github.com/071476/geoalg.git" }
    geoalg-robotics = { git = "https://github.com/071476/geoalg.git" }
    geoalg-calculus = { git = "https://github.com/071476/geoalg.git" }
    geoalg-compat = { git = "https://github.com/071476/geoalg.git" }

## Licenses — Dual Licensing

geoalg uses a dual license model to protect the author's work and enable open use:

| Who uses                         | License     | Cost          |
|----------------------------------|-------------|---------------|
| Open source projects             | AGPL-3.0    | Free          |
| Students & research              | AGPL-3.0    | Free          |
| Companies (closed-source)        | Commercial  | From $19/mo   |
| Companies that don't share code  | Commercial  | From $19/mo   |

- **AGPL-3.0:** free, but any modification must be shared under AGPL-3.0.
- **Commercial:** allows use in proprietary products without sharing code.

Commercial plans:

| Plan       | Price    | Developers | Products   | Support          |
|------------|----------|------------|------------|------------------|
| Starter    | $19/mo   | 1          | 1          | Email (48h)      |
| Team       | $49/mo   | Up to 3    | Unlimited  | Priority (24h)   |
| Business   | $149/mo  | Up to 10   | Unlimited  | Priority (24h)   |
| Enterprise | Contact  | Unlimited  | Unlimited  | SLA guaranteed   |

Annual billing: 2 months free.

See [LICENSE-AGPL](LICENSE-AGPL) and [LICENSE-COMMERCIAL](LICENSE-COMMERCIAL).

## Support the project

If geoalg is useful to you, consider supporting it:
- **GitHub Sponsors:** https://github.com/sponsors/071476
- **Commercial inquiries:** ali-gafer@hotmail.com
