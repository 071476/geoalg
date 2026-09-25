# Roadmap de geoalg

Camino ordenado: cada etapa es un "cliente exigente" que obliga a crecer a la biblioteca.

## Estado actual
- geoalg-core: algebra, multivector, types, ops (37 tests).
- geoalg: prelude y algebras configuradas (39 tests).
- examples: basic_motor, ray_plane_intersection, export_scene.
- Puente Rust-Python funcionando (tools/plot_scene.py, visor 3D estatico).

## Etapa 1 - Brazo robotico
- 1A Brazo de 3 articulaciones animado (visor con cuadros).
- 1B Interpolacion de poses: Motor::exp(), Motor::log(), Motor::between().
- Extras: cinematica directa, deteccion de choque con el suelo.

## Etapa 2 - Personaje animado
- Jerarquias de motores (esqueleto).
- Mezcla de movimientos (blending).
- Rendimiento (sparse, SIMD).

## Etapa 3 - Juego 3D (trazado de rayos)
- Intersecciones: rayo-esfera, rayo-triangulo, rayo-plano.
- Reflexiones y sombras.
- Distancias entre geometrias.

## Etapa 4 - Camara de cine
- Trayectorias suaves: splines sobre motores.
- Encuadre y profundidad de campo.