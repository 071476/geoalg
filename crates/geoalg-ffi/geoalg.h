/* geoalg-ffi: puente C para Unity, Unreal y otros motores. */

#ifndef GEOALG_FFI_H
#define GEOALG_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Motor (pose rígida) como 8 floats: escalar + 6 bivectores + pseudoscalar.
 */
typedef struct GeoMotor {
  double data[8];
} GeoMotor;

/**
 * Posición 3D compatible con C.
 */
typedef struct GeoVec3 {
  double x;
  double y;
  double z;
} GeoVec3;

/**
 * Matriz 4x4 (row-major) compatible con C.
 */
typedef struct GeoMat4 {
  double m[16];
} GeoMat4;

/**
 * Cuaternión (w, x, y, z) compatible con C.
 */
typedef struct GeoQuat {
  double w;
  double x;
  double y;
  double z;
} GeoQuat;

/**
 * Motor identidad (sin movimiento).
 */
struct GeoMotor geoalg_motor_identity(void);

/**
 * Motor de traslación.
 */
struct GeoMotor geoalg_motor_from_translation(double dx, double dy, double dz);

/**
 * Motor de rotación alrededor de un eje que pasa por el origen.
 */
struct GeoMotor geoalg_motor_from_rotation(double ax, double ay, double az, double angle);

/**
 * Composición: `out = a * b` (aplicar `a` primero, luego `b`).
 */
struct GeoMotor geoalg_motor_compose(const struct GeoMotor *a, const struct GeoMotor *b);

/**
 * Aplicar motor a un punto → devuelve el punto transformado.
 */
struct GeoVec3 geoalg_motor_apply_point(const struct GeoMotor *m, double x, double y, double z);

/**
 * Motor → Matriz 4x4 (row-major) para OpenGL/Unity/Unreal.
 */
struct GeoMat4 geoalg_motor_to_mat4(const struct GeoMotor *m);

/**
 * Matriz 4x4 → Motor.
 */
struct GeoMotor geoalg_mat4_to_motor(const struct GeoMat4 *m);

/**
 * Motor → posición + cuaternión (formato ROS/Unity/gltf).
 */
struct GeoVec3 geoalg_motor_to_pose(const struct GeoMotor *m);

/**
 * Motor → cuaternión.
 */
struct GeoQuat geoalg_motor_to_quat(const struct GeoMotor *m);

/**
 * Posición + cuaternión → Motor.
 */
struct GeoMotor geoalg_pose_to_motor(double px,
                                     double py,
                                     double pz,
                                     double qw,
                                     double qx,
                                     double qy,
                                     double qz);

#endif  /* GEOALG_FFI_H */
