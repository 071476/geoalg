/* geoalg-ffi: puente C para Unity, Unreal y otros motores.
 *
 * Convenciones:
 * - geoalg usa la regla de la MANO DERECHA (como ROS, three.js y glTF).
 *   Unity y Unreal usan la MANO IZQUIERDA: hay que convertir los ejes.
 * - GeoMat4 va por FILAS (row-major), traslacion en m[3], m[7], m[11].
 *   Para OpenGL (por columnas) hay que transponerla.
 * - Los cuaterniones van en orden (w, x, y, z).
 * - GeoMotor es OPACO: usa las funciones para crearlo y leerlo.
 * - Un puntero NULL se trata como el motor identidad.
 */

#ifndef GEOALG_FFI_H
#define GEOALG_FFI_H

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

/* Codigos de resultado de las funciones de robot. */
#define GEOALG_OK 0
#define GEOALG_ERR_NULL -1
#define GEOALG_ERR_ARGUMENT -2
#define GEOALG_ERR_INTERNAL -99

/* Robot opaco: solo se usa a traves de un puntero. */
typedef struct GeoRobot GeoRobot;

/* Posicion 3D. */
typedef struct GeoVec3 {
  double x;
  double y;
  double z;
} GeoVec3;

/* Cuaternion (w, x, y, z). */
typedef struct GeoQuat {
  double w;
  double x;
  double y;
  double z;
} GeoQuat;

/* Posicion + cuaternion juntos (formato ROS / glTF). */
typedef struct GeoPose {
  double x;
  double y;
  double z;
  double qw;
  double qx;
  double qy;
  double qz;
} GeoPose;

/* Matriz 4x4 por filas, traslacion en m[3], m[7], m[11]. */
typedef struct GeoMat4 {
  double m[16];
} GeoMat4;

/* Motor (giro + traslacion). OPACO. */
typedef struct GeoMotor {
  double data[8];
} GeoMotor;

/* Motor identidad (sin movimiento). */
struct GeoMotor geoalg_motor_identity(void);

/* Motor de traslacion. */
struct GeoMotor geoalg_motor_from_translation(double dx, double dy, double dz);

/* Giro de `angle` radianes alrededor del eje (ax, ay, az) que pasa por el origen.
 * Si el eje es (0, 0, 0), devuelve la identidad. */
struct GeoMotor geoalg_motor_from_rotation(double ax, double ay, double az, double angle);

/* Composicion: aplica `a` primero y luego `b`. */
struct GeoMotor geoalg_motor_compose(const struct GeoMotor *a, const struct GeoMotor *b);

/* Aplica el motor a un punto. */
struct GeoVec3 geoalg_motor_apply_point(const struct GeoMotor *m, double x, double y, double z);

/* Motor a matriz 4x4 por filas. */
struct GeoMat4 geoalg_motor_to_mat4(const struct GeoMotor *m);

/* Matriz 4x4 por filas a motor. */
struct GeoMotor geoalg_mat4_to_motor(const struct GeoMat4 *m);

/* Solo la posicion del motor. */
struct GeoVec3 geoalg_motor_to_position(const struct GeoMotor *m);

/* Solo el giro, como cuaternion (w, x, y, z). */
struct GeoQuat geoalg_motor_to_quat(const struct GeoMotor *m);

/* Posicion + cuaternion juntos. */
struct GeoPose geoalg_motor_to_pose(const struct GeoMotor *m);

/* Posicion + cuaternion a motor. */
struct GeoMotor geoalg_pose_to_motor(double px,
                                     double py,
                                     double pz,
                                     double qw,
                                     double qx,
                                     double qy,
                                     double qz);

/* ===================== Robots ===================== */

/* Crea un robot vacio. Liberalo con geoalg_robot_free. */
GeoRobot *geoalg_robot_new(void);

/* Crea el brazo demo de 3 articulaciones (Z hacia arriba, tramos de 1). */
GeoRobot *geoalg_robot_demo_arm(void);

/* Libera un robot. Pasar NULL no hace nada. */
void geoalg_robot_free(GeoRobot *robot);

/* Numero de articulaciones, o GEOALG_ERR_NULL. */
int32_t geoalg_robot_joint_count(const GeoRobot *robot);

/* Agrega una articulacion que GIRA sobre el eje (dx, dy, dz).
 * (ox, oy, oz) es el tramo hasta la siguiente articulacion.
 * min y max en radianes. */
int32_t geoalg_robot_add_revolute(GeoRobot *robot,
                                  double dx, double dy, double dz,
                                  double ox, double oy, double oz,
                                  double min, double max);

/* Agrega una articulacion que DESLIZA sobre el eje (dx, dy, dz). */
int32_t geoalg_robot_add_prismatic(GeoRobot *robot,
                                   double dx, double dy, double dz,
                                   double ox, double oy, double oz,
                                   double min, double max);

/* Cinematica directa. q: n valores. out_points: (n + 1) puntos x, y, z
 * seguidos (base, articulaciones, punta). out_len >= (n + 1) * 3. */
int32_t geoalg_robot_fk(const GeoRobot *robot,
                        const double *q,
                        size_t n,
                        double *out_points,
                        size_t out_len);

/* Cinematica inversa de posicion: llevar la punta a (tx, ty, tz).
 * Devuelve 1 si llego, 0 si no llego (out_q trae la mejor postura),
 * o un codigo de error negativo. out_error puede ser NULL. */
int32_t geoalg_robot_ik(const GeoRobot *robot,
                        double tx, double ty, double tz,
                        const double *q0,
                        size_t n,
                        uint32_t max_iter,
                        double tol,
                        double *out_q,
                        double *out_error);

#endif  /* GEOALG_FFI_H */
