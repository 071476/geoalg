// Geoalg.cs — binding C# para Unity.
// Copia este archivo a tu proyecto de Unity (Assets/Scripts/).
// Copia geoalg_ffi.dll a Assets/Plugins/.
// Cambia "geoalg_ffi" por el nombre de tu .dll si es diferente.

using System;
using System.Runtime.InteropServices;

public static class Geoalg
{
    const string DLL = "geoalg_ffi";

    // === Tipos ===

    [StructLayout(LayoutKind.Sequential)]
    public struct GeoVec3
    {
        public double x, y, z;
        public GeoVec3(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        public Vector3d ToVector3d() => new Vector3d(x, y, z);
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GeoQuat
    {
        public double w, x, y, z;
        public GeoQuat(double w, double x, double y, double z) { this.w = w; this.x = x; this.y = y; this.z = z; }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GeoMat4
    {
        public double m0, m1, m2, m3;
        public double m4, m5, m6, m7;
        public double m8, m9, m10, m11;
        public double m12, m13, m14, m15;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct GeoMotor
    {
        public double d0, d1, d2, d3, d4, d5, d6, d7;
    }

    // Para no depender de UnityEngine.Mathematics
    public struct Vector3d
    {
        public double x, y, z;
        public Vector3d(double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
        public UnityEngine.Vector3 ToUnity() => new UnityEngine.Vector3((float)x, (float)y, (float)z);
    }

    // === Crear motores ===

    [DllImport(DLL)] public static extern GeoMotor geoalg_motor_identity();
    [DllImport(DLL)] public static extern GeoMotor geoalg_motor_from_translation(double dx, double dy, double dz);
    [DllImport(DLL)] public static extern GeoMotor geoalg_motor_from_rotation(double ax, double ay, double az, double angle);

    // === Componer y transformar ===

    [DllImport(DLL)] public static extern GeoMotor geoalg_motor_compose(ref GeoMotor a, ref GeoMotor b);
    [DllImport(DLL)] public static extern GeoVec3 geoalg_motor_apply_point(ref GeoMotor m, double x, double y, double z);

    // === Convertir ===

    [DllImport(DLL)] public static extern GeoMat4 geoalg_motor_to_mat4(ref GeoMotor m);
    [DllImport(DLL)] public static extern GeoMotor geoalg_mat4_to_motor(ref GeoMat4 m);
    [DllImport(DLL)] public static extern GeoVec3 geoalg_motor_to_pose(ref GeoMotor m);
    [DllImport(DLL)] public static extern GeoQuat geoalg_motor_to_quat(ref GeoMotor m);
    [DllImport(DLL)] public static extern GeoMotor geoalg_pose_to_motor(double px, double py, double pz, double qw, double qx, double qy, double qz);

    // === Helpers para Unity ===

    /// <summary>Motor → UnityEngine.Transform (position + rotation)</summary>
    public static void ApplyToTransform(ref GeoMotor m, UnityEngine.Transform t)
    {
        var pos = geoalg_motor_to_pose(ref m);
        var quat = geoalg_motor_to_quat(ref m);
        t.position = new UnityEngine.Vector3((float)pos.x, (float)pos.y, (float)pos.z);
        t.rotation = new UnityEngine.Quaternion((float)quat.x, (float)quat.y, (float)quat.z, (float)quat.w);
    }

    /// <summary>Interpolación suave entre dos motores (t en [0,1])</summary>
    public static GeoMotor Lerp(ref GeoMotor a, ref GeoMotor b, double t)
    {
        // Nota: requiere geoalg_motor_between en el FFI (ver abajo).
        // Por ahora usa pose + quaternion como aproximación.
        var posA = geoalg_motor_to_pose(ref a);
        var posB = geoalg_motor_to_pose(ref b);
        var quatA = geoalg_motor_to_quat(ref a);
        var quatB = geoalg_motor_to_quat(ref b);

        double px = posA.x + (posB.x - posA.x) * t;
        double py = posA.y + (posB.y - posA.y) * t;
        double pz = posA.z + (posB.z - posA.z) * t;

        // Slerp quaternion (Unity tiene uno built-in).
        var qa = new UnityEngine.Quaternion((float)quatA.x, (float)quatA.y, (float)quatA.z, (float)quatA.w);
        var qb = new UnityEngine.Quaternion((float)quatB.x, (float)quatB.y, (float)quatB.z, (float)quatB.w);
        var qs = UnityEngine.Quaternion.Slerp(qa, qb, (float)t);

        return geoalg_pose_to_motor(px, py, pz, qs.w, qs.x, qs.y, qs.z);
    }
}