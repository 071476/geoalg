// GeoalgRobot.cs: usar los robots de geoalg desde C# (Unity o .NET).
//
// Requiere la libreria nativa compilada:
//   Windows: geoalg_ffi.dll   (cargo build -p geoalg-ffi --release)
// En Unity, copiala a Assets/Plugins/.
//
// IMPORTANTE (ejes): geoalg NO tiene un "arriba" fijo: depende de como
// describas el robot. Siempre usa la regla de la MANO DERECHA.
// Unity usa Y hacia arriba y la MANO IZQUIERDA. Elige la conversion
// segun como describiste tu robot:
//   - Robot con Z hacia arriba (ROS, URDF, el brazo demo): FromZUp / ToZUp.
//   - Robot con Y hacia arriba (los ejemplos del studio):   FromYUp / ToYUp.
// Para dibujar el giro de una articulacion en Unity: convierte su eje
// con la misma funcion y usa el angulo con signo CONTRARIO.

using System;
using System.Runtime.InteropServices;
#if UNITY_5_3_OR_NEWER
using UnityEngine;
#endif

public static class GeoalgNative
{
    const string Lib = "geoalg_ffi";
    const CallingConvention CC = CallingConvention.Cdecl;

    public const int OK = 0;
    public const int ERR_NULL = -1;
    public const int ERR_ARGUMENT = -2;
    public const int ERR_INTERNAL = -99;

    [DllImport(Lib, CallingConvention = CC)] public static extern IntPtr geoalg_robot_new();
    [DllImport(Lib, CallingConvention = CC)] public static extern IntPtr geoalg_robot_demo_arm();
    [DllImport(Lib, CallingConvention = CC)] public static extern void geoalg_robot_free(IntPtr robot);
    [DllImport(Lib, CallingConvention = CC)] public static extern int geoalg_robot_joint_count(IntPtr robot);

    [DllImport(Lib, CallingConvention = CC)]
    public static extern int geoalg_robot_add_revolute(IntPtr robot,
        double dx, double dy, double dz, double ox, double oy, double oz, double min, double max);

    [DllImport(Lib, CallingConvention = CC)]
    public static extern int geoalg_robot_add_prismatic(IntPtr robot,
        double dx, double dy, double dz, double ox, double oy, double oz, double min, double max);

    [DllImport(Lib, CallingConvention = CC)]
    public static extern int geoalg_robot_fk(IntPtr robot,
        double[] q, UIntPtr n, [Out] double[] outPoints, UIntPtr outLen);

    [DllImport(Lib, CallingConvention = CC)]
    public static extern int geoalg_robot_ik(IntPtr robot,
        double tx, double ty, double tz, double[] q0, UIntPtr n,
        uint maxIter, double tol, [Out] double[] outQ, out double outError);
}

/// Resultado de la IK.
public struct GeoalgIKResult
{
    public bool Converged;
    public double[] Angles;
    public double Error;
}

/// Un robot de geoalg. Usalo con `using` o llama Dispose() al terminar.
public sealed class GeoalgRobot : IDisposable
{
    IntPtr handle;

    GeoalgRobot(IntPtr h)
    {
        if (h == IntPtr.Zero) throw new InvalidOperationException("geoalg no pudo crear el robot");
        handle = h;
    }

    public static GeoalgRobot Empty() { return new GeoalgRobot(GeoalgNative.geoalg_robot_new()); }

    /// El brazo demo: base que gira en Z y dos articulaciones en X, tramos de 1 (Z hacia arriba).
    public static GeoalgRobot DemoArm() { return new GeoalgRobot(GeoalgNative.geoalg_robot_demo_arm()); }

    public int JointCount { get { return Check(GeoalgNative.geoalg_robot_joint_count(Live())); } }

    /// Articulacion que gira. Angulos en radianes. Ejes en coordenadas de geoalg.
    public void AddRevolute(double dx, double dy, double dz, double ox, double oy, double oz, double min, double max)
    {
        Check(GeoalgNative.geoalg_robot_add_revolute(Live(), dx, dy, dz, ox, oy, oz, min, max));
    }

    /// Articulacion que desliza.
    public void AddPrismatic(double dx, double dy, double dz, double ox, double oy, double oz, double min, double max)
    {
        Check(GeoalgNative.geoalg_robot_add_prismatic(Live(), dx, dy, dz, ox, oy, oz, min, max));
    }

    /// FK: devuelve (n + 1) puntos x, y, z seguidos (base, articulaciones, punta).
    public double[] Forward(double[] q)
    {
        var outPts = new double[(q.Length + 1) * 3];
        Check(GeoalgNative.geoalg_robot_fk(Live(), q, (UIntPtr)q.Length, outPts, (UIntPtr)outPts.Length));
        return outPts;
    }

    /// IK de posicion: llevar la punta a (tx, ty, tz) en coordenadas de geoalg.
    public GeoalgIKResult SolveIK(double tx, double ty, double tz, double[] q0, uint maxIter = 200, double tol = 1e-6)
    {
        var outQ = new double[q0.Length];
        double err;
        int r = GeoalgNative.geoalg_robot_ik(Live(), tx, ty, tz, q0, (UIntPtr)q0.Length, maxIter, tol, outQ, out err);
        if (r < 0) Check(r);
        return new GeoalgIKResult { Converged = r == 1, Angles = outQ, Error = err };
    }

    public void Dispose()
    {
        if (handle != IntPtr.Zero)
        {
            GeoalgNative.geoalg_robot_free(handle);
            handle = IntPtr.Zero;
        }
        GC.SuppressFinalize(this);
    }

    ~GeoalgRobot() { Dispose(); }

    IntPtr Live()
    {
        if (handle == IntPtr.Zero) throw new ObjectDisposedException("GeoalgRobot");
        return handle;
    }

    static int Check(int code)
    {
        if (code >= 0) return code;
        switch (code)
        {
            case GeoalgNative.ERR_NULL: throw new InvalidOperationException("geoalg: puntero vacio");
            case GeoalgNative.ERR_ARGUMENT: throw new ArgumentException("geoalg: argumento invalido (NaN, tamano equivocado o min > max)");
            default: throw new InvalidOperationException("geoalg: error interno " + code);
        }
    }

#if UNITY_5_3_OR_NEWER
    // ---- Robots descritos con Z hacia arriba (ROS, URDF, brazo demo) ----

    /// geoalg (Z arriba, mano derecha) -> Unity (Y arriba, mano izquierda).
    public static Vector3 FromZUp(double x, double y, double z)
    {
        return new Vector3((float)x, (float)z, (float)y);
    }

    /// Unity -> geoalg (Z arriba).
    public static void ToZUp(Vector3 v, out double x, out double y, out double z)
    {
        x = v.x; y = v.z; z = v.y;
    }

    // ---- Robots descritos con Y hacia arriba (ejemplos del studio, three.js) ----

    /// geoalg (Y arriba, mano derecha) -> Unity (Y arriba, mano izquierda).
    public static Vector3 FromYUp(double x, double y, double z)
    {
        return new Vector3((float)x, (float)y, (float)-z);
    }

    /// Unity -> geoalg (Y arriba).
    public static void ToYUp(Vector3 v, out double x, out double y, out double z)
    {
        x = v.x; y = v.y; z = -v.z;
    }
#endif
}
