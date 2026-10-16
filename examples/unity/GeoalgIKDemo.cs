// GeoalgIKDemo.cs: prueba de geoalg en Unity.
//
// Que hace:
//   - Arma el brazo demo de geoalg (3 articulaciones, tramos de 1).
//   - Crea una esfera rosa: el OBJETIVO. Muevela en la escena.
//   - En cada cuadro, la IK de geoalg calcula los angulos para que la
//     punta llegue a la esfera, y el brazo se dibuja con esferas y cilindros.
//   - La punta se pinta VERDE si llego, y ROJA si no alcanza.
//
// Como usarlo:
//   1. Copia geoalg_ffi.dll a Assets/Plugins/.
//   2. Copia GeoalgRobot.cs y este archivo a Assets/Scripts/.
//   3. Crea un GameObject vacio en la escena y agregale este script.
//   4. Dale Play y mueve la esfera "Geoalg Target" con la herramienta de mover.
//
// Ejes: el brazo demo esta descrito con Z hacia arriba (como ROS),
// asi que se convierte con FromZUp / ToZUp.

using UnityEngine;

public class GeoalgIKDemo : MonoBehaviour
{
    [Tooltip("El objetivo. Si lo dejas vacio, se crea una esfera rosa.")]
    public Transform target;

    [Tooltip("Vueltas maximas de la IK en cada cuadro.")]
    public uint maxIterations = 100;

    GeoalgRobot robot;
    double[] q;
    Transform[] joints;
    Transform[] links;
    Renderer tipRenderer;
    bool converged;
    double lastError;
    string status = "Iniciando...";

    void Start()
    {
        try
        {
            robot = GeoalgRobot.DemoArm();
        }
        catch (System.DllNotFoundException)
        {
            status = "No se encontro geoalg_ffi.dll. Copiala a Assets/Plugins/ y reinicia Unity.";
            Debug.LogError(status);
            enabled = false;
            return;
        }

        int n = robot.JointCount;
        q = new double[n];

        if (target == null)
        {
            var t = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            t.name = "Geoalg Target";
            t.transform.localScale = Vector3.one * 0.2f;
            t.transform.position = transform.position + new Vector3(1.2f, 1.5f, 0.6f);
            t.GetComponent<Renderer>().material.color = new Color(1f, 0.3f, 0.7f);
            target = t.transform;
        }

        joints = new Transform[n + 1];
        for (int i = 0; i <= n; i++)
        {
            var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s.name = i == n ? "Tip" : "Joint " + i;
            s.transform.SetParent(transform, false);
            s.transform.localScale = Vector3.one * (i == n ? 0.14f : 0.18f);
            s.GetComponent<Renderer>().material.color = i == n ? Color.green : new Color(0.9f, 0.5f, 0.1f);
            Destroy(s.GetComponent<Collider>());
            joints[i] = s.transform;
        }
        tipRenderer = joints[n].GetComponent<Renderer>();

        links = new Transform[n];
        for (int i = 0; i < n; i++)
        {
            var c = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            c.name = "Link " + i;
            c.transform.SetParent(transform, false);
            c.GetComponent<Renderer>().material.color = new Color(0.3f, 0.55f, 0.9f);
            Destroy(c.GetComponent<Collider>());
            links[i] = c.transform;
        }

        status = "geoalg listo: " + n + " articulaciones";
        Debug.Log(status);
    }

    void Update()
    {
        if (robot == null || target == null) return;

        // Objetivo de Unity -> coordenadas de geoalg (Z arriba).
        double tx, ty, tz;
        GeoalgRobot.ToZUp(target.position - transform.position, out tx, out ty, out tz);

        // IK: empieza desde la postura anterior, asi el movimiento es continuo.
        var res = robot.SolveIK(tx, ty, tz, q, maxIterations, 1e-6);
        q = res.Angles;
        converged = res.Converged;
        lastError = res.Error;

        // FK: posiciones de base, articulaciones y punta, para dibujar.
        double[] pts = robot.Forward(q);
        for (int i = 0; i < joints.Length; i++)
        {
            joints[i].position = transform.position
                + GeoalgRobot.FromZUp(pts[i * 3], pts[i * 3 + 1], pts[i * 3 + 2]);
        }
        for (int i = 0; i < links.Length; i++)
        {
            Vector3 a = joints[i].position;
            Vector3 b = joints[i + 1].position;
            Vector3 d = b - a;
            float len = d.magnitude;
            links[i].gameObject.SetActive(len > 1e-4f);
            if (len <= 1e-4f) continue;
            links[i].position = (a + b) * 0.5f;
            links[i].rotation = Quaternion.FromToRotation(Vector3.up, d / len);
            // El cilindro de Unity mide 2 de alto: la escala Y es la mitad del largo.
            links[i].localScale = new Vector3(0.09f, len * 0.5f, 0.09f);
        }

        tipRenderer.material.color = converged ? Color.green : Color.red;
        status = converged
            ? "Llego (error " + lastError.ToString("0.0e0") + ")"
            : "No alcanza: queda a " + lastError.ToString("0.000");
    }

    void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 600, 25), "geoalg IK: " + status);
        if (q != null)
        {
            string a = "Angulos:";
            for (int i = 0; i < q.Length; i++) a += " " + q[i].ToString("0.000");
            GUI.Label(new Rect(10, 30, 600, 25), a);
        }
    }

    void OnDestroy()
    {
        if (robot != null) robot.Dispose();
    }
}
