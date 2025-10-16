using RobotDynamics.Controller;
using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections.Generic;
using UnityEngine;

public class RevoluteRobot : RobotBase
{
    // (opcional) si la usas en tu UI
    public List<double> angles;

    // ===== Telemetría IK =====
    public bool logIK = true;
    public bool drawEEPath = true;
    public int maxIter = 200;       // si más adelante haces bucle manual
    public float tol = 1e-3f;       // tolerancia pos (m)

    // Asignar en Inspector
    public Transform endEffector;   // último link / brida
    public Transform target;        // objetivo de IK

    private Vector3 _lastEEPos;
    private Quaternion _lastEERot;

    // Si luego quieres medir Δq, guarda aquí tu q anterior (rellénalo desde tu flujo)
    private double[] _qPrev;   // opcional

    private Matrix _alpha;
    void Awake()
    {
        Robot = new FanucLRMate();
        _alpha = Matrix.Identity(6);
    }

    void Start()
    {
        if (endEffector != null)
        {
            _lastEEPos = endEffector.position;
            _lastEERot = endEffector.rotation;
        }

        ValidateRig(); // te avisa pivots con offsets/escala != 1
    }

    void Update()
    {
        if (EnableForwardKinematics)
        {
            SetQ(ForwardKinematicsQ);
        }

        if (EnableInverseKinematics && target != null && endEffector != null)
        {
            // --- Estado "antes" (para medir el paso del frame) ---
            Vector3 eePosBefore = endEffector.position;
            Quaternion eeRotBefore = endEffector.rotation;

            // === 1) Ejecuta un paso de IK con RDL (tu flujo actual) ===
            FollowTargetOneStep(target.gameObject);

            // === 2) Telemetría de convergencia por frame ===

            // Pose actual del EE (mundo) DESPUÉS del paso
            Vector3 eePos = endEffector.position;
            Quaternion eeRot = endEffector.rotation;

            // Objetivo (mundo)
            Vector3 targetPos = target.position;
            Quaternion targetRot = target.rotation;

            // Error de posición
            Vector3 posErr = targetPos - eePos;

            // Error de orientación (cuaternión relativo)
            Quaternion dq = targetRot * Quaternion.Inverse(eeRot);
            dq.ToAngleAxis(out float angDeg, out Vector3 axis);
            if (float.IsNaN(axis.x)) axis = Vector3.zero; // por ángulos ~0
            Vector3 angErr = axis.normalized * (angDeg * Mathf.Deg2Rad);

            // Tamaño del paso APROXIMADO este frame (por EE)
            float stepPos = (eePos - eePosBefore).magnitude;
            float stepAngDeg = Quaternion.Angle(eeRotBefore, eeRot); // en grados

            LogIKStepFrame(posErr, angErr, stepPos, stepAngDeg);

            // Paro "suave" por tolerancias (no uses break en Update)
            if (posErr.magnitude < tol && angErr.magnitude < (2f * Mathf.Deg2Rad))
            {
                // Opcional: desactiva IK si alcanzaste el objetivo
                // EnableInverseKinematics = false;
            }

            // Guarda estado para dibujar estela
            _lastEEPos = eePos;
            _lastEERot = eeRot;
        }

        Robot.JointController.ReportNewFrame(Time.deltaTime);
    }

    void LateUpdate()
    {
        if (drawEEPath && endEffector != null)
        {
            Debug.DrawLine(_lastEEPos, endEffector.position, Color.magenta);
            _lastEEPos = endEffector.position;
        }
    }

    void LogIKStepFrame(Vector3 posErr, Vector3 angErrRad, float stepPos, float stepAngDeg)
    {
        if (!logIK) return;
        Debug.Log(
            $"IK frame | posErr={posErr.magnitude:F5} m | angErr={angErrRad.magnitude * Mathf.Rad2Deg:F3} deg | " +
            $"stepEE={stepPos:F5} m, {stepAngDeg:F3} deg");
    }

    // ===== Utilidades de validación del rig =====
    void ValidateRig()
    {
        if (Joints == null) return;
        foreach (var j in Joints)
        {
            var t = j.transform;
            if (t.localScale != Vector3.one)
                Debug.LogWarning($"[Rig] {t.name} localScale != 1 → {t.localScale}");
            if (!ApproxZero(t.localPosition) || !ApproxZeroEuler(t.localEulerAngles))
                Debug.LogWarning($"[Rig] {t.name} no está en cero. Pos={t.localPosition}, Rot(Euler)={t.localEulerAngles}");
        }
    }

    bool ApproxZero(Vector3 v) =>
        Mathf.Abs(v.x) < 1e-6f && Mathf.Abs(v.y) < 1e-6f && Mathf.Abs(v.z) < 1e-6f;

    bool ApproxZeroEuler(Vector3 e) =>
        Mathf.Abs(Mathf.DeltaAngle(0, e.x)) < 1e-4f &&
        Mathf.Abs(Mathf.DeltaAngle(0, e.y)) < 1e-4f &&
        Mathf.Abs(Mathf.DeltaAngle(0, e.z)) < 1e-4f;
}

/*      ORIGINAL
using RobotDynamics.Controller;
using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class RevoluteRobot : RobotBase
{
    public List<double> angles;
    public GameObject Target;

    public bool logIK = true;
    public bool drawEEPath = true;
    public int maxIter = 200;
    public float tol = 1e-3f;

    private Vector3 lastEEPos;


    // Start is called before the first frame update
    void Awake()
    {
        Robot = new FanucLRMate();
        //Robot.AddLinearJoint(new Vector(0, 0, 0), new Vector(0, 1, 0));
    }

    // Update is called once per frame
    void Update()
    {
        if (EnableForwardKinematics)
        {
            SetQ(ForwardKinematicsQ);
        }

        if (EnableInverseKinematics)
        {
            //print("Starting IK on FANUC LR Mate");
            FollowTargetOneStep(Target);
            
            //DebugIK();
            //OnDrawGizmos();
             
        }

        Robot.JointController.ReportNewFrame(Time.deltaTime);
    }
   

   
}
*/
