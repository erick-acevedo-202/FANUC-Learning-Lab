using UnityEngine;

public class WorldMotionController : MonoBehaviour
{ 
    [Header("Refs")]
    public TPController tp;           // tu TPController existente
    public Transform effectorTCP;     // tu effector (TCP real)

    [Header("Opcional: Shift (deadman)")]
    public bool requireShift = true;
    public ButtonController shiftLeft;   // si usas botones de shift, asígnalos
    public ButtonController shiftRight;  // si no, deja null y pon requireShift=false

    [Header("WORLD jog (mm por frame a OVRD=100)")]
    public float worldStepAt100 = 5f; // mm/frame cuando velocity=100

    void Update()
    {
        if (tp == null || effectorTCP == null) return;

        // Solo en modo WORLD (tu TP ya alterna JOINT/WORLD desde el botón coord)
        if (tp.mode != "WORLD") return;

        // Deadman (Shift) opcional
        if (requireShift)
        {
            bool l = (shiftLeft != null && shiftLeft.IsPressed);
            bool r = (shiftRight != null && shiftRight.IsPressed);
            if (!(l || r)) return;
        }

        // Magnitud por frame, escalada por override 1..100 que ya maneja tu TP
        float step = worldStepAt100 * Mathf.Clamp(tp.velocity, 1f, 100f) / 100f;

        // 1) Leer tus botones existentes (pJ1/nJ1, pJ2/nJ2, pJ3/nJ3) y mapear a WORLD
        Vector3 dW = Vector3.zero; // delta en WORLD (mm/frame)
        foreach (var b in tp.blueButtons)
        {
            if (b == null || !b.IsPressed) continue;

            switch (b.buttonID)
            {
                // J1 ↔ Xw
                case "pJ1": dW.x += step; break;
                case "nJ1": dW.x -= step; break;

                // J2 ↔ Yw
                case "pJ2": dW.y += step; break;
                case "nJ2": dW.y -= step; break;

                // J3 ↔ Zw
                case "pJ3": dW.z += step; break;
                case "nJ3": dW.z -= step; break;

                    // J4–J6: reservados para R/P/Y (cuando los actives)
                    // case "pJ4": ...; case "nJ4": ...;
                    // case "pJ5": ...; case "nJ5": ...;
                    // case "pJ6": ...; case "nJ6": ...;
            }
        }
        if (dW == Vector3.zero) return;

        // 2) Pos actual del TCP en UNITY (m) -> mm
        Vector3 pU_mm = effectorTCP.position * 1000f;

        // 3) Convertir a WORLD (según tu convención Xw=front, Zw=up, Yw=lateral) y sumar delta
        Vector3 pW_mm = FrameAdapter.UnityToWorld_Pos_mm(pU_mm);
        Vector3 targetW_mm = pW_mm + dW;

        // 4) Semilla = q actual desde tu TP
        double[] qSeed = new double[6];
        for (int i = 0; i < 6; i++) qSeed[i] = tp.jointAngles[i];

        // 5) IK solo posición (orientación la activamos después)
        var ik = IKSolverDLS.InverseKinematicsNumeric(
            targetW_mm,
            null,              // RPY world (null → posición solamente)
            qSeed,
            positionOnly: true,
            tol_pos: 1e-3, tol_ori: 1e-3, max_iter: 400
        );

        if (!ik.ok)
        {
            // Mantén pose y avisa; límites/tolerancias pueden ajustarse en IKSolverDLS
            Debug.LogWarning($"IK no convergió o excede límites. it={ik.iterations}, res={ik.residual:F6} {ik.limReport}");
            return;
        }

        // 6) Aplicar solución con tu pipeline (respeta límites en TPController)
        for (int j = 0; j < 6; j++)
        {
            tp.jointAngles[j] = (float)ik.qDeg[j];
            tp.ApplyJointMove(j);
        }
    }
}