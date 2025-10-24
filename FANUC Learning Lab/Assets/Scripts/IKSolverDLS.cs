using System;
using UnityEngine;

public static class IKSolverDLS
{
    // --- Utilidades ---
    static double Deg2Rad(double d) => d * Math.PI / 180.0;
    static double Rad2Deg(double r) => r * 180.0 / Math.PI;

    static double[,] RotX(double a)
    {
        double ca = Math.Cos(a), sa = Math.Sin(a);
        return new double[,] { { 1, 0, 0, 0 }, { 0, ca, -sa, 0 }, { 0, sa, ca, 0 }, { 0, 0, 0, 1 } };
    }
    static double[,] RotY(double a)
    {
        double ca = Math.Cos(a), sa = Math.Sin(a);
        return new double[,] { { ca, 0, sa, 0 }, { 0, 1, 0, 0 }, { -sa, 0, ca, 0 }, { 0, 0, 0, 1 } };
    }
    static double[,] RotZ(double a)
    {
        double ca = Math.Cos(a), sa = Math.Sin(a);
        return new double[,] { { ca, -sa, 0, 0 }, { sa, ca, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
    }
    static double[,] Tx(double a)
    {
        return new double[,] { { 1, 0, 0, a }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
    }
    static double[,] Tz(double d)
    {
        return new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, d }, { 0, 0, 0, 1 } };
    }
    static double[,] MatMul(double[,] A, double[,] B)
    {
        int r = A.GetLength(0), c = B.GetLength(1), n = A.GetLength(1);
        var R = new double[r, c];
        for (int i = 0; i < r; i++) for (int j = 0; j < c; j++)
            {
                double s = 0; for (int k = 0; k < n; k++) s += A[i, k] * B[k, j];
                R[i, j] = s;
            }
        return R;
    }
    static double[,] DH(double th, double d, double a, double al)
    {
        // DH clásico: Rz(th)*Tz(d)*Tx(a)*Rx(al)
        return MatMul(MatMul(MatMul(RotZ(th), Tz(d)), Tx(a)), RotX(al));
    }

    // --- Rotaciones / conversiones ---
    static (double roll, double pitch, double yaw) RotmToEulerZYX(double[,] R)
    {
        double r11 = R[0, 0], r12 = R[0, 1], r13 = R[0, 2];
        double r21 = R[1, 0], r22 = R[1, 1], r23 = R[1, 2];
        double r31 = R[2, 0], r32 = R[2, 1], r33 = R[2, 2];
        if (Math.Abs(r31) != 1.0)
        {
            double pitch = -Math.Asin(r31);
            double cos_p = Math.Cos(pitch);
            double roll = Math.Atan2(r32 / cos_p, r33 / cos_p);
            double yaw = Math.Atan2(r21 / cos_p, r11 / cos_p);
            return (roll, pitch, yaw);
        }
        else
        {
            double yaw = 0.0, pitch, roll;
            if (r31 == -1.0)
            {
                pitch = Math.PI / 2;
                roll = yaw + Math.Atan2(r12, r13);
            }
            else
            {
                pitch = -Math.PI / 2;
                roll = -yaw + Math.Atan2(-r12, -r13);
            }
            return (roll, pitch, yaw);
        }
    }
    static double[,] EulerZYX_ToRotm(double yaw, double pitch, double roll)
    {
        // Z(yaw) * Y(pitch) * X(roll)
        var Rz = new double[,] {
            {Math.Cos(yaw), -Math.Sin(yaw), 0},
            {Math.Sin(yaw),  Math.Cos(yaw), 0},
            {0,0,1}
        };
        var Ry = new double[,] {
            { Math.Cos(pitch), 0, Math.Sin(pitch)},
            {0,1,0},
            {-Math.Sin(pitch),0, Math.Cos(pitch)}
        };
        var Rx = new double[,] {
            {1,0,0},
            {0, Math.Cos(roll), -Math.Sin(roll)},
            {0, Math.Sin(roll),  Math.Cos(roll)}
        };
        return MatMul3(Rz, Ry, Rx);
    }
    static double[,] MatMul3(double[,] A, double[,] B, double[,] C) => MatSub(MatMul(MatUp(A), MatUp(B)), C); // helper
    static double[,] MatUp(double[,] X) => X; static double[,] MatSub(double[,] AB, double[,] C) => MatMul(AB, C);

    static double[] RotmToRotvec(double[,] R3)
    {
        // Versión 3x3
        double tr = R3[0, 0] + R3[1, 1] + R3[2, 2];
        double cos_ang = (tr - 1.0) / 2.0;
        cos_ang = Math.Max(-1.0, Math.Min(1.0, cos_ang));
        double angle = Math.Acos(cos_ang);
        if (Math.Abs(angle) < 1e-8) return new double[] { 0, 0, 0 };
        double denom = 2.0 * Math.Sin(angle);
        double rx = (R3[2, 1] - R3[1, 2]) / denom;
        double ry = (R3[0, 2] - R3[2, 0]) / denom;
        double rz = (R3[1, 0] - R3[0, 1]) / denom;
        return new double[] { rx * angle, ry * angle, rz * angle };
    }

    // --- DH de tu código (mm, grados en offsets) ---
    static readonly double[] IMAGE_D = { 0.0, 0.0, 0.0, -320.0, 0.0, -80.0 };
    static readonly double[] IMAGE_A = { 75.0, 300.0, 75.0, 0.0, 0.0, 0.0 };
    static readonly double[] IMAGE_ALPHA = { -90.0, 180.0, -90.0, 90.0, -90.0, 0.0 }; // deg
    static readonly double[] THETA_OFFS = { 0.0, -90.0, 0.0, 0.0, 0.0, 0.0 }; // deg

    static readonly (double lo, double hi)[] JOINT_LIMITS_DEG = new (double, double)[] {
        (-180,180), (-60,60), (-74,74), (-90,90), (-90,90), (-180,180)
    };

    static void GetDH(out double[] d, out double[] a, out double[] alphaRad, out double[] offsRad)
    {
        d = (double[])IMAGE_D.Clone();
        a = (double[])IMAGE_A.Clone();
        alphaRad = new double[6];
        offsRad = new double[6];
        for (int i = 0; i < 6; i++) { alphaRad[i] = Deg2Rad(IMAGE_ALPHA[i]); offsRad[i] = Deg2Rad(THETA_OFFS[i]); }
    }

    // --- FK ---
    public static void ForwardKinematics(double[] jointsDeg,
                                         out double[,] T, out Vector3 pos_mm, out Vector3 rpy_deg)
    {
        GetDH(out var d, out var a, out var al, out var off);
        double[,] M = new double[,] { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };
        for (int i = 0; i < 6; i++)
        {
            double th = Deg2Rad(jointsDeg[i]) + off[i];
            var Ai = DH(th, d[i], a[i], al[i]);
            M = MatMul(M, Ai);
        }
        T = M;
        pos_mm = new Vector3((float)M[0, 3], (float)M[1, 3], (float)M[2, 3]);
        var (roll, pitch, yaw) = RotmToEulerZYX(new double[,] {
            {M[0,0],M[0,1],M[0,2]},
            {M[1,0],M[1,1],M[1,2]},
            {M[2,0],M[2,1],M[2,2]}
        });
        rpy_deg = new Vector3((float)Rad2Deg(roll), (float)Rad2Deg(pitch), (float)Rad2Deg(yaw));
    }

    // --- Chequeo de límites ---
    public static bool InLimits(double[] qDeg, out string report)
    {
        System.Text.StringBuilder sb = new();
        bool ok = true;
        for (int i = 0; i < 6; i++)
        {
            var (lo, hi) = JOINT_LIMITS_DEG[i];
            if (qDeg[i] < lo || qDeg[i] > hi)
            {
                ok = false; sb.AppendLine($"J{i + 1}={qDeg[i]:F3} fuera [{lo},{hi}]");
            }
        }
        report = sb.ToString();
        return ok;
    }

    // --- IK numérica (DLS sobre JJ^T) ---
    public static (bool ok, double[] qDeg, double residual, int iterations, string limReport)
        InverseKinematicsNumeric(
            Vector3 targetPos_mm,
            Vector3? targetRPY_deg,
            double[] initialGuessDeg,
            bool positionOnly = false,
            double tol_pos = 1e-3,
            double tol_ori = 1e-3,
            int max_iter = 400)
    {
        double[] theta = new double[6];
        if (initialGuessDeg != null) for (int i = 0; i < 6; i++) theta[i] = Deg2Rad(initialGuessDeg[i]);

        double[,] R_target = null;
        if (targetRPY_deg.HasValue)
        {
            var rpy = targetRPY_deg.Value;
            double roll = Deg2Rad(rpy.x), pitch = Deg2Rad(rpy.y), yaw = Deg2Rad(rpy.z);
            R_target = EulerZYX_ToRotm(yaw, pitch, roll);
        }

        double lam = 0.01;
        double[] posTarget = { targetPos_mm.x, targetPos_mm.y, targetPos_mm.z };
        double[] err = null;

        for (int it = 0; it < max_iter; it++)
        {
            double[] qDeg = new double[6];
            for (int i = 0; i < 6; i++) qDeg[i] = Rad2Deg(theta[i]);
            ForwardKinematics(qDeg, out var T, out var pos, out _);
            var Rcur = new double[,] {
                {T[0,0],T[0,1],T[0,2]},
                {T[1,0],T[1,1],T[1,2]},
                {T[2,0],T[2,1],T[2,2]}
            };

            double[] dp = { posTarget[0] - pos.x, posTarget[1] - pos.y, posTarget[2] - pos.z };
            double[] rot_err = { 0, 0, 0 };
            if (!positionOnly && R_target != null)
            {
                // R_err = R_target * Rcur^T
                var Rct = new double[,] {
                    {Rcur[0,0],Rcur[1,0],Rcur[2,0]},
                    {Rcur[0,1],Rcur[1,1],Rcur[2,1]},
                    {Rcur[0,2],Rcur[1,2],Rcur[2,2]}
                };
                var Rerr = MatMul(R_target, Rct);
                rot_err = RotmToRotvec(Rerr);
            }
            err = new double[] { dp[0], dp[1], dp[2], rot_err[0], rot_err[1], rot_err[2] };

            if (Norm(dp) < tol_pos && Norm(rot_err) < tol_ori)
            {
                var sol = new double[6]; for (int i = 0; i < 6; i++) sol[i] = Rad2Deg(theta[i]);
                bool okLimits = InLimits(sol, out var rep);
                return (okLimits, sol, Norm(err), it, rep);
            }

            // Jacobiano numérico
            double[,] J = new double[6, 6];
            double eps = 1e-6;
            for (int i = 0; i < 6; i++)
            {
                var pert = (double[])theta.Clone(); pert[i] += eps;
                var qDeg2 = new double[6]; for (int k = 0; k < 6; k++) qDeg2[k] = Rad2Deg(pert[k]);
                ForwardKinematics(qDeg2, out var T2, out var pos2, out _);
                var R2 = new double[,] {
                    {T2[0,0],T2[0,1],T2[0,2]},
                    {T2[1,0],T2[1,1],T2[1,2]},
                    {T2[2,0],T2[2,1],T2[2,2]}
                };
                double[] dp_d = { (pos2.x - pos.x) / eps, (pos2.y - pos.y) / eps, (pos2.z - pos.z) / eps };
                double[] drot_d = { 0, 0, 0 };
                if (!positionOnly && R_target != null)
                {
                    var Rerr2 = MatMul(R_target, new double[,] {
                        {R2[0,0],R2[1,0],R2[2,0]},
                        {R2[0,1],R2[1,1],R2[2,1]},
                        {R2[0,2],R2[1,2],R2[2,2]}
                    });
                    var rot_err2 = RotmToRotvec(Rerr2);
                    drot_d = new double[] { (rot_err2[0] - rot_err[0]) / eps, (rot_err2[1] - rot_err[1]) / eps, (rot_err2[2] - rot_err[2]) / eps };
                }
                for (int r = 0; r < 3; r++) J[r, i] = dp_d[r];
                for (int r = 0; r < 3; r++) J[3 + r, i] = drot_d[r];
            }

            // DLS: qdot = J^T (JJ^T + λ^2 I)^-1 * err
            var JJt = MatMul(J, Transpose(J));
            var lamI = Eye(6); Scale(lamI, lam * lam); var JJt_l = Add(JJt, lamI);
            var inv = Inv(JJt_l);
            var tmp = MatVec(Transpose(J), MatVec(inv, err));
            // Paso máximo
            double maxStep = Deg2Rad(10.0);
            double nrm = Norm(tmp);
            if (nrm > maxStep)
            {
                for (int i = 0; i < 6; i++) tmp[i] *= (maxStep / nrm);
            }
            for (int i = 0; i < 6; i++)
            {
                theta[i] += tmp[i];
                // wrap [-pi,pi]
                theta[i] = (theta[i] + Math.PI) % (2 * Math.PI) - Math.PI;
            }
        }

        var solEnd = new double[6]; for (int i2 = 0; i2 < 6; i2++) solEnd[i2] = Rad2Deg(theta[i2]);
        bool okL = InLimits(solEnd, out var repEnd);
        return (okL, solEnd, Norm(err), max_iter, repEnd);
    }

    // --- Álgebra auxiliar ---
    static double[,] Transpose(double[,] A)
    {
        int r = A.GetLength(0), c = A.GetLength(1);
        var R = new double[c, r];
        for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) R[j, i] = A[i, j];
        return R;
    }
    static double[,] Eye(int n)
    {
        var I = new double[n, n];
        for (int i = 0; i < n; i++) I[i, i] = 1;
        return I;
    }
    static void Scale(double[,] A, double s)
    {
        int r = A.GetLength(0), c = A.GetLength(1);
        for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) A[i, j] *= s;
    }
    static double[,] Add(double[,] A, double[,] B)
    {
        int r = A.GetLength(0), c = A.GetLength(1);
        var R = new double[r, c];
        for (int i = 0; i < r; i++) for (int j = 0; j < c; j++) R[i, j] = A[i, j] + B[i, j];
        return R;
    }
    static double[] MatVec(double[,] A, double[] x)
    {
        int r = A.GetLength(0), c = A.GetLength(1);
        var y = new double[r];
        for (int i = 0; i < r; i++)
        {
            double s = 0; for (int j = 0; j < c; j++) s += A[i, j] * x[j];
            y[i] = s;
        }
        return y;
    }
    static double Norm(double[] v) { double s = 0; for (int i = 0; i < v.Length; i++) s += v[i] * v[i]; return Math.Sqrt(s); }
    static double[,] Inv(double[,] M)
    {
        // Pequeño inversor genérico 6x6 vía Gauss-Jordan
        int n = M.GetLength(0);
        var A = new double[n, n]; var I = Eye(n);
        Array.Copy(M, A, M.Length);
        for (int i = 0; i < n; i++)
        {
            // pivot
            int piv = i; double best = Math.Abs(A[i, i]);
            for (int r = i + 1; r < n; r++) { double v = Math.Abs(A[r, i]); if (v > best) { best = v; piv = r; } }
            if (piv != i) { for (int c = 0; c < n; c++) { (A[i, c], A[piv, c]) = (A[piv, c], A[i, c]); (I[i, c], I[piv, c]) = (I[piv, c], I[i, c]); } }
            double diag = A[i, i];
            double invDiag = 1.0 / diag;
            for (int c = 0; c < n; c++) { A[i, c] *= invDiag; I[i, c] *= invDiag; }
            for (int r = 0; r < n; r++) if (r != i)
                {
                    double f = A[r, i];
                    for (int c = 0; c < n; c++) { A[r, c] -= f * A[i, c]; I[r, c] -= f * I[i, c]; }
                }
        }
        return I;
    }
}