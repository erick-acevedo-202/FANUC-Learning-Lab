using System;
using System.Collections.Generic;
using UnityEngine;

public class IKController : MonoBehaviour
{
    [Header("Link Lengths (m, from DH table and diagram)")]
    public float horizontalOffset = 0.075f; // ai1 = 75 mm, offset horizontal J1
    public float verticalOffset = 0.33f;    // d1 = 330 mm, offset vertical base to J2
    public float L2 = 0.3f;                 // ai2 = 300 mm, upper arm
    public float a3 = 0.075f;               // ai3 = 75 mm, forearm horizontal
    public float d4 = -0.32f;               // di4 = -320 mm, wrist vertical offset
    public float L5 = -0.08f;               // di6 = -80 mm, tool offset along Z

    private float[] minLimits;
    private float[] maxLimits;
    private float[] currentAngles;

    public float[] SolveIK(Vector3 targetPos, Quaternion targetRot, float[] currAngles, float[] mins, float[] maxs)
    {
        minLimits = mins;
        maxLimits = maxs;
        currentAngles = currAngles;

        Vector3 wc = targetPos - L5 * (targetRot * Vector3.forward);
        Debug.Log($"Wrist Center (wc): {wc}, Target Pos: {targetPos}, Tool Offset: {L5}");

        float wx = wc.x, wy = wc.y, wz = wc.z;
        Debug.Log($"Initial wx, wy, wz: {wx}, {wy}, {wz}");

        wz -= verticalOffset;
        Debug.Log($"Adjusted wz (after verticalOffset): {wz}");

        List<float[]> candidates = new List<float[]>();

        float th1 = (float)Math.Atan2(wy, wx);
        AddArmSolutions(wx, wy, wz, th1, false, candidates);

        float th1_flip = (float)Math.Atan2(-wy, -wx) + (float)Math.PI;
        AddArmSolutions(wx, wy, wz, th1_flip, true, candidates);

        List<float[]> allSolutions = new List<float[]>();
        foreach (var arm in candidates)
        {
            float th1_arm = arm[0], th2 = arm[1], th3 = arm[2];
            AddWristSolutions(th1_arm, th2, th3, targetRot, allSolutions);
        }

        float[] best = FindBestSolution(allSolutions, currentAngles);
        Debug.Log($"Candidates: {candidates.Count}, Solutions: {allSolutions.Count}, Best: {(best != null ? "Found" : "None")}");
        return best;
    }

    private void AddArmSolutions(float wx, float wy, float wz, float th1, bool isFlip, List<float[]> candidates)
    {
        float rho = (float)Math.Sqrt(wx * wx + wy * wy);
        float sign = isFlip ? 1f : -1f;
        float rho_effective = rho + sign * horizontalOffset;
        if (rho_effective < 0)
        {
            Debug.LogWarning($"rho_effective negative: {rho_effective}, rho: {rho}, offset: {horizontalOffset}");
            return;
        }
        Debug.Log($"rho_effective: {rho_effective}");

        float h = wz;
        float r = (float)Math.Sqrt(rho_effective * rho_effective + h * h);
        Debug.Log($"h: {h}, r: {r}");

        float L3 = (float)Math.Sqrt(a3 * a3 + d4 * d4);
        float D = (r * r - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        if (Math.Abs(D) > 1)
        {
            Debug.LogWarning($"D out of range: {D}, r: {r}, L2: {L2}, L3: {L3}");
            return;
        }
        Debug.Log($"D: {D}");

        float cos_y = D;
        float sen_y = (float)Math.Sqrt(1 - D * D);

        float alpha = (float)Math.Atan2(Math.Abs(d4), a3);

        // Elbow down
        float y_pos = (float)Math.Atan2(sen_y, cos_y);
        float th3_pos = alpha - y_pos;
        float sinSigma = a3 * (float)Math.Sin(th3_pos) + Math.Abs(d4) * (float)Math.Cos(th3_pos);
        float cosSigma = L2 + a3 * (float)Math.Cos(th3_pos) - Math.Abs(d4) * (float)Math.Sin(th3_pos);
        float th2_pos = (float)Math.Atan2(h, rho_effective) - (float)Math.Atan2(sinSigma, cosSigma);
        if (isFlip || wy < 0)
        {
            th2_pos += (float)Math.PI;
            th3_pos += (float)Math.PI;
        }
        candidates.Add(new float[] { th1, th2_pos, th3_pos });

        // Elbow up
        float y_neg = (float)Math.Atan2(-sen_y, cos_y);
        float th3_neg = alpha - y_neg;
        sinSigma = a3 * (float)Math.Sin(th3_neg) + Math.Abs(d4) * (float)Math.Cos(th3_neg);
        cosSigma = L2 + a3 * (float)Math.Cos(th3_neg) - Math.Abs(d4) * (float)Math.Sin(th3_neg);
        float th2_neg = (float)Math.Atan2(h, rho_effective) - (float)Math.Atan2(sinSigma, cosSigma);
        if (isFlip || wy < 0)
        {
            th2_neg += (float)Math.PI;
            th3_neg += (float)Math.PI;
        }
        candidates.Add(new float[] { th1, th2_neg, th3_neg });
    }


private void AddWristSolutions(float th1, float th2, float th3, Quaternion targetRot, List<float[]> solutions)
    {
        float[,] R03 = ComputeR03(th1, th2, th3);
        float[,] R03T = Transpose(R03);
        float[,] R06 = GetRotationMatrix(targetRot);
        float[,] R36 = MatrixMultiply(R03T, R06);

        float sin5 = -R36[1, 2];
        if (Math.Abs(sin5) > 1) return;

        // Primary solution
        float th5 = (float)Math.Asin(sin5);
        AddWristIfValid(th1, th2, th3, th5, R36, solutions);

        // Alternative wrist (flip)
        float th5_alt = (float)Math.PI - th5;
        AddWristIfValid(th1, th2, th3, th5_alt, R36, solutions);
    }

    private void AddWristIfValid(float th1, float th2, float th3, float th5, float[,] R36, List<float[]> solutions)
    {
        float cos5 = (float)Math.Cos(th5);
        if (Math.Abs(cos5) < 1e-6f) return; // Skip degenerate for simplicity

        float th6 = (float)Math.Atan2(R36[2, 2] / cos5, R36[0, 2] / cos5);
        float th4 = (float)Math.Atan2(R36[1, 0] / cos5, R36[0, 0] / cos5);

        // Convert to degrees
        float[] angles = new float[6];
        angles[0] = th1 * Mathf.Rad2Deg;
        angles[1] = th2 * Mathf.Rad2Deg;
        angles[2] = th3 * Mathf.Rad2Deg;
        angles[3] = th4 * Mathf.Rad2Deg;
        angles[4] = th5 * Mathf.Rad2Deg;
        angles[5] = th6 * Mathf.Rad2Deg;

        // Clamp to limits and validate
        bool valid = true;
        float[] originalAngles = new float[] { th1 * Mathf.Rad2Deg, th2 * Mathf.Rad2Deg, th3 * Mathf.Rad2Deg, th4 * Mathf.Rad2Deg, th5 * Mathf.Rad2Deg, th6 * Mathf.Rad2Deg };
        for (int i = 0; i < 6; i++)
        {
            angles[i] = Mathf.Clamp(angles[i], minLimits[i], maxLimits[i]);
            // Check if original angle was outside limits
            if (originalAngles[i] < minLimits[i] - 1e-3 || originalAngles[i] > maxLimits[i] + 1e-3)
            {
                valid = false;
            }
        }
        if (valid) solutions.Add(angles);
    }

    private float[] FindBestSolution(List<float[]> sols, float[] curr)
    {
        if (sols.Count == 0) return null;

        float minErr = float.MaxValue;
        float[] best = null;
        for (int i = 0; i < sols.Count; i++)
        {
            float err = 0f;
            for (int j = 0; j < 6; j++)
            {
                float d = sols[i][j] - curr[j];
                err += d * d;
            }
            if (err < minErr)
            {
                minErr = err;
                best = (float[])sols[i].Clone();
            }
        }
        return best;
    }

    // Matrix utilities
    private float[,] MatrixMultiply(float[,] a, float[,] b)
    {
        float[,] c = new float[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                for (int k = 0; k < 3; k++)
                    c[i, j] += a[i, k] * b[k, j];
        return c;
    }

    private float[,] Transpose(float[,] m)
    {
        float[,] t = new float[3, 3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                t[i, j] = m[j, i];
        return t;
    }

    private float[,] GetRotationMatrix(Quaternion q)
    {
        Vector3 x = q * Vector3.right;
        Vector3 y = q * Vector3.up;
        Vector3 z = q * Vector3.forward;
        return new float[,] {
            { x.x, y.x, z.x },
            { x.y, y.y, z.y },
            { x.z, y.z, z.z }
        };
    }

    private float[,] ComputeR03(float th1, float th2, float th3)
    {
        float[,] R01 = GetRot01(th1);
        float[,] R12 = GetRot12(th2);
        float[,] R23 = GetRot23(th3);
        float[,] temp = MatrixMultiply(R12, R23);
        return MatrixMultiply(R01, temp);
    }

    private float[,] GetRot01(float th)
    {
        float c = (float)Math.Cos(th), s = (float)Math.Sin(th);
        return new float[,] { { c, 0, -s }, { s, 0, c }, { 0, -1, 0 } };
    }

    private float[,] GetRot12(float th)
    {
        float c = (float)Math.Cos(th), s = (float)Math.Sin(th);
        return new float[,] { { c, -s, 0 }, { s, c, 0 }, { 0, 0, -1 } };
    }

    private float[,] GetRot23(float th)
    {
        float c = (float)Math.Cos(th), s = (float)Math.Sin(th);
        return new float[,] { { c, 0, s }, { s, 0, -c }, { 0, -1, 0 } };
    }
}