using UnityEngine;

public static class FrameAdapter
{
    // Tu deseo: +X(world)=frente, +Z(world)=arriba, +Y(world)=lado
    // En Unity: frente=+Z, arriba=+Y, lado=+X
    // Mapeo posiciones (mm):
    //  Xw = Zu,  Yw = Xu,  Zw = Yu
    public static Vector3 WorldToUnity_Pos_mm(Vector3 pWorld_mm)
    {
        return new Vector3(pWorld_mm.y, pWorld_mm.z, pWorld_mm.x);
    }
    public static Vector3 UnityToWorld_Pos_mm(Vector3 pUnity_mm)
    {
        return new Vector3(pUnity_mm.z, pUnity_mm.x, pUnity_mm.y);
    }

    // Orientaciones (RPY ZYX en WORLD) -> aplico misma permuta a ejes.
    // Para robustez real se haría con matrices de cambio de base;
    // aquí dejamos helpers si luego habilitamos RPY en el pendant.
    public static Vector3 WorldToUnity_RPYdeg(Vector3 rpyWorld_deg)
    {
        // Placeholder de misma permuta (coherente con posiciones)
        return new Vector3(rpyWorld_deg.y, rpyWorld_deg.z, rpyWorld_deg.x);
    }
    public static Vector3 UnityToWorld_RPYdeg(Vector3 rpyUnity_deg)
    {
        return new Vector3(rpyUnity_deg.z, rpyUnity_deg.x, rpyUnity_deg.y);
    }
}