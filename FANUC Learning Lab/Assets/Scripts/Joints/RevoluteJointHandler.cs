using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RevoluteJointHandler : JointHandler
{

    public bool drawFrame = true;
    public float frameLen = 0.05f; // 5cm

    // Llamado por RDL
    public void SetJointValue(HomogenousTransformation T,
                              HomogenousTransformation Tnext,
                              Link link, double q)
    {
        // Pose local del joint i respecto a su padre (i-1)
        var p = T.GetPosition().ToVector3();
        var R = T.GetRotation().ToUnityMatrix().rotation;

        transform.localPosition = p;
        transform.localRotation = R;
    }

    private void OnDrawGizmos()
    {
        if (!drawFrame) return;

        // Dibuja ejes locales del pivot para inspección
        var p = transform.position;
        var right = transform.right * frameLen;
        var up = transform.up * frameLen;
        var fwd = transform.forward * frameLen;

        Debug.DrawLine(p, p + right, Color.red);
        Debug.DrawLine(p, p + up, Color.green);
        Debug.DrawLine(p, p + fwd, Color.blue);
    }

    /*
    public void Awake()
    {
        if(RootObject == null)
        {
            RootObject = this.gameObject;
        }
    }

    public override void SetJointValue(HomogenousTransformation transformation, HomogenousTransformation nextTransformation, Link link, double q)
    {
        
         //CODIGO ORIGINAL
        Vector p = nextTransformation.GetPosition();
        RootObject.transform.localPosition = p.ToVector3();
        RotationMatrix R = nextTransformation.GetRotation();
        RootObject.transform.localRotation = R.ToUnityMatrix().rotation;

        // SUGERENCIA DE OTRA IA
       Vector p = transformation.GetPosition();
       RootObject.transform.localPosition = p.ToVector3();

       RotationMatrix R = transformation.GetRotation();
       RootObject.transform.localRotation = R.ToUnityMatrix().rotation;*/

        /*
        // GROK code
        print("q = " + q);
        // Obtener el eje de rotación desde el Link
        Vector axis = link.GetN(); // Devuelve (1,0,0) para 'x', (0,1,0) para 'y', (0,0,1) para 'z'
        if (axis == null)
        {
            print("Axis not defined for link: " + RootObject.name);
            return;
        }
         
        // Convertir q (radianes) a grados para Unity
        float angleDegrees = (float)(q * Mathf.Rad2Deg);

        // Crear una rotación en Unity alrededor del eje local
        Quaternion rotation = Quaternion.AngleAxis(angleDegrees, new Vector3((float)axis.X, (float)axis.Y, (float)axis.Z));

        // Aplicar la rotación al GameObject localmente (relativo al padre)
        RootObject.transform.localRotation = rotation;

        // No ajustar localPosition aquí, ya que las posiciones están fijas por los offsets en FanucLRMate.cso
        
        
    }

    public GameObject RootObject;
    */
}
