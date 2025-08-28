using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class IK : MonoBehaviour
{
    public Transform endEffector;
    public Transform target;
    public Transform[] joints; // Desde la base hasta el end effector
    
    public int maxIterations = 10;
    public float threshold = 0.01f;


    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }
    void LateUpdate()
    {

        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            // Empezar desde el último joint hacia el primero
            for (int i = joints.Length - 2; i >= 0; i--)
            {
                Transform joint = joints[i];

                Vector3 toEndEffector = endEffector.position - joint.position;
                Vector3 toTarget = target.position - joint.position;

                float angle = Vector3.Angle(toEndEffector, toTarget);
                Vector3 cross = Vector3.Cross(toEndEffector, toTarget);
                joint.rotation = Quaternion.AngleAxis(angle, cross.normalized) * joint.rotation;

                if ((endEffector.position - target.position).magnitude < threshold)
                    return; // ya llegó
            }
        }
    }
}
