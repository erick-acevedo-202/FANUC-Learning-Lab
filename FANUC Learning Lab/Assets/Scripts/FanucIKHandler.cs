using RobotDynamics.Controller;
using RobotDynamics.MathUtilities;
using RobotDynamics.Robots;
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FanucIKHandler : MonoBehaviour
{
    public List<double> angles;
    public GameObject Target;
    private Robot Fanuc; 


    // Start is called before the first frame update
    void Awake()
    {
        Fanuc = new FanucLRMate();
        
    }


    // Update is called once per frame

    void Update()
    {

    }
}

