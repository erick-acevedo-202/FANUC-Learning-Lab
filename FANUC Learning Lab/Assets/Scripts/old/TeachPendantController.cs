using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;


//SCRIPT CONTROLADOR DEL TP. Se descarto por que cada que presionas 1 boton se hace 1 movimiento, entonces
//  tienes que presionar mucas veces para llevar el robot a algun lado.
//Se reemplazo por TPController

public class TeachPendantController : MonoBehaviour
{

    string idButton;

    public int velocity = 5;

    public TextMeshProUGUI velocityText;

    private int shiftPressed = 0; // 0 == off, 1 == on

    //  HIGHLIGH Shift on Click
    public Image onClicSL;
    public Image onClicSR;

    // Move Joints Variables
    public Transform[] joints;

    //Eache Joint Limits
    //                              J1    J2     J3     J4     J5      J6
    private float[] minLimits = { -720f, -80f, -60f, -190f, -125f, -360f };
    private float[] maxLimits = { 720f, 60f, 170f, 190f, 125f, 360f };

    private float[] jointAngles = new float[6];


    // Start is called before the first frame update
    void Start()
    {

        velocityText.text = $"{velocity}";


    }

    // Update is called once per frame
    void Update()
    {

    }

    //    receives:                   -1 = Negative Movement,     1 = Positive Movement
    public void MoveJoints(int jointIndex, int direction)
    {
        float jointVelocity = (float)velocity;

        float nextJointAngle = jointAngles[jointIndex] + (direction * jointVelocity);

        if (nextJointAngle > maxLimits[jointIndex])
        {
            jointAngles[jointIndex] = maxLimits[jointIndex];
            Debug.Log($"Joint Max Angle Limit {maxLimits[jointIndex]} Reached");
        }
        else if (nextJointAngle < minLimits[jointIndex])
        {
            jointAngles[jointIndex] = minLimits[jointIndex];
            Debug.Log($"Joint Min Angle Limit {minLimits[jointIndex]} Reached");
        }
        else
        {
            jointAngles[jointIndex] += direction * jointVelocity;

        }

        // JointIndex == 0 -> Joint1

        //      J2, J3 and J5   ---->   Rotate in "Y" axis
        if (jointIndex == 1 || jointIndex == 2 || jointIndex == 4)
        {
            //Change from accomulative rotation to absolute rotation
            //joints[jointIndex].rotation = joints[jointIndex].rotation * Quaternion.Euler(0.0f, jointAngles[jointIndex] ,0.0f);
            joints[jointIndex].localRotation = Quaternion.Euler(0.0f, jointAngles[jointIndex], 0.0f);
        }
        //      J4, J6      ---->    Rotate in "X" axis   
        else if (jointIndex == 3 || jointIndex == 5)
        {
            //joints[jointIndex].rotation = joints[jointIndex].rotation * Quaternion.Euler(jointAngles[jointIndex], 0.0f ,0.0f);
            joints[jointIndex].localRotation = Quaternion.Euler(jointAngles[jointIndex], 0.0f, 0.0f);

        }
        //  J1  ---->   Rotate in "Z" axis
        else if (jointIndex == 0)
        {
            //joints[jointIndex].rotation = joints[jointIndex].rotation * Quaternion.Euler( 0.0f ,0.0f, jointAngles[jointIndex] );
            joints[jointIndex].localRotation = Quaternion.Euler(0.0f, 0.0f, jointAngles[jointIndex]);
        }
        else
        {
            // This Joint isn´t programmed Yet
            Debug.Log($"Joint Index: {jointIndex} Out Of Range :(");
        }

        /*
        foreach(float i in jointAngles){
            Debug.Log($"{i}");
        }
        */

    }


    public void OnButtonClicked(string idButton)
    {
        /*  Input ex. nJ1
        n/p =   Negative / Positive
        J1  =   Joint Number
        S =   shift right or left
        incVel/decVel   =   Increment / Decrese Velocity
        */
        Debug.Log($"Button {idButton} pressed");


        switch (idButton)
        {
            case ("SL"):
                shiftPressed = 1 - shiftPressed; // 1 <---> 0
                Debug.Log($"Shift State changed to: {shiftPressed}");
                if (shiftPressed == 1)
                {
                    onClicSL.color = new Color32(30, 40, 50, 200);  //Highligh button to show the user that is currently pressed
                }
                else
                {
                    onClicSL.color = new Color32(30, 40, 50, 0);
                }

                break;

            case ("SR"):
                shiftPressed = 1 - shiftPressed; // 1 <---> 0
                Debug.Log($"Shift State changed to: {shiftPressed}");
                if (shiftPressed == 1)
                {
                    onClicSR.color = new Color32(30, 40, 50, 200);  //Highligh button to show the user that is currently pressed
                }
                else
                {
                    onClicSR.color = new Color32(30, 40, 50, 0);
                }

                break;

            case ("incVel"):
                if (shiftPressed == 1)
                {

                    velocity = (velocity < 100 && velocity >= 50) ? 100 :
                                (velocity < 50 && velocity >= 1) ? 50 : velocity;
                    //(velocity<5 && velocity>=1) ? 5 : velocity ;
                }
                else
                {
                    velocity = (velocity == 1) ? 5 :
                                (velocity > 1 && velocity <= 95) ? velocity += 5 : velocity;
                }
                velocityText.text = $"{velocity}";
                break;

            case ("decVel"):
                if (shiftPressed == 1)
                {

                    velocity = (velocity <= 50 && velocity > 1) ? 1 :
                                (velocity <= 100 && velocity > 50) ? 50 : velocity;
                }
                else
                {
                    velocity = (velocity == 5) ? 1 :
                                (velocity > 5 && velocity <= 100) ? velocity -= 5 : velocity;
                }
                velocityText.text = $"{velocity}";
                break;

            /*  MOVING JOINTS   */

            case ("nJ1"):
                MoveJoints(0, -1);
                /*
                if(counterJ1 > -720){
                    // rango de movimiento +-720°
                    counterJ1 -= jointVelocity;
                    //Debug.Log($"{counterVelocity}");
                    jointJ1.rotation = jointJ1.rotation * Quaternion.Euler(0.0f, 0.0f, -jointVelocity);
                }
                else{
                    Debug.Log($"Joint 1 Out of Range{counterJ1}. Limit -720");
                }
                */


                break;

            case ("pJ1"):
                MoveJoints(0, 1);
                /*
                if(counterJ1 < 720){
                    counterJ1 += jointVelocity;
                    jointJ1.rotation = jointJ1.rotation * Quaternion.Euler(0.0f, 0.0f, jointVelocity);
                }
                else{
                    Debug.Log($"Joint 1 Out of Range{counterJ1}. Limit 720");
                }
                */
                break;

            case ("nJ2"):
                MoveJoints(1, -1);
                /*
                if(counterJ2 > -90){
                    counterJ2-=jointVelocity;
                    jointJ2.rotation = jointJ2.rotation * Quaternion.Euler(0.0f, -jointVelocity, 0.0f);
                }
                else{
                    Debug.Log($"Joint 2 Out of Range{counterJ2}. Limit -90");
                }
                */

                break;

            case ("pJ2"):
                MoveJoints(1, 1);
                /*
                if(counterJ2 < 135){
                    counterJ2+=jointVelocity;
                    jointJ2.rotation = jointJ2.rotation * Quaternion.Euler(0.0f, jointVelocity, 0.0f);
                }
                else{
                    Debug.Log($"Joint 2 Out of Range{counterJ2}. Limit 135");
                }
                */

                break;

            case ("nJ3"):
                MoveJoints(2, -1);
                //jointJ3.rotation = jointJ3.rotation * Quaternion.Euler(0.0f, -jointVelocity, 0.0f);
                break;

            case ("pJ3"):
                MoveJoints(2, 1);
                //jointJ3.rotation = jointJ3.rotation * Quaternion.Euler(0.0f, jointVelocity, 0.0f);
                break;

            case ("nJ4"):
                MoveJoints(3, -1);
                //jointJ4.rotation = jointJ4.rotation * Quaternion.Euler(-jointVelocity, 0.0f, 0.0f);
                break;

            case ("pJ4"):
                MoveJoints(3, 1);
                //jointJ4.rotation = jointJ4.rotation * Quaternion.Euler(jointVelocity, 0.0f, 0.0f);
                break;

            case ("nJ5"):
                MoveJoints(4, -1);
                //jointJ5.rotation = jointJ5.rotation * Quaternion.Euler(0.0f, -jointVelocity, 0.0f);
                break;

            case ("pJ5"):
                MoveJoints(4, 1);
                //jointJ5.rotation = jointJ5.rotation * Quaternion.Euler(0.0f, jointVelocity, 0.0f);
                break;

            case ("nJ6"):
                MoveJoints(5, -1);
                //jointJ6.rotation = jointJ6.rotation * Quaternion.Euler(-jointVelocity, 0.0f, 0.0f);
                break;

            case ("pJ6"):
                MoveJoints(5, 1);
                //jointJ6.rotation = jointJ6.rotation * Quaternion.Euler(jointVelocity, 0.0f, 0.0f);
                break;

            default:
                break;


        }
        /*
        if(idButton == "SL" || idButton == "SR"){
            shiftPressed = 1 - shiftPressed; // 1 <---> 0
            Debug.Log($"Shift State changed to: {shiftPressed}");
        }
        */
    }



}
