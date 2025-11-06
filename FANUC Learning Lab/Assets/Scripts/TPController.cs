using System;
using Preliy.Flange;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class TPController : MonoBehaviour
{
    [Header("Kinematics Controllers")]
    public Controller kinematics_controller;

    // --- IK / Cartesian ---
    public Robot6RSphericalWrist robot;     // arrástralo (el mismo del robot)
    public Transform baseFrame;              // Frame "Base" del robot
    public Transform flangeFrame;            // Frame "Flange" (TCP actual)

    [Header("Cartesian Steps")]
    public float posStepMm = 5f;             // paso en mm para X/Y/Z
    public float rotStepDeg = 2f;            // paso en grados para Rx/Ry/Rz

    private Matrix4x4 _targetBase;

    [Header("Velocity Settings")]
    [Range(1, 100)]
    public int velocity = 5;
    public TextMeshProUGUI velocityText;

    [Header("Mode Settings")]
    public string mode = "JOINT";
    public TextMeshProUGUI modeText;

    [Header("Shift Button Highlight")]
    public Image onClickSL;
    public Image onClickSR;

    [Header("Joints Configuration")]
    public Transform[] joints;
    private readonly float[] minLimits = { -720f, -80f, -60f, -190f, -125f, -360f };
    private readonly float[] maxLimits = { 720f, 60f, 170f, 190f, 125f, 360f };
    private float[] jointAngles = new float[6]; //Currennt Joint Positions 

    public Transform effector;

    [Header("Button Controllers")]
    public ButtonController[] blueButtons;

    private bool LshiftPressed = false;
    private bool RshiftPressed = false;

    void Awake()
    {
        if (kinematics_controller == null)
            kinematics_controller = FindObjectOfType<Controller>();

        if (robot == null)
            robot = FindObjectOfType<Robot6RSphericalWrist>();

        // Base->TCP inicial
        var T_base_w = Matrix4x4.TRS(baseFrame.position, baseFrame.rotation, Vector3.one);
        var T_tcp_w = Matrix4x4.TRS(flangeFrame.position, flangeFrame.rotation, Vector3.one);
        _targetBase = T_base_w.inverse * T_tcp_w;
    }


    void Update()
    {
        
        foreach (var button in blueButtons)
        {
            if (button.IsPressed)
            {
                OnButtonClicked(button.buttonID);
            }
        }


    }

    /// <summary>
    /// Moves the specified joint in the given direction.
    /// </summary>
    public void MoveJoints(int jointIndex, int direction)
    {
        float jointVelocity = velocity;
        float nextAngle = jointAngles[jointIndex] + direction * jointVelocity/100; //ChECAR ESTO

        

        if (LshiftPressed || RshiftPressed)
        {
            // Clamp angle to defined limits
            if (nextAngle > maxLimits[jointIndex])
            {
                jointAngles[jointIndex] = maxLimits[jointIndex];
                Debug.Log($"Joint Max Angle Limit {maxLimits[jointIndex]} Reached");
            }
            else if (nextAngle < minLimits[jointIndex])
            {
                jointAngles[jointIndex] = minLimits[jointIndex];
                Debug.Log($"Joint Min Angle Limit {minLimits[jointIndex]} Reached");
            }
            else
            {
                jointAngles[jointIndex] = nextAngle;
            }

            ApplyJointMove(jointIndex);
        }
        else
        {
            Debug.Log($"Shift must be Pressed");
        }

        
    }

    /// <summary>
    /// Applies rotation to a joint based on joint index and stored angle.
    /// </summary>
    public void ApplyJointMove(int jointIndex)
    {
        var tj = kinematics_controller.MechanicalGroup.Joints[jointIndex];
        // Usa tu arreglo jointAngles[] que ya calculas
        var nextDeg = jointAngles[jointIndex];

        // (opcional) clamp con límites reales del joint de la lib
        float limMin = tj.Config.Limits.x;
        float limMax = tj.Config.Limits.y;
        nextDeg = Mathf.Clamp(nextDeg, limMin, limMax);

        // ¡listo! esto mueve el slider y la FK
        tj.Position.Value = nextDeg;
        
    }

    /// <summary>
    /// Handles all button logic based on the ID passed.
    /// </summary>
    public void OnButtonClicked(string idButton)
    {
        switch (idButton)
        {
            case "SL":
                ToggleShift("L", onClickSL);
                break;

            case "SR":
                ToggleShift("R", onClickSR);
                break;

            case "incVel":
                IncreaseVelocity();
                break;

            case "decVel":
                DecreaseVelocity();
                break;

            case "coord":
                ToggleMode();
                break;

            //DEBUGGEAR (comprobar erroes)
            case "joints":
                foreach (float angle in jointAngles)
                    Debug.Log(angle);
                break;

            case "effector":
                Debug.Log("EFFECTOR   ");
                Debug.Log($"Position: {effector.position}");
                Debug.Log($"Rotation: {effector.rotation}");
                Debug.Log($"Velocity: {velocity.ToString()}");

                break;

            default:
                HandleJointMovement(idButton);
                break;
        }
    }

    /// <summary>
    /// Toggles the shift mode and highlights the button.
    /// </summary>
    private void ToggleShift(string shiftType, Image image)
    {
        bool isPressed;

        if (shiftType == "L")
        {
            LshiftPressed = !LshiftPressed;
            isPressed = LshiftPressed;
        }
        else // shiftType == "R"
        {
            RshiftPressed = !RshiftPressed;
            isPressed = RshiftPressed;
        }

        Debug.Log($"Shift {shiftType} state changed to: {isPressed}");

        // Actualizar color
        image.color = isPressed
            ? new Color32(30, 40, 50, 200)
            : new Color32(30, 40, 50, 0);
    }

    /// <summary>
    /// Increases movement velocity with or without shift.
    /// </summary>
    private void IncreaseVelocity()
    {
        if (LshiftPressed || RshiftPressed)
        {
            velocity = velocity < 50 ? 50 : (velocity < 100 ? 100 : velocity);
        }
        else
        {
            velocity = velocity == 1 ? 5 : (velocity <= 95 ? velocity + 5 : velocity);
        }
        UpdateVelocityText();
    }

    /// <summary>
    /// Decreases movement velocity with or without shift.
    /// </summary>
    private void DecreaseVelocity()
    {
        if (LshiftPressed || RshiftPressed)
        {
            velocity = velocity > 50 ? 50 : (velocity > 1 ? 1 : velocity);
        }
        else
        {
            velocity = velocity == 5 ? 1 : (velocity > 5 ? velocity - 5 : velocity);
        }
        UpdateVelocityText();
    }

    /// <summary>
    /// Updates the velocity display.
    /// </summary>
    private void UpdateVelocityText()
    {
        velocityText.text = velocity.ToString();
    }

    /// <summary>
    /// Toggles between JOINT and WORLD mode.
    /// </summary>
    private void ToggleMode()
    {
        mode = (mode == "JOINT") ? "WORLD" : "JOINT";
        modeText.text = mode;
    }

    /// <summary>
    /// Determines joint and direction from button ID and moves it.
    /// </summary>
    private void HandleJointMovement(string idButton)
    {
        if (idButton.Length != 3 || (idButton[0] != 'n' && idButton[0] != 'p') || idButton[1] != 'J') return;

        int direction = idButton[0] == 'n' ? -1 : 1;
        if (int.TryParse(idButton[2].ToString(), out int jointIndex) && jointIndex >= 1 && jointIndex <= 6)
        {
            if(mode == "JOINT")
            {
                MoveJoints(jointIndex - 1, direction);
            }
            else
            {

                Debug.Log("Inverse Kinematics HERE");
                //MoveWorld(jointIndex, direction);
            }
            
        }
    }

    // jointIndex 1..6 mapea a: X, Y, Z, Rx, Ry, Rz
    private void MoveWorld(int jointIndex1Based, int direction)
    {
        if (jointIndex1Based < 1 || jointIndex1Based > 6) return;

        // velocidad afecta el tamaño del paso (opcional)
        float posStep = posStepMm * Mathf.Max(1, velocity) / 50f;   // mm
        float rotStep = rotStepDeg * Mathf.Max(1, velocity) / 50f;  // deg

        switch (jointIndex1Based)
        {
            case 1: NudgePos(new Vector3(direction * posStep, 0f, 0f)); break; // X
            case 2: NudgePos(new Vector3(0f, direction * posStep, 0f)); break; // Y
            case 3: NudgePos(new Vector3(0f, 0f, direction * posStep)); break; // Z
            case 4: NudgeRot(new Vector3(direction * rotStep, 0f, 0f)); break; // Rx
            case 5: NudgeRot(new Vector3(0f, direction * rotStep, 0f)); break; // Ry
            case 6: NudgeRot(new Vector3(0f, 0f, direction * rotStep)); break; // Rz
        }
    }

    // Traslación en marco Base (como gizmo Move)
    private void NudgePos(Vector3 deltaMm)
    {
        Vector3 deltaM = deltaMm * 0.001f; // mm -> m
        _targetBase = Matrix4x4.Translate(deltaM) * _targetBase; // pre-multiplica (ejes de Base)
        SolveAndApplyIK();
    }

    // Rotación intrínseca alrededor del TCP (como gizmo Rotate)
    private void NudgeRot(Vector3 deltaDeg)
    {
        _targetBase = _targetBase * Matrix4x4.Rotate(Quaternion.Euler(deltaDeg)); // post-multiplica
        SolveAndApplyIK();
    }

    private void SolveAndApplyIK()
    {
        var cfg = kinematics_controller.Configuration.Value;
        var ik = robot.ComputeInverse(_targetBase, cfg, SolutionIgnoreMask.None);

        if (!ik.IsValid)
        {
            Debug.LogWarning($"IK inválida");
            return;
        }

        var joints = kinematics_controller.MechanicalGroup.Joints;

        // 6 DOF; evita usar .Count en JointTarget
        int n = Mathf.Min(joints.Count, 6);
        for (int i = 0; i < n; i++)
        {
            float deg = ik.JointTarget[i];   // acceso por indexador

            Vector2 lim = joints[i].Config.Limits;
            deg = Mathf.Clamp(deg, lim.x, lim.y);

            joints[i].Position.Value = deg;
        }

        // (opcional) mantiene el gizmo pegado a la solución en mundo
        var T_base_w = Matrix4x4.TRS(baseFrame.position, baseFrame.rotation, Vector3.one);
        var T_tcp_w = T_base_w * _targetBase;
        flangeFrame.SetPositionAndRotation(T_tcp_w.GetColumn(3), T_tcp_w.rotation);
    }


}
