using System;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class TPController : MonoBehaviour
{
    

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

    [Header("IK Integration")]
    public IKController ikController;
    public Vector3 targetPosition;
    public Quaternion targetOrientation;

    private bool isWorldMode = false; // Track mode

    [Header("Target Integration")]
    public Transform endEfectorTarget;

    void Start()
    {
        if (endEfectorTarget == null)
            endEfectorTarget = GameObject.Find("EndEfector").transform;
        if (ikController == null) ikController = GetComponent<IKController>();

        // Inicializar target en una posición alcanzable
        endEfectorTarget.position = new Vector3(0f, 0f, 0.4f); // 400 mm en Z
        endEfectorTarget.rotation = Quaternion.Euler(0f, 0f, 0f); // Resetear orientación
        targetPosition = endEfectorTarget.position;
        targetOrientation = endEfectorTarget.rotation;
        isWorldMode = (mode == "WORLD");

        for (int i = 0; i < 6; i++)
        {
            if (joints[i] != null) jointAngles[i] = joints[i].localEulerAngles.y;
        }
        modeText.text = mode;
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
        Quaternion rotation;

        switch (jointIndex)
        {
            case 0:                   
                rotation = Quaternion.Euler(0f, -jointAngles[jointIndex], 0f);
                break;
            case 1:
                rotation = Quaternion.Euler(0f, 0f, -jointAngles[jointIndex]);
                break;
            case 2:
            case 4:
                rotation = Quaternion.Euler(0f, 0f, jointAngles[jointIndex]);
                break;
            case 3:
            case 5:
                rotation = Quaternion.Euler(jointAngles[jointIndex], 0f, 0f);
                break;
            default:
                Debug.LogWarning($"Joint Index {jointIndex} is not configured.");
                return;
        }

        joints[jointIndex].localRotation = rotation;
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
        isWorldMode = (mode == "WORLD");
    }

    /// <summary>
    /// Determines joint and direction from button ID and moves it.
    /// </summary>
    /*private void HandleJointMovement(string idButton)
    {
        if (idButton.Length != 3 || (idButton[0] != 'n' && idButton[0] != 'p') || idButton[1] != 'J') return;

        int direction = idButton[0] == 'n' ? -1 : 1;
        if (int.TryParse(idButton[2].ToString(), out int jointIndex) && jointIndex >= 1 && jointIndex <= 6)
        {
            MoveJoints(jointIndex - 1, direction);
        }
    }*/
    private void HandleJointMovement(string idButton)
    {
        if (idButton.Length != 3 || (idButton[0] != 'n' && idButton[0] != 'p') || idButton[1] != 'J') return;

        int direction = idButton[0] == 'n' ? -1 : 1;
        if (int.TryParse(idButton[2].ToString(), out int jointIndex) && jointIndex >= 1 && jointIndex <= 6)
        {
            if (isWorldMode)
            {
                if (LshiftPressed || RshiftPressed)
                {
                    float delta = direction * (velocity / 10f) * Time.deltaTime;
                    int axis = jointIndex - 1;
                    if (axis < 3)
                    {
                        Vector3 moveDir = (axis == 0 ? Vector3.right : (axis == 1 ? Vector3.up : Vector3.forward));
                        endEfectorTarget.position += moveDir * delta;
                    }
                    else
                    {
                        axis -= 3;
                        Vector3 rotDir = (axis == 0 ? Vector3.right : (axis == 1 ? Vector3.up : Vector3.forward));
                        float deltaAngle = delta * Mathf.Rad2Deg * 10f;
                        endEfectorTarget.rotation = Quaternion.AngleAxis(deltaAngle, rotDir) * endEfectorTarget.rotation;
                    }
                    targetPosition = endEfectorTarget.position;
                    targetOrientation = endEfectorTarget.rotation;
                    Debug.Log($"Target Position: {targetPosition * 1000f} mm, Orientation: {targetOrientation.eulerAngles}");

                    float[] newAngles = ikController.SolveIK(targetPosition, targetOrientation, jointAngles, minLimits, maxLimits);
                    if (newAngles != null)
                    {
                        jointAngles = newAngles;
                        for (int i = 0; i < 6; i++)
                        {
                            ApplyJointMove(i);
                        }
                    }
                    else
                    {
                        Debug.LogWarning("No valid IK solution found. Target out of reach or limits exceeded.");
                    }
                }
                else
                {
                    Debug.Log("Shift must be Pressed");
                }
            }
            else
            {
                MoveJoints(jointIndex - 1, direction);
            }
        }
    }
}
