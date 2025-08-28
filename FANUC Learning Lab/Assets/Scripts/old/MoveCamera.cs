using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveCamera : MonoBehaviour
{

    [SerializeField]
    private float _mouseSensivity = 3.0f;

    private float _rotationY;
    private float _rotationX;

    [SerializeField]
    private Transform _target;

    [SerializeField]
    private float _distanceFromTarget = 1000.0f;





    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        float mouseX = Input.GetAxis("Mouse X") * _mouseSensivity;
        float mouseY = Input.GetAxis("Mouse Y") * _mouseSensivity;
        
        _rotationX += mouseX;
        _rotationY += mouseY;

        //Limit X rotation
        _rotationX = Mathf.Clamp(_rotationX, -40, 40);

        transform.localEulerAngles = new Vector3(_rotationX, _rotationY, 0);

        transform.position = _target.position - transform.forward * _distanceFromTarget;
    }
}
