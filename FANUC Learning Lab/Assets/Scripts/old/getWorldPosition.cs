using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class getWorldPosition : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        Debug.Log($"{gameObject.name} World Position: {transform.position}");
        Debug.Log($"{gameObject.name} World Position: {transform.rotation}");
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
