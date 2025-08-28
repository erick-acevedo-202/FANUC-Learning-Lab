using UnityEngine;
using UnityEngine.EventSystems;


public class ButtonController : MonoBehaviour, IPointerDownHandler, IPointerUpHandler
{
    public string buttonID; // Identificador único del botón
    private bool _isPressed = false;

    public bool IsPressed
    {
        
        get { return _isPressed; }
        
    }

    public void OnPointerDown(PointerEventData eventData)
    {
        Debug.Log("From Button Controller:  IS PRESSED");
        _isPressed = true;
    }

    public void OnPointerUp(PointerEventData eventData)
    {
        _isPressed = false;
    }

    // Opcional: Para manejar también el teclado/Gamepad
    void OnDisable()
    {
        _isPressed = false;
    }
}