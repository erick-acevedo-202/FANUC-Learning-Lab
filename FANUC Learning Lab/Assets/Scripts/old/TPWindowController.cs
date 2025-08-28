using UnityEngine;
using UnityEngine.UI;
using UnityEngine.EventSystems;

public class TPWindowController : MonoBehaviour
{
    private RectTransform rectTransform;
    private bool isMinimized = false;
    private Vector2 originalSize;

    [Header("Referencias")]
    public Button ButtonClose;
    public Button ButtonMinimize;
    public GameObject contenidoVentana;

    void Start()
    {
        rectTransform = GetComponent<RectTransform>();
        originalSize = rectTransform.sizeDelta;

        if (ButtonClose != null) ButtonClose.onClick.AddListener(CerrarVentana);
        if (ButtonMinimize != null) ButtonMinimize.onClick.AddListener(MinimizarVentana);

        // Configura el EventTrigger para arrastrar (opcional)
        EventTrigger trigger = gameObject.AddComponent<EventTrigger>();
        EventTrigger.Entry dragEntry = new EventTrigger.Entry();
        dragEntry.eventID = EventTriggerType.Drag;
        dragEntry.callback.AddListener((data) => { OnDrag((PointerEventData)data); });
        trigger.triggers.Add(dragEntry);
    }

    public void CerrarVentana()
    {
        gameObject.SetActive(false);
    }

    public void MinimizarVentana()
    {
        isMinimized = !isMinimized;

        if (isMinimized)
        {
            rectTransform.sizeDelta = new Vector2(originalSize.x, 50);
            if (contenidoVentana != null) contenidoVentana.SetActive(false);
        }
        else
        {
            rectTransform.sizeDelta = originalSize;
            if (contenidoVentana != null) contenidoVentana.SetActive(true);
        }
    }

    public void OnDrag(PointerEventData eventData)
    {
        rectTransform.anchoredPosition += eventData.delta;
        // Limita la posición a la pantalla
        Vector2 clampedPosition = rectTransform.anchoredPosition;
        clampedPosition.x = Mathf.Clamp(clampedPosition.x, 0, Screen.width - rectTransform.sizeDelta.x);
        clampedPosition.y = Mathf.Clamp(clampedPosition.y, 0, Screen.height - rectTransform.sizeDelta.y);
        rectTransform.anchoredPosition = clampedPosition;
    }

    public void OpenWindow() => gameObject.SetActive(true);

    void Awake() => DontDestroyOnLoad(gameObject);
}